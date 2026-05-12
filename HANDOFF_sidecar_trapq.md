# Handoff: move buffer-stepper chunks off the main toolhead timeline

**Audience**: the next agent picking this up. You are expected to **plan** (not yet implement) the change described below. Use the Plan agent and Explore agents as needed. Land the plan via `ExitPlanMode` before any edits.

**Branch state**: `claude/fix-buffer-log-messages-1Qr49`, currently at commit `04c44de`. That commit fixed four short-horizon issues (silent EMPTY-recovery stall, recovery overshoot, log noise, sensor-conflict races) but **explicitly punted** on the architectural issue described here.

---

## 1. The problem in one paragraph

`klipper/buffer.py` issues all its buffer-stepper motion (recovery fills, initial fill, safety retract, continuous manual feed, retract-until-clear) through `force_move.manual_move(stepper, dist, speed, accel)`. That helper ends with `self.toolhead.dwell(chunk_duration)` on the **main toolhead**. So every chunk reserves ~0.14 s of main-toolhead `print_time` — even though the buffer stepper is on its own custom trapq during the move. The main X/Y/Z/E steppers see a forced gap; main extrusion is paced by recovery; the gcode parser uses the slack to deepen the lookahead. A PAUSE during recovery captures `gcode_position` at the end of that inflated lookahead and resumes 100+ mm further along than the user expected. The whole point of unsyncing was to keep the buffer stepper **off** the print's critical path. The current implementation undermines that goal.

## 2. Why this is the right fix to make now

- Pause-overshoot is the user's most recent observation; this directly addresses it.
- Once buffer chunks no longer dwell the main toolhead, the residual pause-overshoot is just normal Klipper queue-drain — the user can fix it cleanly with `M400` at the top of their PAUSE macro. We don't need a `pause_drain_queue` knob or any plugin-level pause workaround.
- Main extrusion speed during EMPTY recovery is restored to baseline. Today recovery slows main extrusion by ~70% in steady state (chunks reserve more toolhead-time per second than wall-clock provides; the host throttles via `_check_pause` once `buffer_time_high ≈ 2 s` is hit).
- The five chunked-motion sites (recovery, initial fill, safety retract, continuous feed, retract-until-clear) share the same defect; a single mechanism fixes all of them.

## 3. Mechanical detail (read before planning)

`force_move.manual_move` in upstream Kalico/Klipper does (paraphrased):

```python
def manual_move(self, stepper, dist, speed, accel=0.):
    self.toolhead.flush_step_generation()
    prev_trapq = stepper.set_trapq(self.trapq)            # own sidecar trapq
    print_time = self.toolhead.get_last_move_time()       # pinned to main toolhead's tail
    trapq_append(self.trapq, print_time, …)
    stepper.generate_steps(print_time)
    stepper.set_trapq(prev_trapq)
    self.toolhead.note_kinematic_activity(print_time)
    self.toolhead.dwell(accel_t + cruise_t + accel_t)     # ← LEAK
```

Steps 1–5 are fine. Step 6's `toolhead.dwell` is what advances main `print_time` by the chunk duration, which is how subsequent G1 moves get scheduled behind our chunks and how the gcode lookahead inflates.

`toolhead.dwell(delay)`:
```python
def dwell(self, delay):
    next_print_time = self.get_last_move_time() + max(0., delay)
    self._update_move_time(next_print_time)
    self._check_pause()
```

And `_check_pause` throttles the host once `(print_time - est_print_time) > buffer_time_high` (default 2 s). That's the safety net keeping our chunk loop from queueing infinitely; remove the dwell and you remove that throttle too, so the replacement must self-throttle.

## 4. What we want the replacement to do

- The buffer stepper continues to execute moves with the correct step timing on the MCU.
- Each chunk's motion is **scheduled in wall-clock time**, not on the main toolhead's `last_move_time`.
- The main toolhead's `print_time` is **untouched** by chunk scheduling. Main G1 motion proceeds at its baseline cadence regardless of how many chunks are in flight.
- The chunk loop self-paces so we don't queue unbounded motion ahead of physical execution. Today the loop is paced by a reactor timer at `recovery_move_distance / speed * 0.5`, which is fine *if* we add an explicit "is the previous chunk physically done?" check. (Right now the implicit throttle was `_check_pause` on the main toolhead.)
- Tool-change behavior is preserved: when bound, the buffer stepper is `sync_to_extruder`'d to the active extruder's main trapq; when unsynced for chunked operations, it runs on its sidecar.
- All five chunked sites in `buffer.py` route through one helper (don't fix only recovery).

## 5. Reference material — read these before planning

The right Klipper reference for "run a stepper move independently of the toolhead's main timeline" is `klippy/extras/manual_stepper.py` (upstream Klipper, identical pattern in Kalico). It maintains its own trapq, generates steps, and synchronizes with the rest of the system via `toolhead.note_mcu_movequeue_activity` (or the version-appropriate equivalent) rather than `toolhead.dwell`. Read its `do_move`, `do_homing_move`, and `sync_print_time` to see how chunks are sequenced without leaking into the main timeline.

Read the Klipper class hierarchy:
- `klippy/kinematics/extruder.py` → `ExtruderStepper` (our `self.extruder_stepper`). Has `sync_to_extruder(name)` we already use.
- `klippy/extras/force_move.py` → for the trapq-append pattern and the `cartesian_position_kinematics` helper.
- `klippy/toolhead.py` → for `flush_step_generation`, `get_last_move_time`, `note_mcu_movequeue_activity`, `register_lookahead_callback`, `_check_pause`.

You **do not** have these source files in this repo. Reading them via WebFetch from `github.com/KalicoCrew/kalico` is the fastest path. The user is on Kalico `v0.12.0-811-g107302c4` — fetch from that ref if you can.

## 6. Files in this repo you must read

- `klipper/buffer.py` — the only production file. Key call sites for chunked motion:
  - `_do_recovery_fill_chunk` (around line 836)
  - `_do_fill_chunk` (initial fill, around line 1115)
  - `_do_safety_retract` (around line 1248)
  - `_do_continuous_chunk` (manual feed/retract continuous, around line 1379)
  - `_do_retract_until_clear_chunk` (around line 1610)
  All of these call `self.force_move.manual_move(self.extruder_stepper.stepper, ...)`. All need to route through the new sidecar helper.
- `klipper/buffer.py` lines 296–333 — `_handle_ready`. Resolves `self.force_move` and `self.toolhead`. The new sidecar will need a trapq allocated here (or lazily on first use).
- `klipper/buffer.py` lines 480–522 — `_sync` / `_unsync`. The new sidecar must coexist with these (chunks fire only while unsynced; sync rebinds to extruder's trapq).
- `tests/conftest.py` — `MockForceMove` captures `(stepper, dist, speed, accel)`. The new helper will need its own mock, and many tests assert against `force_move.moves` — those assertions will need to point at the new helper's recorded moves. Check `tests/test_*.py` for everywhere `force_move.moves` is referenced.
- `sample_config/lll-plus.cfg` and `README.md` — only update if you add new config knobs.

## 7. What's already been built (don't redo)

Commit `04c44de` (just landed on this branch) added:
- `recovery_move_distance` config (default 5 mm)
- Control-timer watchdog enforcing `extreme_recovery_timeout` independently of the chunk chain (EMPTY-only)
- `_safety_zone_start` no longer re-arms during active recovery
- Chunk-chain stall kick from the control timer
- Self-paced chunk timer (`_recovery_fill_timer` + `_recovery_fill_timer_cb`) replacing `register_callback`
- `_dlog_throttled` helper applied to zone-transition + rd_mult logs
- Sensor-conflict deferral (`_conflict_since`) — transient conflicts no longer fault, persistent ones escalate from control timer
- `recovery_chunk_count` exposed in `get_status`

The self-paced chunk timer is **already there and correct** — the new sidecar mechanism plugs into the same loop. You don't need to rewrite the pacing; you only need to replace the per-chunk call from `force_move.manual_move` with the new helper.

## 8. Acceptance criteria

The plan must satisfy:

1. After the change, **no `toolhead.dwell` is called as a side effect of a buffer-stepper chunk**. (Verifiable via a test that fires a chunk and asserts `MockToolhead.dwell` was not called.)
2. During an EMPTY recovery cycle, main-toolhead `last_move_time` advances only from main-toolhead motion sources — the buffer's chunk loop must not contribute. (Test: enter EMPTY recovery, fire N chunks, assert `last_move_time` delta = 0.)
3. All 188 existing tests pass; new tests are added for #1, #2, and at least one for each of the five chunked sites.
4. The chunk loop self-throttles in the absence of `_check_pause`. Design must explain how (e.g., wait for the previous chunk's print_time to be ≤ `mcu.estimated_print_time(now)` before queueing the next; or use `toolhead.register_lookahead_callback`).
5. Tool-change handling continues to work: `_handle_extruder_change` must not be broken by the new sidecar. Sync/unsync semantics unchanged from the user's perspective.
6. No new dependencies beyond stock Kalico/Klipper APIs. We cannot fork Klipper.
7. `BUFFER_STATUS` output is unchanged from the user's perspective (no new noise added unless you also choose to expose chunk-pacing diagnostics).
8. Safety retract, initial fill, continuous manual feed, retract-until-clear: all five sites use the new helper. Don't fix only recovery.

## 9. Risk register — flag these in your plan

- **Trapq lifetime / GC**: a custom trapq must be kept alive for the buffer's lifetime. Look at how `manual_stepper.py` allocates it.
- **Stepper rebinding under tool change**: today `_unsync` does `extruder_stepper.set_rotation_distance(base_rd)` after `sync_to_extruder(None)`. The new sidecar mustn't fight this. Carefully sequence: unsync → bind to sidecar trapq → queue moves → unbind sidecar → restore for next sync.
- **MCU-side step queue depth**: without `toolhead.dwell`'s throttle, you could queue arbitrarily many chunks into the MCU's step queue. The MCU has finite room; overflow is a hard error. The replacement throttle must cap pending chunk count or pending chunk wall-clock window. A test that fires 1000 chunks rapidly without a host-side throttle should still not crash the mock MCU.
- **Estimated print time skew at startup**: `mcu.estimated_print_time(eventtime)` requires the MCU to be ready. Make sure first-chunk-after-ready handles the bootstrapping case.
- **`flush_step_generation` semantics**: still call it once before each chunk batch, but don't rely on toolhead.dwell to advance the time horizon.
- **Existing tests using `force_move.moves`**: at least a dozen tests assert against this. You'll likely route through a new attribute (e.g. `buf.sidecar_moves` or extend MockForceMove) — update tests accordingly. Don't break the mock.

## 10. What to deliver

Use the standard plan-mode workflow:

1. **Explore phase**: launch 1–3 Explore agents in parallel to read upstream Klipper's `manual_stepper.py`, `force_move.py`, `toolhead.py`, and any existing patterns in this repo that look like they could be repurposed. Be ruthless about staying inside Klipper-native APIs.
2. **Design phase**: 1 Plan agent. Output should specify: where the sidecar trapq lives, what the new helper's signature is, how throttling works without `_check_pause`, how the five existing chunk sites are refactored, what changes in `MockForceMove`/`MockToolhead`.
3. **Review phase**: read the critical files yourself (don't trust agent summaries blindly), confirm with the user via `AskUserQuestion` only if there are open requirement questions (try not to — the requirements here should be sufficient).
4. **Write plan to** the plan file specified in your plan-mode system message. Include verification steps (the 8 acceptance criteria above, and `python -m pytest tests/` from the repo root).
5. **`ExitPlanMode`** to request approval. Do not implement.

## 11. Out of scope

- Multi-extruder tool-change behavior changes
- Anything to do with the gcode parser's lookahead depth (that's Klipper's; users handle pause-overshoot via M400 in their PAUSE macro)
- New config knobs unless strictly necessary to expose the sidecar mechanism
- Refactoring `_unsync` / `_sync` beyond what's needed to coexist with the sidecar
- Touching the FULL-recovery path (it's passive, no chunks, not affected)
