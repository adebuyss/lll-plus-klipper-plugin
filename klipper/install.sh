#!/bin/bash
# Install the buffer Klipper extras module
# Symlinks buffer.py into the Klipper extras directory

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
KLIPPER_EXTRAS="${HOME}/klipper/klippy/extras"

if [ ! -d "$KLIPPER_EXTRAS" ]; then
    echo "Error: Klipper extras directory not found at $KLIPPER_EXTRAS"
    echo "Make sure Klipper is installed at ~/klipper/"
    exit 1
fi

echo "Installing buffer.py into $KLIPPER_EXTRAS ..."
ln -sf "${SCRIPT_DIR}/buffer.py" "${KLIPPER_EXTRAS}/buffer.py"

if [ -L "${KLIPPER_EXTRAS}/buffer.py" ]; then
    echo "Success! buffer.py symlinked."
    echo "Restart Klipper to load the module: sudo systemctl restart klipper"
else
    echo "Error: Failed to create symlink"
    exit 1
fi
