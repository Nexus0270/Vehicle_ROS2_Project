#!/usr/bin/env bash
# Arduino IDE launcher.
#
# Electron refuses to start unless its chrome-sandbox helper is owned by root
# with mode 4755. The zip ships it owned by the extracting user, and setting
# that needs sudo, so this wrapper falls back to --no-sandbox until you run:
#
#   sudo chown root:root ~/Applications/arduino-ide_2.3.10_Linux_64bit/chrome-sandbox
#   sudo chmod 4755      ~/Applications/arduino-ide_2.3.10_Linux_64bit/chrome-sandbox
#
# After that the check below passes on its own and the sandbox is used.

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SANDBOX="$DIR/chrome-sandbox"

if [ -u "$SANDBOX" ] && [ "$(stat -c %u "$SANDBOX")" = "0" ]; then
  exec "$DIR/arduino-ide" "$@"
else
  exec "$DIR/arduino-ide" --no-sandbox "$@"
fi
