#!/usr/bin/env bash
# ============================================================================
#  CoveDoor — Build & Flash Script
# ============================================================================
#  Builds the ESP32 firmware and flashes it to a connected device.
#
#  Usage:
#    ./flash.sh              Build and flash (auto-detect serial port)
#    ./flash.sh /dev/ttyUSB0 Build and flash to specific port
#    ./flash.sh --build-only Build without flashing
#    ./flash.sh --monitor    Build, flash, and open serial monitor
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UPLOAD_SPEED=921600
MONITOR_SPEED=115200
ENV_NAME="esp32"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

log()   { echo -e "${CYAN}[CoveDoor]${NC} $*"; }
ok()    { echo -e "${GREEN}[OK]${NC} $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()   { echo -e "${RED}[ERROR]${NC} $*" >&2; }

# ── Parse arguments ─────────────────────────────────────────────────────────

BUILD_ONLY=false
MONITOR=false
PORT=""

for arg in "$@"; do
    case "$arg" in
        --build-only) BUILD_ONLY=true ;;
        --monitor)    MONITOR=true ;;
        /dev/*)       PORT="$arg" ;;
        *)            err "Unknown argument: $arg"; exit 1 ;;
    esac
done

# ── Check PlatformIO ────────────────────────────────────────────────────────

if ! command -v pio &>/dev/null; then
    err "PlatformIO CLI (pio) not found."
    log "Install with: pip install platformio"
    exit 1
fi

# ── Detect serial port if not specified ─────────────────────────────────────

detect_port() {
    local ports=()
    for p in /dev/ttyUSB* /dev/ttyACM* /dev/cu.usbserial* /dev/cu.SLAB_USBtoUART*; do
        [ -e "$p" ] && ports+=("$p")
    done
    if [ ${#ports[@]} -eq 0 ]; then
        return 1
    elif [ ${#ports[@]} -eq 1 ]; then
        echo "${ports[0]}"
    else
        warn "Multiple serial ports found:"
        for i in "${!ports[@]}"; do
            echo "  [$i] ${ports[$i]}"
        done
        echo -n "Select port [0]: "
        read -r choice
        choice=${choice:-0}
        echo "${ports[$choice]}"
    fi
}

# ── Build ───────────────────────────────────────────────────────────────────

log "Building CoveDoor firmware..."
cd "$SCRIPT_DIR"

if pio run -e "$ENV_NAME" 2>&1; then
    ok "Build successful"
else
    err "Build failed"
    exit 1
fi

# Show firmware size
FIRMWARE_BIN="$SCRIPT_DIR/.pio/build/$ENV_NAME/firmware.bin"
if [ -f "$FIRMWARE_BIN" ]; then
    SIZE=$(stat -f%z "$FIRMWARE_BIN" 2>/dev/null || stat -c%s "$FIRMWARE_BIN" 2>/dev/null || echo "unknown")
    log "Firmware binary: $FIRMWARE_BIN ($SIZE bytes)"
fi

if $BUILD_ONLY; then
    ok "Build-only mode — skipping flash"
    exit 0
fi

# ── Flash ───────────────────────────────────────────────────────────────────

if [ -z "$PORT" ]; then
    log "Detecting serial port..."
    PORT=$(detect_port) || {
        err "No ESP32 serial port detected. Is the device connected?"
        err "  - Check USB cable connection"
        err "  - Install CP2102/CH340 drivers if needed"
        err "  - Specify port manually: ./flash.sh /dev/ttyUSB0"
        exit 1
    }
fi

log "Flashing to $PORT at ${UPLOAD_SPEED} baud..."

if pio run -e "$ENV_NAME" -t upload --upload-port "$PORT" 2>&1; then
    ok "Flash successful!"
else
    err "Flash failed. Troubleshooting:"
    err "  - Hold BOOT button on ESP32 during flash"
    err "  - Try a different USB cable (data cable, not charge-only)"
    err "  - Check port permissions: sudo chmod 666 $PORT"
    err "  - Verify driver: ls -la $PORT"
    exit 1
fi

# ── Monitor (optional) ─────────────────────────────────────────────────────

if $MONITOR; then
    log "Opening serial monitor at ${MONITOR_SPEED} baud (Ctrl+C to exit)..."
    pio device monitor -b "$MONITOR_SPEED" -p "$PORT"
fi

ok "Done."
