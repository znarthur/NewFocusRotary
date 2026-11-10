#!/usr/bin/env bash
# Launch RLS EncoSight (Windows, 32-bit x86) under Wine with the E201-9B mapped to a COM port.
#
# EncoSight only accepts COM ports registered under HKLM\HARDWARE\DEVICEMAP\SERIALCOMM with a
# Windows USB-serial name (\Device\USBSER000).  Wine registers mapped ports as \Device\SerialN and
# rebuilds that (volatile) key every session, so this script fixes it up on each launch.
#
# Environment:
#   ENCOSIGHT_PREFIX  Wine prefix            (default ~/.wine-encosight)
#   ENCOSIGHT_EXE     path to EncoSight .exe (default $ENCOSIGHT_PREFIX/drive_c/EncoSight/EncoSight.exe)
#   E201_PORT         Linux serial device    (default: auto-detect USB VID 0483 = STMicro / E201)
#   E201_COM          COM port number        (default 5)
#   WINE              wine command           (default wine)
#   E201_SKIP_BUSY_CHECK=1  skip the "port in use" check
#
# Options:
#   --sniff [LOGFILE]  log all E201 traffic (default ~/e201-traffic-<date>.log) through
#                      e201_sniff.py, e.g. to see what EncoSight sends for a multiturn reset
set -euo pipefail

PREFIX=${ENCOSIGHT_PREFIX:-$HOME/.wine-encosight}
EXE=${ENCOSIGHT_EXE:-$PREFIX/drive_c/EncoSight/EncoSight.exe}
COMN=${E201_COM:-5}
WINE=${WINE:-wine}
WINESERVER=${WINESERVER:-wineserver}
export WINEPREFIX=$PREFIX WINEDEBUG=${WINEDEBUG:--all}

SNIFF_LOG=""
if [ "${1:-}" = "--sniff" ]; then
    SNIFF_LOG=${2:-$HOME/e201-traffic-$(date +%Y%m%d-%H%M%S).log}
fi
SNIFFER=${E201_SNIFFER:-$(dirname "$(readlink -f "$0")")/e201_sniff.py}

find_e201() {
    local dev
    for dev in /dev/ttyACM* /dev/ttyUSB*; do
        [ -e "$dev" ] || continue
        if udevadm info -q property -n "$dev" 2>/dev/null | grep -q '^ID_VENDOR_ID=0483$'; then
            # Prefer the stable by-id name if there is one.
            local byid
            for byid in /dev/serial/by-id/*; do
                [ "$(readlink -f "$byid")" = "$(readlink -f "$dev")" ] && { echo "$byid"; return 0; }
            done
            echo "$dev"; return 0
        fi
    done
    return 1
}

PORT=${E201_PORT:-$(find_e201 || true)}
if [ -z "$PORT" ] || [ ! -e "$PORT" ]; then
    echo "E201 not found (no USB serial device with vendor 0483). Plug it in or set E201_PORT." >&2
    exit 1
fi
[ -r "$PORT" ] && [ -w "$PORT" ] || { echo "No read/write access to $PORT (add yourself to the dialout group)." >&2; exit 1; }
if [ -z "${E201_SKIP_BUSY_CHECK:-}" ] && command -v fuser >/dev/null && fuser -s "$PORT" 2>/dev/null; then
    echo "$PORT is in use by another program (the EPICS IOC?). Stop it first:" >&2
    fuser -v "$PORT" >&2 || true
    exit 1
fi
[ -f "$EXE" ] || { echo "EncoSight not found at $EXE (run setup-pi.sh, or set ENCOSIGHT_EXE)." >&2; exit 1; }

SNIFF_PID=""
if [ -n "$SNIFF_LOG" ]; then
    [ -f "$SNIFFER" ] || { echo "sniffer not found: $SNIFFER" >&2; exit 1; }
    PTY_LINK=$(mktemp -u "${TMPDIR:-/tmp}/e201-sniff-XXXXXX")
    python3 "$SNIFFER" "$PORT" "$PTY_LINK" "$SNIFF_LOG" &
    SNIFF_PID=$!
    for _ in $(seq 50); do [ -e "$PTY_LINK" ] && break; sleep 0.1; done
    [ -e "$PTY_LINK" ] || { echo "sniffer failed to start" >&2; kill "$SNIFF_PID" 2>/dev/null; exit 1; }
    echo "Logging E201 traffic to $SNIFF_LOG"
    PORT=$PTY_LINK
fi
echo "E201: $PORT -> COM$COMN"
"$WINESERVER" -k 2>/dev/null || true
# Persistent port mapping (Wine's documented override), then start a session that stays up
# while we fix the registry, so wineboot cannot rebuild the key under us.
"$WINE" reg add 'HKLM\Software\Wine\Ports' /v "COM$COMN" /t REG_SZ /d "$PORT" /f >/dev/null
"$WINESERVER" -k 2>/dev/null || true
"$WINESERVER" -p
"$WINE" wineboot >/dev/null 2>&1 || true
# Re-register COMn under a USB-serial name, the only kind EncoSight recognises.
key='HKLM\HARDWARE\DEVICEMAP\SERIALCOMM'
"$WINE" reg query "$key" 2>/dev/null | tr -d '\r' | awk -v com="COM$COMN" '$3 == com {print $1}' | while read -r name; do
    "$WINE" reg delete "$key" /v "$name" /f >/dev/null 2>&1 || true
done
"$WINE" reg add "$key" /v '\Device\USBSER000' /t REG_SZ /d "COM$COMN" /f >/dev/null
trap '"$WINESERVER" -k 2>/dev/null || true; [ -n "$SNIFF_PID" ] && kill "$SNIFF_PID" 2>/dev/null || true' EXIT
cd "$(dirname "$EXE")"
"$WINE" "$(basename "$EXE")"
