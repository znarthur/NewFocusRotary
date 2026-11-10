#!/usr/bin/env bash
# One-time setup: RLS EncoSight (32-bit Windows) under Wine on a Raspberry Pi (64-bit Debian /
# Raspberry Pi OS), talking to the E201-9B over USB.
#
#   ./setup-pi.sh /path/to/EncoSight_for_E201-9B_ver5.0.0.17.exe
#
# On ARM, x86 Windows code needs an emulator.  This uses Pi-Apps' "Wine (x64)" package (Box64 +
# x86-64 Wine, which also runs 32-bit programs).  If a working `wine` is already on PATH it is used
# as is.  On an x86-64 Debian/Ubuntu PC it installs the distribution's wine instead.
#
# Environment: ENCOSIGHT_PREFIX (default ~/.wine-encosight), FORCE=1 to skip the page-size check.
set -euo pipefail

EXE_SRC=${1:-}
PREFIX=${ENCOSIGHT_PREFIX:-$HOME/.wine-encosight}
HERE=$(dirname "$(readlink -f "$0")")
BIN=$HOME/.local/bin

say() { printf '\n== %s\n' "$*"; }
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

[ -n "$EXE_SRC" ] && [ -f "$EXE_SRC" ] || die "usage: $0 /path/to/EncoSight_for_E201-9B_*.exe"
[ "$(id -u)" -ne 0 ] || die "run as your normal user (sudo is used where needed), not as root"
ARCH=$(uname -m)

say "System: $(. /etc/os-release && echo "$PRETTY_NAME"), $ARCH, page size $(getconf PAGESIZE)"
if [ "$ARCH" = aarch64 ] && [ "$(getconf PAGESIZE)" != 4096 ] && [ -z "${FORCE:-}" ]; then
    cat >&2 <<'MSG'
The kernel uses 16K memory pages (Raspberry Pi 5 default). Wine + Box64 need 4K pages.
Fix: add this line to /boot/firmware/config.txt, reboot, and run this script again:

    kernel=kernel8.img

(Check afterwards with: getconf PAGESIZE   -> must print 4096)
MSG
    exit 1
fi
[ "$ARCH" != armv7l ] || die "32-bit Raspberry Pi OS is not supported; install the 64-bit OS"

say "Serial port access"
if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
    sudo usermod -aG dialout "$USER"
    echo "Added $USER to 'dialout'. Log out and back in (or reboot) before launching EncoSight."
else
    echo "$USER is in 'dialout': OK"
fi
sudo apt-get install -y psmisc udev python3 >/dev/null   # fuser (port-busy check), udevadm, sniffer

say "Wine"
if command -v wine >/dev/null && wine --version >/dev/null 2>&1; then
    echo "Using existing $(wine --version)"
elif [ "$ARCH" = x86_64 ]; then
    sudo dpkg --add-architecture i386
    sudo apt-get update
    sudo apt-get install -y wine wine32:i386 wine64
elif [ "$ARCH" = aarch64 ]; then
    if [ ! -x "$HOME/pi-apps/manage" ]; then
        sudo apt-get install -y git
        git clone --depth 1 https://github.com/Botspot/pi-apps "$HOME/pi-apps"
    fi
    echo "Installing Box64 + Wine via Pi-Apps (this takes a while)..."
    "$HOME/pi-apps/manage" install "Wine (x64)"
    hash -r
    command -v wine >/dev/null || die "Pi-Apps finished but 'wine' is not on PATH; open a new terminal and re-run"
    wine --version >/dev/null 2>&1 || die "'wine' does not run; see Pi-Apps' Wine (x64) notes"
    echo "Installed $(wine --version)"
else
    die "unsupported architecture $ARCH"
fi

say "Wine prefix $PREFIX"
export WINEPREFIX=$PREFIX WINEDEBUG=-all
if [ ! -d "$PREFIX/drive_c" ]; then
    wineboot -i >/dev/null 2>&1 || die "wineboot failed"
fi
wineserver -w || true
mkdir -p "$PREFIX/drive_c/EncoSight"
cp "$EXE_SRC" "$PREFIX/drive_c/EncoSight/EncoSight.exe"
echo "EncoSight copied to C:\\EncoSight\\EncoSight.exe"

say "Launcher"
mkdir -p "$BIN" "$HOME/.local/share/applications" "$HOME/.local/share/encosight-wine"
install -m 755 "$HERE/encosight.sh" "$HOME/.local/share/encosight-wine/encosight.sh"
install -m 755 "$HERE/e201_sniff.py" "$HOME/.local/share/encosight-wine/e201_sniff.py"
ln -sf "$HOME/.local/share/encosight-wine/encosight.sh" "$BIN/encosight"
cat > "$HOME/.local/share/applications/encosight.desktop" <<EOF
[Desktop Entry]
Name=EncoSight (E201-9B)
Comment=RLS encoder diagnostics under Wine
Exec=$BIN/encosight
Terminal=false
Type=Application
Categories=Utility;
EOF
echo "Run:  encosight            (or from the desktop menu)"
echo "      encosight --sniff    (also logs all E201 traffic to ~/e201-traffic-*.log)"
case ":$PATH:" in *":$BIN:"*) ;; *) echo "Note: add $BIN to PATH (e.g. in ~/.profile)";; esac

say "Done. Stop the EPICS IOC first: EncoSight and the IOC cannot share the E201 port."
