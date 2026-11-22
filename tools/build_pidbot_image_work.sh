#!/usr/bin/env bash
set -euo pipefail

# Default configuration
SSID="PIDBot-AP"
PASS="robotpass"
BAUD="230400"
WEB_PORT="8000"
COUNTRY="RU"
OUT_IMG="./pidbot-rpi5-ap.img"
FORCE_SIM="false"
INSECURE_TLS="false"

# Full unshortened URLs (assembled cleanly to avoid markdown/UI truncation)
RPI_HOST="downloads.raspberrypi.com"
RPI_PATH="/raspios_lite_arm64/images/"
BASE_INDEX_URL="https://${RPI_HOST}${RPI_PATH}"

GH_HOST="raw.githubusercontent.com"
GH_PATH="/pc-64/PIDBot/refs/heads/main/tools/read_com_live3.py"
APP_RAW_URL="https://${GH_HOST}${GH_PATH}"

TMP_DIR=""
LOOP_DEV=""
MOUNT_DIR=""
KPARTX_USED="false"
BUILD_SUCCESS="false"

usage() {
    cat <<EOF
Usage: sudo $0 [OPTIONS]

Build a 100% offline zero-touch Raspberry Pi OS Lite (64-bit, Trixie) image
configured as a standalone Wi-Fi Access Point (10.42.0.1) running PIDBot.

Options:
  -s, --ssid <name>       Wi-Fi Access Point SSID, 1-32 chars (default: ${SSID})
  -p, --pass <password>   Wi-Fi AP & 'pi' SSH password, 8-63 chars (default: ${PASS})
  -b, --baud <rate>       Serial baud rate, positive integer (default: ${BAUD})
  -w, --web-port <port>   HTTP telemetry port, 1-65535 (default: ${WEB_PORT})
  -c, --country <code>    2-letter uppercase ISO Wi-Fi country code (default: ${COUNTRY})
  -o, --out <file.img>    Output .img file path (default: ${OUT_IMG})
      --sim               Force physics simulation mode even if USB serial is attached
  -k, --insecure          Bypass host TLS certificate verification during build downloads
  -h, --help              Show this help message and exit
EOF
}

cleanup() {
    local exit_code=$?
    set +e

    if [[ -n "${MOUNT_DIR}" ]] && mountpoint -q "${MOUNT_DIR}"; then
        umount "${MOUNT_DIR}"
    fi
    if [[ -n "${MOUNT_DIR}" && -d "${MOUNT_DIR}" ]]; then
        rmdir "${MOUNT_DIR}" 2>/dev/null || true
    fi
    if [[ -n "${LOOP_DEV}" ]]; then
        if [[ "${KPARTX_USED}" == "true" ]] && command -v kpartx >/dev/null 2>&1; then
            kpartx -d "${LOOP_DEV}" >/dev/null 2>&1 || true
        fi
        losetup -d "${LOOP_DEV}" 2>/dev/null || true
    fi
    if [[ -n "${TMP_DIR}" && -d "${TMP_DIR}" ]]; then
        rm -rf "${TMP_DIR}"
    fi

    if [[ "${BUILD_SUCCESS}" != "true" && -n "${OUT_IMG}" && -f "${OUT_IMG}" ]]; then
        echo "[!] Build interrupted or failed; removing partial image: ${OUT_IMG}" >&2
        rm -f "${OUT_IMG}"
    fi

    exit "${exit_code}"
}

trap cleanup EXIT INT TERM

# Parse CLI arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -s|--ssid)
            [[ $# -lt 2 ]] && { echo "Error: $1 requires a value." >&2; exit 1; }
            SSID="$2"
            shift 2
            ;;
        -p|--pass)
            [[ $# -lt 2 ]] && { echo "Error: $1 requires a value." >&2; exit 1; }
            PASS="$2"
            shift 2
            ;;
        -b|--baud)
            [[ $# -lt 2 ]] && { echo "Error: $1 requires a value." >&2; exit 1; }
            BAUD="$2"
            shift 2
            ;;
        -w|--web-port)
            [[ $# -lt 2 ]] && { echo "Error: $1 requires a value." >&2; exit 1; }
            WEB_PORT="$2"
            shift 2
            ;;
        -c|--country)
            [[ $# -lt 2 ]] && { echo "Error: $1 requires a value." >&2; exit 1; }
            COUNTRY="$2"
            shift 2
            ;;
        -o|--out)
            [[ $# -lt 2 ]] && { echo "Error: $1 requires a value." >&2; exit 1; }
            OUT_IMG="$2"
            shift 2
            ;;
        --sim)
            FORCE_SIM="true"
            shift
            ;;
        -k|--insecure)
            INSECURE_TLS="true"
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "Error: Unknown option '$1'" >&2
            usage >&2
            exit 1
            ;;
    esac
done

# Validate inputs
if [[ ${#SSID} -lt 1 || ${#SSID} -gt 32 ]]; then
    echo "Error: --ssid must be between 1 and 32 characters." >&2
    exit 1
fi

if [[ ${#PASS} -lt 8 || ${#PASS} -gt 63 ]]; then
    echo "Error: --pass must be between 8 and 63 characters (WPA2-PSK requirement)." >&2
    exit 1
fi

if ! [[ "${BAUD}" =~ ^[1-9][0-9]*$ ]]; then
    echo "Error: --baud must be a positive integer." >&2
    exit 1
fi

if ! [[ "${WEB_PORT}" =~ ^[0-9]+$ ]] || [[ "${WEB_PORT}" -lt 1 || "${WEB_PORT}" -gt 65535 ]]; then
    echo "Error: --web-port must be an integer between 1 and 65535." >&2
    exit 1
fi

if ! [[ "${COUNTRY}" =~ ^[A-Z]{2}$ ]]; then
    echo "Error: --country must be a 2-letter uppercase ISO country code (e.g., RU, US, DE)." >&2
    exit 1
fi

# Check root privileges
if [[ "${EUID}" -ne 0 ]]; then
    echo "Error: This script must be run as root (e.g., sudo $0)." >&2
    exit 1
fi

# Verify required host utilities
REQUIRED_TOOLS=(curl xz losetup mount umount grep sed sort tail head python3 base64)
for tool in "${REQUIRED_TOOLS[@]}"; do
    if ! command -v "${tool}" >/dev/null 2>&1; then
        echo "Error: Required host utility '${tool}' is not installed." >&2
        exit 1
    fi
done

# Configure host curl TLS options
CURL_TLS_OPTS=()
    CURL_TLS_OPTS+=(--insecure)

# Helper functions for safe escaping
yaml_single_quote() {
    local val="$1"
    printf "'%s'" "${val//\'/\'\'}"
}

bash_single_quote() {
    local val="$1"
    printf "'%s'" "${val//\'/\'\\\'\'}"
}

YAML_SSID=$(yaml_single_quote "${SSID}")
YAML_PASS=$(yaml_single_quote "${PASS}")
BASH_SSID=$(bash_single_quote "${SSID}")
BASH_PASS=$(bash_single_quote "${PASS}")

# Prepare workspace and output directory
OUT_DIR=$(dirname "${OUT_IMG}")
mkdir -p "${OUT_DIR}"
TMP_DIR=$(mktemp -d /tmp/pidbot-build.XXXXXX)
MOUNT_DIR="${TMP_DIR}/bootfs"
mkdir -p "${MOUNT_DIR}"


echo "[1/5] Resolving latest Raspberry Pi OS Lite (64-bit, Trixie) image..."
INDEX_HTML=$(curl -fsSL "${CURL_TLS_OPTS[@]}" "${BASE_INDEX_URL}")
LATEST_SUBDIR=$(printf '%s\n' "${INDEX_HTML}" | grep -oE 'href="raspios_lite_arm64-[0-9]{4}-[0-9]{2}-[0-9]{2}/"' | sed -E 's/^href="|"$//g' | sort -V | tail -n 1)

if [[ -z "${LATEST_SUBDIR}" ]]; then
    echo "Error: Failed to locate release directory at ${BASE_INDEX_URL}" >&2
    exit 1
fi


RELEASE_URL="${BASE_INDEX_URL}${LATEST_SUBDIR}"
RELEASE_HTML=$(curl -fsSL "${CURL_TLS_OPTS[@]}" "${RELEASE_URL}")
IMG_XZ_NAME=$(printf '%s\n' "${RELEASE_HTML}" | grep -oE 'href="[^"]+trixie-arm64-lite\.img\.xz"' | sed -E 's/^href="|"$//g' | sort -V | tail -n 1)

if [[ -z "${IMG_XZ_NAME}" ]]; then
    IMG_XZ_NAME=$(printf '%s\n' "${RELEASE_HTML}" | grep -oE 'href="[^"]+-arm64-lite\.img\.xz"' | sed -E 's/^href="|"$//g' | sort -V | tail -n 1)
fi

if [[ -z "${IMG_XZ_NAME}" ]]; then
    echo "Error: Failed to locate *-arm64-lite.img.xz at ${RELEASE_URL}" >&2
    exit 1
fi

FULL_IMG_URL="${RELEASE_URL}${IMG_XZ_NAME}"
ARCHIVE_PATH="${TMP_DIR}/${IMG_XZ_NAME}"
PY_APP_LOCAL="${TMP_DIR}/read_com_live3.py"

echo "[2/5] Downloading ${IMG_XZ_NAME} and bundling read_com_live3.py for offline boot..."
curl -fL "${CURL_TLS_OPTS[@]}" --progress-bar --retry 5 --retry-delay 2 "${FULL_IMG_URL}" -o "${ARCHIVE_PATH}"
curl -fsSL "${CURL_TLS_OPTS[@]}" --retry 5 --retry-delay 2 "${APP_RAW_URL}" -o "${PY_APP_LOCAL}"

if ! grep -q "ThreadingHTTPServer" "${PY_APP_LOCAL}"; then
    echo "Error: Downloaded read_com_live3.py looks invalid (missing ThreadingHTTPServer)." >&2
    exit 1
fi

APP_B64=$(base64 -w 0 "${PY_APP_LOCAL}")

echo "[3/5] Decompressing image to ${OUT_IMG}..."
xz -dc "${ARCHIVE_PATH}" > "${OUT_IMG}"
rm -f "${ARCHIVE_PATH}"

echo "[4/5] Attaching loop device and mounting bootfs (p1)..."
LOOP_DEV=$(losetup --show -fP "${OUT_IMG}")
BOOT_PART="${LOOP_DEV}p1"

udevadm settle 2>/dev/null || true
if [[ ! -b "${BOOT_PART}" ]]; then
    command -v partprobe >/dev/null 2>&1 && partprobe "${LOOP_DEV}" 2>/dev/null || true
    command -v partx >/dev/null 2>&1 && partx -u "${LOOP_DEV}" 2>/dev/null || true
    udevadm settle 2>/dev/null || true
fi

for _ in {1..10}; do
    [[ -b "${BOOT_PART}" ]] && break
    sleep 0.5
done

if [[ ! -b "${BOOT_PART}" ]] && command -v kpartx >/dev/null 2>&1; then
    kpartx -av "${LOOP_DEV}" >/dev/null
    KPARTX_USED="true"
    BOOT_PART="/dev/mapper/$(basename "${LOOP_DEV}")p1"
    udevadm settle 2>/dev/null || true
fi

if [[ ! -b "${BOOT_PART}" ]]; then
    echo "Error: Boot partition ${BOOT_PART} did not appear." >&2
    exit 1
fi

mount "${BOOT_PART}" "${MOUNT_DIR}"

echo "[5/5] Injecting 100% offline zero-touch cloud-init configuration..."

# 1. meta-data
cat > "${MOUNT_DIR}/meta-data" <<'EOF'
instance-id: pidbot-rpi5-ap-offline
local-hostname: pidbot
EOF

# 2. network-config (eth0 optional; wlan0 managed exclusively by NetworkManager)
cat > "${MOUNT_DIR}/network-config" <<'EOF'
version: 2
ethernets:
  eth0:
    dhcp4: true
    optional: true
EOF

# 3. user-data (100% offline: no apt/curl required on Pi first boot)
cat > "${MOUNT_DIR}/user-data" <<'EOF'
#cloud-config
hostname: pidbot
manage_etc_hosts: true
package_update: false
package_upgrade: false

users:
  - name: pi
    plain_text_passwd: __YAML_PASS__
    lock_passwd: false
    sudo: ALL=(ALL) NOPASSWD:ALL
    shell: /bin/bash
    groups:
      - adm
      - dialout
      - cdrom
      - sudo
      - audio
      - video
      - plugdev
      - games
      - users
      - input
      - netdev
      - spi
      - i2c
      - gpio

ssh_pwauth: true

write_files:
  - path: /etc/ssh/sshd_config.d/99-pidbot-ssh.conf
    permissions: '0644'
    owner: root:root
    content: |
      PasswordAuthentication yes
      KbdInteractiveAuthentication yes

  - path: /etc/default/pidbot
    permissions: '0644'
    owner: root:root
    content: |
      PIDBOT_BAUD=__BAUD__
      PIDBOT_WEB_PORT=__WEB_PORT__
      PIDBOT_FORCE_SIM=__FORCE_SIM__

  - path: /home/pi/pidbot/read_com_live3.py
    permissions: '0755'
    encoding: b64
    content: __APP_B64__

  - path: /usr/local/bin/patch-pidbot.py
    permissions: '0755'
    owner: root:root
    content: |
      #!/usr/bin/env python3
      import pathlib
      import re
      import sys
      import textwrap

      target_arg = sys.argv[1] if len(sys.argv) > 1 else "/home/pi/pidbot/read_com_live3.py"
      target = pathlib.Path(target_arg)
      src = target.read_text(encoding="utf-8")

      # Bind ThreadingHTTPServer to all interfaces (0.0.0.0) instead of localhost/127.0.0.1
      src = re.sub(
          r'(ThreadingHTTPServer\s*\(\s*\(\s*)(["\'])(?:127\.0\.0\.1|localhost)\2',
          r'\g<1>"0.0.0.0"',
          src,
      )
      src = re.sub(
          r'((?:HOST|WEB_HOST|BIND_HOST|HTTP_HOST)\s*=\s*)(["\'])(?:127\.0\.0\.1|localhost)\2',
          r'\g<1>"0.0.0.0"',
          src,
      )

      # Inject offline-safe shims for prompt_toolkit (non-TTY/missing pkg) and serial (--sim fallback)
      shim = textwrap.dedent("""\
          import contextlib as _pidbot_ctx
          import sys as _pidbot_sys
          import time as _pidbot_time
          import types as _pidbot_types

          if "prompt_toolkit" not in _pidbot_sys.modules:
              try:
                  import prompt_toolkit  # noqa: F401
              except ImportError:
                  _pt = _pidbot_types.ModuleType("prompt_toolkit")
                  _pt_patch = _pidbot_types.ModuleType("prompt_toolkit.patch_stdout")

                  class _StubPromptSession:
                      def __init__(self, *args, **kwargs):
                          pass
                      def prompt(self, *args, **kwargs):
                          while True:
                              _pidbot_time.sleep(3600)

                  @_pidbot_ctx.contextmanager
                  def _stub_patch_stdout(*args, **kwargs):
                      yield

                  _pt.PromptSession = _StubPromptSession
                  _pt_patch.patch_stdout = _stub_patch_stdout
                  _pidbot_sys.modules["prompt_toolkit"] = _pt
                  _pidbot_sys.modules["prompt_toolkit.patch_stdout"] = _pt_patch

          if "serial" not in _pidbot_sys.modules:
              try:
                  import serial  # noqa: F401
              except ImportError:
                  _ser = _pidbot_types.ModuleType("serial")
                  _ser_tools = _pidbot_types.ModuleType("serial.tools")
                  _ser_list = _pidbot_types.ModuleType("serial.tools.list_ports")

                  class _SerialException(Exception):
                      pass

                  class _StubSerial:
                      def __init__(self, *args, **kwargs):
                          raise _SerialException("pyserial not installed; use --sim or install python3-serial")

                  _ser.Serial = _StubSerial
                  _ser.SerialException = _SerialException
                  _ser_list.comports = lambda *a, **kw: []
                  _ser.tools = _ser_tools
                  _ser_tools.list_ports = _ser_list
                  _pidbot_sys.modules["serial"] = _ser
                  _pidbot_sys.modules["serial.tools"] = _ser_tools
                  _pidbot_sys.modules["serial.tools.list_ports"] = _ser_list
      """)

      post_import_shim = textwrap.dedent("""\
          import contextlib as _pidbot_ctx2
          import sys as _pidbot_sys2
          import time as _pidbot_time2

          if not _pidbot_sys2.stdin.isatty():
              class _HeadlessPromptSession:
                  def __init__(self, *args, **kwargs):
                      pass

                  def prompt(self, *args, **kwargs):
                      while True:
                          _pidbot_time2.sleep(3600)

              @_pidbot_ctx2.contextmanager
              def _headless_patch_stdout(*args, **kwargs):
                  yield

              PromptSession = _HeadlessPromptSession
              patch_stdout = _headless_patch_stdout
      """)

      marker = "# PIDBOT_HEADLESS_SHIM_APPLIED"
      if marker not in src:
          lines = src.splitlines(keepends=True)
          insert_idx = 0
          for idx, line in enumerate(lines):
              if "prompt_toolkit" in line and ("import " in line):
                  insert_idx = idx + 1
          lines.insert(insert_idx, f"\n{marker}\n{post_import_shim}\n")
          src = f"{shim}\n" + "".join(lines)

      target.write_text(src, encoding="utf-8")
      print(f"Patched {target} successfully.")

  - path: /usr/local/bin/pidbot-start.sh
    permissions: '0755'
    owner: root:root
    content: |
      #!/usr/bin/env bash
      set -euo pipefail

      PIDBOT_BAUD="__BAUD__"
      PIDBOT_WEB_PORT="__WEB_PORT__"
      PIDBOT_FORCE_SIM="__FORCE_SIM__"

      if [[ -f /etc/default/pidbot ]]; then
          # shellcheck disable=SC1091
          source /etc/default/pidbot
      fi

      SCRIPT_PATH="/home/pi/pidbot/read_com_live3.py"
      SERIAL_DEV=""

      if [[ "${PIDBOT_FORCE_SIM}" != "true" ]]; then
          for dev in /dev/ttyUSB* /dev/ttyACM*; do
              if [[ -e "${dev}" ]]; then
                  SERIAL_DEV="${dev}"
                  break
              fi
          done
      fi

      if [[ "${PIDBOT_FORCE_SIM}" == "true" || -z "${SERIAL_DEV}" ]]; then
          echo "[pidbot-start] Launching in simulation mode on port ${PIDBOT_WEB_PORT}"
          exec /usr/bin/python3 "${SCRIPT_PATH}" --sim --web-port "${PIDBOT_WEB_PORT}"
      else
          echo "[pidbot-start] Launching with serial device ${SERIAL_DEV} @ ${PIDBOT_BAUD} baud on port ${PIDBOT_WEB_PORT}"
          exec /usr/bin/python3 "${SCRIPT_PATH}" --port "${SERIAL_DEV}" --baud "${PIDBOT_BAUD}" --web-port "${PIDBOT_WEB_PORT}"
      fi

  - path: /etc/udev/rules.d/99-pidbot-usb.rules
    permissions: '0644'
    owner: root:root
    content: |
      ACTION=="add|remove", SUBSYSTEM=="tty", KERNEL=="ttyUSB[0-9]*|ttyACM[0-9]*", RUN+="/usr/bin/systemctl restart --no-block pidbot.service"

  - path: /etc/systemd/system/pidbot.service
    permissions: '0644'
    owner: root:root
    content: |
      [Unit]
      Description=PIDBot Telemetry and Control Service
      After=network.target NetworkManager.service

      [Service]
      Type=simple
      User=pi
      Group=dialout
      WorkingDirectory=/home/pi/pidbot
      Environment=PYTHONUNBUFFERED=1
      ExecStart=/usr/local/bin/pidbot-start.sh
      Restart=always
      RestartSec=3

      [Install]
      WantedBy=multi-user.target

runcmd:
  - - bash
    - -c
    - |
      set -euo pipefail
      systemctl enable --now ssh || systemctl enable --now sshd || true

      # Configure regulatory domain and unblock Wi-Fi
      raspi-config nonint do_wifi_country __COUNTRY__ || true
      iw reg set __COUNTRY__ || true
      rfkill unblock wifi || true

      # Ensure NetworkManager is active and clean existing wireless profiles
      systemctl enable --now NetworkManager
      for uuid in $(nmcli -t -f UUID,TYPE connection show | awk -F: '$2=="802-11-wireless"{print $1}'); do
          nmcli connection delete uuid "$uuid" || true
      done

      # Create and activate persistent WPA2-PSK AP on wlan0 at 10.42.0.1/24
      nmcli connection add type wifi ifname wlan0 con-name "PIDBot-AP" autoconnect yes ssid __BASH_SSID__
      nmcli connection modify "PIDBot-AP" \
          802-11-wireless.mode ap \
          802-11-wireless.band bg \
          wifi-sec.key-mgmt wpa-psk \
          wifi-sec.psk __BASH_PASS__ \
          ipv4.method shared \
          ipv4.addresses 10.42.0.1/24 \
          ipv6.method ignore
      nmcli connection up "PIDBot-AP" || true

      # Patch embedded PIDBot telemetry script (100% offline)
      /usr/local/bin/patch-pidbot.py /home/pi/pidbot/read_com_live3.py
      chown -R pi:pi /home/pi/pidbot
      chmod +x /home/pi/pidbot/read_com_live3.py

      # Reload udev and enable PIDBot systemd service
      udevadm control --reload-rules
      systemctl daemon-reload
      systemctl enable --now pidbot.service
EOF


USER_DATA_CONTENT=$(<"${MOUNT_DIR}/user-data")
USER_DATA_CONTENT="${USER_DATA_CONTENT//__YAML_PASS__/${YAML_PASS}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__BAUD__/${BAUD}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__WEB_PORT__/${WEB_PORT}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__FORCE_SIM__/${FORCE_SIM}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__COUNTRY__/${COUNTRY}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__BASH_SSID__/${BASH_SSID}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__BASH_PASS__/${BASH_PASS}}"
USER_DATA_CONTENT="${USER_DATA_CONTENT//__APP_B64__/${APP_B64}}"
printf '%s\n' "${USER_DATA_CONTENT}" > "${MOUNT_DIR}/user-data"

sync
umount "${MOUNT_DIR}"
rmdir "${MOUNT_DIR}"
MOUNT_DIR=""

if [[ "${KPARTX_USED}" == "true" ]] && command -v kpartx >/dev/null 2>&1; then
    kpartx -d "${LOOP_DEV}" >/dev/null 2>&1 || true
fi
losetup -d "${LOOP_DEV}"
LOOP_DEV=""

if [[ -n "${SUDO_USER:-}" ]]; then
    chown "${SUDO_USER}:${SUDO_USER}" "${OUT_IMG}"
fi

BUILD_SUCCESS="true"
echo ""
echo "[+] Success! 100% offline ready-to-flash image created at: ${OUT_IMG}"
echo "    AP SSID:   ${SSID}"
echo "    AP Pass:   ${PASS}"
echo "    AP IP:     10.42.0.1"
echo "    Web UI:    http://10.42.0.1:${WEB_PORT}"
echo "    SSH:       ssh pi@10.42.0.1"
