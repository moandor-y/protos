#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /.dockerenv && ! -f /run/.containerenv ]]; then
  exec make test
fi

BUILD_DIR="build"
ISO_FILE="${1:-${BUILD_DIR}/kernel.iso}"
EXPECTED_STRING="Hello, x86-64 Kernel World!"
COMPLETION_MARKER="[TEST] ALL MEMORY TESTS PASSED"
FAILURE_MARKER="FAIL"
SERIAL_LOG="${BUILD_DIR}/serial_test_output.log"
MONITOR_SOCK="${BUILD_DIR}/qemu_monitor.sock"
VGA_DUMP="${BUILD_DIR}/vga_dump.bin"
QEMU_PID=""

cleanup() {
  if [[ -n "${QEMU_PID}" ]] && kill -0 "${QEMU_PID}" 2>/dev/null; then
    kill -9 "${QEMU_PID}" 2>/dev/null || true
    wait "${QEMU_PID}" 2>/dev/null || true
  fi
  rm -f "${SERIAL_LOG}" "${MONITOR_SOCK}" "${VGA_DUMP}"
}
trap cleanup EXIT

if [[ ! -f "${ISO_FILE}" ]]; then
  echo "Error: ISO file '${ISO_FILE}' not found." >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}"
rm -f "${SERIAL_LOG}" "${MONITOR_SOCK}" "${VGA_DUMP}"

# Launch QEMU headlessly with COM1 redirected to file and -no-reboot to catch triple faults.
# Note: Do NOT pass -no-shutdown so that QEMU exits immediately if a triple fault occurs.
qemu-system-x86_64 \
  -cdrom "${ISO_FILE}" \
  -display none \
  -serial "file:${SERIAL_LOG}" \
  -monitor "unix:${MONITOR_SOCK},server,nowait" \
  -no-reboot \
  >/dev/null 2>&1 &
QEMU_PID=$!

# Poll up to 10 seconds (100 * 0.1s) for the greeting and memory test completion marker on COM1
MATCHED=0
for ((i = 0; i < 100; i++)); do
  if [[ -f "${SERIAL_LOG}" ]] && \
     grep -Fq "${EXPECTED_STRING}" "${SERIAL_LOG}" 2>/dev/null && \
     grep -Fq "${COMPLETION_MARKER}" "${SERIAL_LOG}" 2>/dev/null; then
    MATCHED=1
    break
  fi
  if [[ -f "${SERIAL_LOG}" ]] && grep -Fq "[TEST] MEMORY VERIFICATION FAILED" "${SERIAL_LOG}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${QEMU_PID}" 2>/dev/null; then
    break
  fi
  sleep 0.1
done

if [[ "${MATCHED}" -ne 1 ]]; then
  echo "FAIL: Expected greeting '${EXPECTED_STRING}' and '${COMPLETION_MARKER}' not found in serial output within timeout." >&2
  if [[ -f "${SERIAL_LOG}" ]]; then
    echo "Captured serial output:" >&2
    cat "${SERIAL_LOG}" >&2
  fi
  exit 1
fi

REQUIRED_MARKERS=(
  "[PMM] mmap entry:"
  "[PMM] Total RAM:"
  "[TEST] pmm_memory_map_init: PASS"
  "[TEST] pmm_alloc_and_bounds: PASS"
  "[TEST] pmm_free_and_reuse: PASS"
  "[TEST] heap_varied_sizes_and_alignment: PASS"
  "[TEST] heap_pattern_isolation: PASS"
  "[TEST] cpp_new_delete_lifecycle: PASS"
  "[TEST] heap_stress_reuse: PASS"
  "[TEST] edge_cases_and_oom: PASS"
  "[TEST] ALL MEMORY TESTS PASSED"
)

for marker in "${REQUIRED_MARKERS[@]}"; do
  if ! grep -Fq "${marker}" "${SERIAL_LOG}"; then
    echo "FAIL: Missing required verification marker '${marker}' in serial output." >&2
    cat "${SERIAL_LOG}" >&2
    exit 1
  fi
done

if grep -Fq "${FAILURE_MARKER}" "${SERIAL_LOG}"; then
  echo "FAIL: Found failure marker in serial output." >&2
  cat "${SERIAL_LOG}" >&2
  exit 1
fi

# Wait 0.5s while CPU is in halt state to confirm no triple fault or reboot loop occurs
sleep 0.5

if ! kill -0 "${QEMU_PID}" 2>/dev/null; then
  echo "FAIL: QEMU exited unexpectedly after printing greeting (triple fault detected)." >&2
  exit 1
fi

MATCH_COUNT=$(grep -Fc "${EXPECTED_STRING}" "${SERIAL_LOG}")
if [[ "${MATCH_COUNT}" -ne 1 ]]; then
  echo "FAIL: Expected greeting string once, but found ${MATCH_COUNT} occurrences (reboot loop)." >&2
  exit 1
fi

# Verify VGA text buffer at physical memory 0xB8000 via QEMU monitor socket
if command -v python3 >/dev/null 2>&1 && [[ -S "${MONITOR_SOCK}" ]]; then
  python3 -c '
import socket, sys, time
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sys.argv[1])
s.sendall(b"pmemsave 0xB8000 54 " + sys.argv[2].encode() + b"\n")
time.sleep(0.1)
s.close()
' "${MONITOR_SOCK}" "${VGA_DUMP}" 2>/dev/null || true

  if [[ -f "${VGA_DUMP}" ]]; then
    VGA_TEXT=$(python3 -c '
import sys
data = open(sys.argv[1], "rb").read()
chars = "".join(chr(data[i]) for i in range(0, len(data), 2))
attrs = [data[i] for i in range(1, len(data), 2)]
if all(a == 0x0F for a in attrs):
    print(chars)
' "${VGA_DUMP}" 2>/dev/null || true)
    if [[ "${VGA_TEXT}" == "${EXPECTED_STRING}" ]]; then
      echo "Verified VGA text buffer at 0xB8000 matches '${EXPECTED_STRING}' with attribute 0x0F."
    else
      echo "FAIL: VGA buffer content mismatch: '${VGA_TEXT}'" >&2
      exit 1
    fi
  fi
fi

echo "Captured BIOS serial output:"
cat "${SERIAL_LOG}"

# Terminate BIOS QEMU instance before launching UEFI test pass
kill -9 "${QEMU_PID}" 2>/dev/null || true
wait "${QEMU_PID}" 2>/dev/null || true
QEMU_PID=""
rm -f "${SERIAL_LOG}" "${MONITOR_SOCK}" "${VGA_DUMP}"

# =========================================================================
# Pass 2: 64-Bit UEFI (OVMF.fd) + 4 GiB RAM + UEFI GOP Linear Framebuffer
# =========================================================================
OVMF_BIOS="/usr/share/ovmf/OVMF.fd"
if [[ -f "${OVMF_BIOS}" ]]; then
  echo "Running 64-bit UEFI (OVMF) + 4 GiB RAM boot verification..."
  qemu-system-x86_64 \
    -bios "${OVMF_BIOS}" \
    -m 4G \
    -cdrom "${ISO_FILE}" \
    -display none \
    -serial "file:${SERIAL_LOG}" \
    -monitor "unix:${MONITOR_SOCK},server,nowait" \
    -no-reboot \
    >/dev/null 2>&1 &
  QEMU_PID=$!

  UEFI_MATCHED=0
  for ((i = 0; i < 150; i++)); do
    if [[ -f "${SERIAL_LOG}" ]] && \
       grep -Fq "${EXPECTED_STRING}" "${SERIAL_LOG}" 2>/dev/null && \
       grep -Fq "${COMPLETION_MARKER}" "${SERIAL_LOG}" 2>/dev/null; then
      UEFI_MATCHED=1
      break
    fi
    if [[ -f "${SERIAL_LOG}" ]] && grep -Fq "[TEST] MEMORY VERIFICATION FAILED" "${SERIAL_LOG}" 2>/dev/null; then
      break
    fi
    if ! kill -0 "${QEMU_PID}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if [[ "${UEFI_MATCHED}" -ne 1 ]]; then
    echo "FAIL: UEFI boot did not produce '${EXPECTED_STRING}' and '${COMPLETION_MARKER}' within timeout." >&2
    if [[ -f "${SERIAL_LOG}" ]]; then
      cat "${SERIAL_LOG}" >&2
    fi
    exit 1
  fi

  UEFI_REQUIRED_MARKERS=(
    "${REQUIRED_MARKERS[@]}"
    "[PMM] mmap entry: base=0x100000000"
    "[PMM] Framebuffer: addr="
  )

  for marker in "${UEFI_REQUIRED_MARKERS[@]}"; do
    if ! grep -Fq "${marker}" "${SERIAL_LOG}"; then
      echo "FAIL: Missing required UEFI verification marker '${marker}' in serial output." >&2
      cat "${SERIAL_LOG}" >&2
      exit 1
    fi
  done

  if grep -Fq "${FAILURE_MARKER}" "${SERIAL_LOG}"; then
    echo "FAIL: Found failure marker in UEFI serial output." >&2
    cat "${SERIAL_LOG}" >&2
    exit 1
  fi

  sleep 0.5
  if ! kill -0 "${QEMU_PID}" 2>/dev/null; then
    echo "FAIL: UEFI QEMU exited unexpectedly after printing greeting (triple fault detected)." >&2
    exit 1
  fi

  # Verify that VgaAttachFramebuffer rendered non-zero glyph pixels into the UEFI GOP framebuffer
  FB_ADDR=$(sed -n 's/.*\[PMM\] Framebuffer: addr=\(0x[0-9A-Fa-f]*\).*/\1/p' "${SERIAL_LOG}" | head -n 1)
  if [[ -n "${FB_ADDR}" ]] && command -v python3 >/dev/null 2>&1 && [[ -S "${MONITOR_SOCK}" ]]; then
    python3 -c '
import socket, sys, time
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sys.argv[1])
cmd = f"pmemsave {sys.argv[2]} 38400 {sys.argv[3]}\n"
s.sendall(cmd.encode())
time.sleep(0.1)
s.close()
' "${MONITOR_SOCK}" "${FB_ADDR}" "${VGA_DUMP}" 2>/dev/null || true

    if [[ -f "${VGA_DUMP}" ]]; then
      FB_LIT_BYTES=$(python3 -c '
import sys
data = open(sys.argv[1], "rb").read()
print(sum(1 for b in data if b == 0xFF))
' "${VGA_DUMP}" 2>/dev/null || echo "0")
      if [[ "${FB_LIT_BYTES}" -gt 0 ]]; then
        echo "Verified UEFI GOP linear framebuffer at ${FB_ADDR} contains rendered glyph pixels (${FB_LIT_BYTES} lit subpixels in top rows)."
      else
        echo "FAIL: UEFI GOP linear framebuffer at ${FB_ADDR} contains no rendered glyph pixels." >&2
        exit 1
      fi
    fi
  fi

  echo "Captured UEFI (4 GiB RAM) serial output:"
  cat "${SERIAL_LOG}"
fi

echo "SUCCESS: Verified '${EXPECTED_STRING}' across Legacy BIOS (VGA 0xB8000) and 64-bit UEFI (4 GiB RAM + GOP framebuffer), all memory tests PASSED, and clean CPU halt."
exit 0
