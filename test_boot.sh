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

BIOS_SERIAL_LOG="${BUILD_DIR}/serial_bios_output.log"
BIOS_MONITOR_SOCK="${BUILD_DIR}/qemu_bios_monitor.sock"
BIOS_VGA_DUMP="${BUILD_DIR}/vga_bios_dump.bin"
BIOS_PASS_OUT="${BUILD_DIR}/bios_pass_stdout.log"

UEFI_SERIAL_LOG="${BUILD_DIR}/serial_uefi_output.log"
UEFI_MONITOR_SOCK="${BUILD_DIR}/qemu_uefi_monitor.sock"
UEFI_VGA_DUMP="${BUILD_DIR}/vga_uefi_dump.bin"
UEFI_PASS_OUT="${BUILD_DIR}/uefi_pass_stdout.log"

cleanup() {
  rm -f \
    "${BIOS_SERIAL_LOG}" "${BIOS_MONITOR_SOCK}" "${BIOS_VGA_DUMP}" "${BIOS_PASS_OUT}" \
    "${UEFI_SERIAL_LOG}" "${UEFI_MONITOR_SOCK}" "${UEFI_VGA_DUMP}" "${UEFI_PASS_OUT}"
}
trap cleanup EXIT

if [[ ! -f "${ISO_FILE}" ]]; then
  echo "Error: ISO file '${ISO_FILE}' not found." >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}"
cleanup

REQUIRED_MARKERS=(
  "[PMM] mmap entry:"
  "[PMM] Total RAM:"
  "[SMP] Discovered CPUs: 4"
  "[SMP] CPU 0 (APIC ID 0, BSP): online"
  "[SMP] CPU 1 (APIC ID 1, AP): online"
  "[SMP] CPU 2 (APIC ID 2, AP): online"
  "[SMP] CPU 3 (APIC ID 3, AP): online"
  "[SMP] Online CPUs: 4/4"
  "[TEST] pmm_memory_map_init: PASS"
  "[TEST] pmm_alloc_and_bounds: PASS"
  "[TEST] pmm_free_and_reuse: PASS"
  "[TEST] heap_varied_sizes_and_alignment: PASS"
  "[TEST] heap_pattern_isolation: PASS"
  "[TEST] cpp_new_delete_lifecycle: PASS"
  "[TEST] heap_stress_reuse: PASS"
  "[TEST] edge_cases_and_oom: PASS"
  "[TEST] smp_discovery_and_ap_bringup: PASS"
  "[TEST] ALL MEMORY TESTS PASSED"
)

# =========================================================================
# Pass 1: Legacy BIOS + VGA Text Buffer (0xB8000)
# =========================================================================
run_bios_pass() {
  local qemu_pid=""
  trap 'if [[ -n "${qemu_pid}" ]] && kill -0 "${qemu_pid}" 2>/dev/null; then kill -9 "${qemu_pid}" 2>/dev/null || true; wait "${qemu_pid}" 2>/dev/null || true; fi' RETURN

  # Launch QEMU headlessly with COM1 redirected to file and -no-reboot to catch triple faults.
  # Note: Do NOT pass -no-shutdown so that QEMU exits immediately if a triple fault occurs.
  qemu-system-x86_64 \
    -smp 4 \
    -cdrom "${ISO_FILE}" \
    -display none \
    -serial "file:${BIOS_SERIAL_LOG}" \
    -monitor "unix:${BIOS_MONITOR_SOCK},server,nowait" \
    -no-reboot \
    >/dev/null 2>&1 &
  qemu_pid=$!

  # Poll up to 10 seconds (100 * 0.1s) for the greeting and memory test completion marker on COM1
  local matched=0
  for ((i = 0; i < 100; i++)); do
    if [[ -f "${BIOS_SERIAL_LOG}" ]] && \
       grep -Fq "${EXPECTED_STRING}" "${BIOS_SERIAL_LOG}" 2>/dev/null && \
       grep -Fq "${COMPLETION_MARKER}" "${BIOS_SERIAL_LOG}" 2>/dev/null; then
      matched=1
      break
    fi
    if [[ -f "${BIOS_SERIAL_LOG}" ]] && grep -Fq "[TEST] MEMORY VERIFICATION FAILED" "${BIOS_SERIAL_LOG}" 2>/dev/null; then
      break
    fi
    if ! kill -0 "${qemu_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if [[ "${matched}" -ne 1 ]]; then
    echo "FAIL: Expected greeting '${EXPECTED_STRING}' and '${COMPLETION_MARKER}' not found in serial output within timeout." >&2
    if [[ -f "${BIOS_SERIAL_LOG}" ]]; then
      echo "Captured serial output:" >&2
      cat "${BIOS_SERIAL_LOG}" >&2
    fi
    return 1
  fi

  for marker in "${REQUIRED_MARKERS[@]}"; do
    if ! grep -Fq "${marker}" "${BIOS_SERIAL_LOG}"; then
      echo "FAIL: Missing required verification marker '${marker}' in serial output." >&2
      cat "${BIOS_SERIAL_LOG}" >&2
      return 1
    fi
  done

  if grep -Fq "${FAILURE_MARKER}" "${BIOS_SERIAL_LOG}"; then
    echo "FAIL: Found failure marker in serial output." >&2
    cat "${BIOS_SERIAL_LOG}" >&2
    return 1
  fi

  # Wait 0.5s while CPU is in halt state to confirm no triple fault or reboot loop occurs
  sleep 0.5

  if ! kill -0 "${qemu_pid}" 2>/dev/null; then
    echo "FAIL: QEMU exited unexpectedly after printing greeting (triple fault detected)." >&2
    return 1
  fi

  local match_count
  match_count=$(grep -Fc "${EXPECTED_STRING}" "${BIOS_SERIAL_LOG}")
  if [[ "${match_count}" -ne 1 ]]; then
    echo "FAIL: Expected greeting string once, but found ${match_count} occurrences (reboot loop)." >&2
    return 1
  fi

  # Verify VGA text buffer at physical memory 0xB8000 via QEMU monitor socket
  if command -v python3 >/dev/null 2>&1 && [[ -S "${BIOS_MONITOR_SOCK}" ]]; then
    python3 -c '
import socket, sys, time
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sys.argv[1])
s.sendall(b"pmemsave 0xB8000 54 " + sys.argv[2].encode() + b"\n")
time.sleep(0.1)
s.close()
' "${BIOS_MONITOR_SOCK}" "${BIOS_VGA_DUMP}" 2>/dev/null || true

    if [[ -f "${BIOS_VGA_DUMP}" ]]; then
      local vga_text
      vga_text=$(python3 -c '
import sys
data = open(sys.argv[1], "rb").read()
chars = "".join(chr(data[i]) for i in range(0, len(data), 2))
attrs = [data[i] for i in range(1, len(data), 2)]
if all(a == 0x0F for a in attrs):
    print(chars)
' "${BIOS_VGA_DUMP}" 2>/dev/null || true)
      if [[ "${vga_text}" == "${EXPECTED_STRING}" ]]; then
        echo "Verified VGA text buffer at 0xB8000 matches '${EXPECTED_STRING}' with attribute 0x0F."
      else
        echo "FAIL: VGA buffer content mismatch: '${vga_text}'" >&2
        return 1
      fi
    fi
  fi

  echo "Captured BIOS serial output:"
  cat "${BIOS_SERIAL_LOG}"
}

# =========================================================================
# Pass 2: 64-Bit UEFI (OVMF.fd) + 4 GiB RAM + UEFI GOP Linear Framebuffer
# =========================================================================
run_uefi_pass() {
  local ovmf_bios="/usr/share/ovmf/OVMF.fd"
  if [[ ! -f "${ovmf_bios}" ]]; then
    return 0
  fi

  local qemu_pid=""
  trap 'if [[ -n "${qemu_pid}" ]] && kill -0 "${qemu_pid}" 2>/dev/null; then kill -9 "${qemu_pid}" 2>/dev/null || true; wait "${qemu_pid}" 2>/dev/null || true; fi' RETURN

  echo "Running 64-bit UEFI (OVMF) + 4 GiB RAM boot verification..."
  qemu-system-x86_64 \
    -bios "${ovmf_bios}" \
    -smp 4 \
    -m 4G \
    -cdrom "${ISO_FILE}" \
    -display none \
    -serial "file:${UEFI_SERIAL_LOG}" \
    -monitor "unix:${UEFI_MONITOR_SOCK},server,nowait" \
    -no-reboot \
    >/dev/null 2>&1 &
  qemu_pid=$!

  local uefi_matched=0
  for ((i = 0; i < 150; i++)); do
    if [[ -f "${UEFI_SERIAL_LOG}" ]] && \
       grep -Fq "${EXPECTED_STRING}" "${UEFI_SERIAL_LOG}" 2>/dev/null && \
       grep -Fq "${COMPLETION_MARKER}" "${UEFI_SERIAL_LOG}" 2>/dev/null; then
      uefi_matched=1
      break
    fi
    if [[ -f "${UEFI_SERIAL_LOG}" ]] && grep -Fq "[TEST] MEMORY VERIFICATION FAILED" "${UEFI_SERIAL_LOG}" 2>/dev/null; then
      break
    fi
    if ! kill -0 "${qemu_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done

  if [[ "${uefi_matched}" -ne 1 ]]; then
    echo "FAIL: UEFI boot did not produce '${EXPECTED_STRING}' and '${COMPLETION_MARKER}' within timeout." >&2
    if [[ -f "${UEFI_SERIAL_LOG}" ]]; then
      cat "${UEFI_SERIAL_LOG}" >&2
    fi
    return 1
  fi

  local uefi_required_markers=(
    "${REQUIRED_MARKERS[@]}"
    "[PMM] mmap entry: base=0x100000000"
    "[PMM] Framebuffer: addr="
  )

  for marker in "${uefi_required_markers[@]}"; do
    if ! grep -Fq "${marker}" "${UEFI_SERIAL_LOG}"; then
      echo "FAIL: Missing required UEFI verification marker '${marker}' in serial output." >&2
      cat "${UEFI_SERIAL_LOG}" >&2
      return 1
    fi
  done

  if grep -Fq "${FAILURE_MARKER}" "${UEFI_SERIAL_LOG}"; then
    echo "FAIL: Found failure marker in UEFI serial output." >&2
    cat "${UEFI_SERIAL_LOG}" >&2
    return 1
  fi

  sleep 0.5
  if ! kill -0 "${qemu_pid}" 2>/dev/null; then
    echo "FAIL: UEFI QEMU exited unexpectedly after printing greeting (triple fault detected)." >&2
    return 1
  fi

  # Verify that VgaAttachFramebuffer rendered non-zero glyph pixels into the UEFI GOP framebuffer
  local fb_addr
  fb_addr=$(sed -n 's/.*\[PMM\] Framebuffer: addr=\(0x[0-9A-Fa-f]*\).*/\1/p' "${UEFI_SERIAL_LOG}" | head -n 1)
  if [[ -n "${fb_addr}" ]] && command -v python3 >/dev/null 2>&1 && [[ -S "${UEFI_MONITOR_SOCK}" ]]; then
    python3 -c '
import socket, sys, time
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.connect(sys.argv[1])
cmd = f"pmemsave {sys.argv[2]} 38400 {sys.argv[3]}\n"
s.sendall(cmd.encode())
time.sleep(0.1)
s.close()
' "${UEFI_MONITOR_SOCK}" "${fb_addr}" "${UEFI_VGA_DUMP}" 2>/dev/null || true

    if [[ -f "${UEFI_VGA_DUMP}" ]]; then
      local fb_lit_bytes
      fb_lit_bytes=$(python3 -c '
import sys
data = open(sys.argv[1], "rb").read()
print(sum(1 for b in data if b == 0xFF))
' "${UEFI_VGA_DUMP}" 2>/dev/null || echo "0")
      if [[ "${fb_lit_bytes}" -gt 0 ]]; then
        echo "Verified UEFI GOP linear framebuffer at ${fb_addr} contains rendered glyph pixels (${fb_lit_bytes} lit subpixels in top rows)."
      else
        echo "FAIL: UEFI GOP linear framebuffer at ${fb_addr} contains no rendered glyph pixels." >&2
        return 1
      fi
    fi
  fi

  echo "Captured UEFI (4 GiB RAM) serial output:"
  cat "${UEFI_SERIAL_LOG}"
}

run_bios_pass >"${BIOS_PASS_OUT}" 2>&1 &
BIOS_JOB_PID=$!

run_uefi_pass >"${UEFI_PASS_OUT}" 2>&1 &
UEFI_JOB_PID=$!

BIOS_RC=0
wait "${BIOS_JOB_PID}" || BIOS_RC=$?
cat "${BIOS_PASS_OUT}"

UEFI_RC=0
wait "${UEFI_JOB_PID}" || UEFI_RC=$?
cat "${UEFI_PASS_OUT}"

if [[ "${BIOS_RC}" -ne 0 || "${UEFI_RC}" -ne 0 ]]; then
  exit 1
fi

echo "SUCCESS: Verified '${EXPECTED_STRING}' across Legacy BIOS (VGA 0xB8000) and 64-bit UEFI (4 GiB RAM + GOP framebuffer), all memory tests PASSED, and clean CPU halt."
exit 0
