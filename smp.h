#ifndef PROTOS_SMP_H_
#define PROTOS_SMP_H_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

#include "check.h"

namespace protos {

// Maximum number of CPUs tracked by the kernel topology table.
constexpr int kMaxCpus = 64;
// Number of 4 KiB physical frames allocated per Application Processor stack
// (4 * 4 KiB = 16 KiB per AP).
constexpr int64_t kApStackFrames = 4;
// Size in bytes of each CPU's kernel stack (16 KiB).
constexpr int64_t kApStackSize = kApStackFrames * 4096;
// Default physical base address of the x86 Local APIC MMIO registers.
constexpr uintptr_t kDefaultLocalApicPhysAddr = 0xFEE00000;
// Physical address below 1 MiB where the 16-bit real-mode AP startup
// trampoline is copied before sending INIT-SIPI-SIPI.
constexpr uintptr_t kApTrampolinePhysAddr = 0x8000;
// Startup IPI (SIPI) vector corresponding to `kApTrampolinePhysAddr >> 12`.
constexpr uint8_t kApTrampolineVector = 0x08;

// Architectural IA32_GS_BASE MSR address used for per-CPU `CpuLocal` pointer.
constexpr uint32_t kMsrIa32GsBase = 0xC0000101u;

// 8254 Programmable Interval Timer (PIT) base oscillator frequency in Hz.
constexpr int64_t kPitBaseFrequencyHz = 1193182;
// PIT Channel 2 reload count used during BSP Local APIC timer calibration
// (~10.00015 ms window at 1,193,182 Hz).
constexpr int kPitCalibrationReloadCount = 11932;
// Periodic Local APIC timer target interrupt frequency in Hz.
constexpr int kApicTimerTargetHz = 250;

// Per-CPU descriptor populated during ACPI MADT discovery and AP bring-up.
struct CpuInfo {
  uint8_t apic_id;
  uint8_t acpi_processor_id;
  uint8_t observed_apic_id;
  bool is_bsp;
  bool online;
  bool long_mode_active;
  uintptr_t stack_base;
  uintptr_t stack_top;
  uintptr_t observed_rsp;
  uintptr_t observed_cr3;
};

// Cache-line-aligned per-CPU runtime state bound to `IA32_GS_BASE` on bare
// metal (`gs:[0]`) and a `thread_local` pointer in host unit tests.
struct alignas(64) CpuLocal {
  CpuLocal* self = nullptr;
  int cpu_id = 0;
  uint8_t apic_id = 0;
  bool online = false;
  uintptr_t stack_base = 0;
  uintptr_t stack_top = 0;
  std::atomic<int64_t> timer_ticks{0};
  std::atomic<int64_t> ipi_count{0};
};
static_assert(offsetof(CpuLocal, self) == 0);
static_assert(alignof(CpuLocal) == 64);
static_assert(std::is_standard_layout_v<CpuLocal>);

namespace internal {

#if !__STDC_HOSTED__
inline std::atomic<bool> g_cpu_local_bound{false};
#else
inline CpuLocal** HostCurrentCpuSlot() {
  thread_local CpuLocal* current_cpu = nullptr;
  return &current_cpu;
}
#endif

}  // namespace internal

// Binds `cpu` as the calling CPU's (or host thread's) active `CpuLocal` state
// (`IA32_GS_BASE` MSR `0xC0000101` on bare metal, `thread_local` pointer on
// host). Validates `cpu != nullptr`, `cpu->self == cpu`, and
// `0 <= cpu->cpu_id < kMaxCpus` with `DCHECK`.
inline void BindCpuLocal(CpuLocal* const cpu) {
  DCHECK(cpu != nullptr);
  DCHECK(cpu->self == cpu);
  DCHECK(cpu->cpu_id >= 0 && cpu->cpu_id < kMaxCpus);
#if !__STDC_HOSTED__
  const uint64_t addr = reinterpret_cast<uintptr_t>(cpu);
  const uint32_t low = static_cast<uint32_t>(addr & 0xFFFFFFFFu);
  const uint32_t high = static_cast<uint32_t>(addr >> 32);
  asm volatile("wrmsr" : : "c"(kMsrIa32GsBase), "a"(low), "d"(high) : "memory");
  internal::g_cpu_local_bound.store(true, std::memory_order_release);
#else
  *internal::HostCurrentCpuSlot() = cpu;
#endif
}

// Returns the calling CPU's `CpuLocal*` if bound and valid, or `nullptr` if no
// `CpuLocal` has been bound yet (safe for early panic/interrupt paths).
inline CpuLocal* CurrentCpuOrNull() {
#if !__STDC_HOSTED__
  if (!internal::g_cpu_local_bound.load(std::memory_order_acquire)) {
    return nullptr;
  }
  CpuLocal* cpu = nullptr;
  asm volatile("mov %0, qword ptr gs:[0]" : "=r"(cpu) : : "memory");
  if (cpu != nullptr && cpu->self == cpu) {
    return cpu;
  }
  return nullptr;
#else
  CpuLocal* const cpu = *internal::HostCurrentCpuSlot();
  if (cpu != nullptr && cpu->self == cpu) {
    return cpu;
  }
  return nullptr;
#endif
}

// Returns the calling CPU's `CpuLocal*` in O(1) (`gs:[0]` on bare metal,
// `thread_local` pointer on host). Validates with `DCHECK` that
// `cpu != nullptr` and `cpu->self == cpu`.
inline CpuLocal* CurrentCpu() {
#if !__STDC_HOSTED__
  CpuLocal* cpu = nullptr;
  asm volatile("mov %0, qword ptr gs:[0]" : "=r"(cpu) : : "memory");
#else
  CpuLocal* const cpu = *internal::HostCurrentCpuSlot();
#endif
  DCHECK(cpu != nullptr);
  DCHECK(cpu->self == cpu);
  return cpu;
}

// Returns the logical CPU index (`0 .. SmpCpuCount() - 1`) of the calling CPU
// in O(1) via `CurrentCpu()->cpu_id`.
inline int CurrentCpuId() { return CurrentCpu()->cpu_id; }

#if __STDC_HOSTED__
// Clears the calling host thread's bound `CpuLocal*` pointer for unit tests.
inline void ResetCpuLocalForTest() {
  *internal::HostCurrentCpuSlot() = nullptr;
}
#endif

// Discovered multiprocessor topology from ACPI MADT firmware tables.
struct SmpTopology {
  uintptr_t local_apic_phys_addr;
  uint8_t bsp_apic_id;
  int cpu_count;
  CpuInfo cpus[kMaxCpus];
};

// Scans a Multiboot2 information structure at non-zero `multiboot_info_addr`
// (bounded by non-zero `max_physical_addr`) for an embedded ACPI RSDP tag
// (preferring Tag 15 ACPI 2.0+ RSDP over Tag 14 ACPI 1.0 RSDP) and returns the
// physical address of the validated RSDP payload, or 0 if no valid RSDP tag is
// present. Validates preconditions with `DCHECK`.
uintptr_t SmpFindRsdpInMultiboot2(uintptr_t multiboot_info_addr,
                                  uintptr_t max_physical_addr);

// Scans 16-byte-aligned physical addresses in `[range_start, range_start +
// range_length)` (`range_start != 0`, `range_length > 0`) for a valid ACPI
// RSDP structure ("RSD PTR " signature and valid v1/v2 checksums). Returns the
// physical address of the RSDP, or 0 if none is found. Validates preconditions
// with `DCHECK`.
uintptr_t SmpFindRsdpInMemoryRange(uintptr_t range_start, int64_t range_length);

// Validates the ACPI RSDP structure at non-zero `rsdp_addr` (below non-zero
// `max_physical_addr`) and extracts the 32-bit RSDT physical address into
// non-null `*out_rsdt_addr` and (for revision >= 2 with valid extended
// checksum) the 64-bit XSDT physical address into non-null `*out_xsdt_addr`.
// Validates preconditions with `DCHECK`. Returns true if the RSDP is valid and
// provides at least one root table address.
bool SmpParseRsdp(uintptr_t rsdp_addr,          //
                  uintptr_t max_physical_addr,  //
                  uintptr_t* out_rsdt_addr,     //
                  uintptr_t* out_xsdt_addr);

// Searches the ACPI XSDT (at `xsdt_addr`, if non-zero and valid) and/or RSDT
// (at `rsdt_addr`, if non-zero and valid) for a Multiple APIC Description
// Table (MADT, signature "APIC") with a valid ACPI checksum. Requires at least
// one of `rsdt_addr` or `xsdt_addr` to be non-zero and `max_physical_addr > 0`
// via `DCHECK`. Returns the physical address of the MADT, or 0 if not found.
uintptr_t SmpFindMadtInSdt(uintptr_t rsdt_addr,  //
                           uintptr_t xsdt_addr,  //
                           uintptr_t max_physical_addr);

// Parses the ACPI MADT at non-zero `madt_addr` (below non-zero
// `max_physical_addr`), extracting the Local APIC MMIO base address (including
// any Type 5 64-bit override) and all enabled or online-capable processors
// (Type 0 Local APIC and Type 9 Local x2APIC entries) into non-null
// `*out_topology` with `bsp_apic_id` (`!= 0xFF`) placed at index 0. Validates
// preconditions with `DCHECK`. Returns true if the MADT is valid and at least
// one CPU is present.
bool SmpParseMadt(uintptr_t madt_addr,          //
                  uintptr_t max_physical_addr,  //
                  uint8_t bsp_apic_id,          //
                  SmpTopology* out_topology);

// Discovers the system CPU topology from Multiboot2 ACPI tags (if
// `multiboot_magic` is Multiboot2) or legacy BIOS memory ranges (`ebda_base`
// and `bios_rom_base`), populating non-null `*out_topology`. Validates
// preconditions with `DCHECK`. Returns true if a valid MADT was found and
// parsed.
bool SmpDiscoverTopology(uint32_t multiboot_magic,      //
                         uint64_t multiboot_info_addr,  //
                         uintptr_t ebda_base,           //
                         uintptr_t bios_rom_base,       //
                         uintptr_t max_physical_addr,   //
                         uint8_t bsp_apic_id,           //
                         SmpTopology* out_topology);

// Computes the periodic Local APIC timer initial count for `target_hz`
// (`0 < target_hz <= 1193182`) given `elapsed_apic_ticks` measured over
// `pit_reload_count` (`0 < pit_reload_count <= 0xFFFF`) 8254 PIT ticks at
// 1,193,182 Hz. Validates preconditions with `DCHECK`.
uint32_t ComputeApicTimerInitialCount(uint64_t elapsed_apic_ticks,  //
                                      int pit_reload_count,         //
                                      int target_hz);

// Discovers all CPUs, initializes the BSP Local APIC and `CpuLocal` state,
// calibrates the Local APIC timer using PIT Channel 2, allocates dedicated
// 16 KiB per-CPU stacks from the PMM, wakes all Application Processors into
// 64-bit Long Mode via the real-mode trampoline at 0x8000 (loading the
// 256-entry IDT, binding `CpuLocal` via `IA32_GS_BASE`, and enabling the
// periodic Local APIC timer on each CPU), and parks all APs in a maskable `sti;
// hlt` loop. Panics via `CHECK` if topology discovery, MMIO mapping, timer
// calibration, stack allocation, or AP bring-up fails.
void SmpInit(uint32_t multiboot_magic, uint64_t multiboot_info_addr);

// Returns the total number of CPUs discovered during `SmpInit`. Validates with
// `DCHECK` that `SmpInit` has completed.
int SmpCpuCount();

// Returns the number of CPUs (BSP + APs) currently online in 64-bit mode.
// Validates with `DCHECK` that `SmpInit` has completed.
int SmpOnlineCpuCount();

// Returns a non-null pointer to the descriptor for CPU `index`. Validates with
// `DCHECK` that `SmpInit` has completed and `0 <= index < SmpCpuCount()`.
const CpuInfo* SmpGetCpuInfo(int index);

// Returns a non-null pointer to the per-CPU `CpuLocal` state for CPU
// `cpu_index`. Validates with `DCHECK` that `SmpInit` has completed and
// `0 <= cpu_index < SmpCpuCount()`.
CpuLocal* SmpGetCpuLocal(int cpu_index);

// Returns the physical base address of the Local APIC MMIO region discovered
// during `SmpInit`. Validates with `DCHECK` that `SmpInit` has completed.
uintptr_t SmpLocalApicPhysAddr();

// Returns the PIT-calibrated periodic Local APIC timer initial count computed
// during `SmpInit`. Validates with `DCHECK` that `SmpInit` has completed.
uint32_t SmpCalibratedTimerInitialCount();

// Writes End-Of-Interrupt (`0`) to the Local APIC EOI register (`0x0B0`).
void SmpSendLocalApicEoi();

// Sends a fixed-delivery Inter-Processor Interrupt (IPI) on `vector`
// (`vector >= 32`) to online CPU `target_cpu_index` (`0 <= target_cpu_index <
// SmpCpuCount()`). Validates preconditions with `DCHECK`.
void SmpSendIpi(int target_cpu_index, uint8_t vector);

// Callback signature for multi-CPU work dispatched via `SmpRunOnAllCpus`.
using SmpWorkFn = void (*)(int cpu_index, void* context);

// Dispatches non-null `work_fn(cpu_index, context)` across all online CPUs
// (`0 .. SmpCpuCount() - 1`), waking idle APs via maskable IPI vector `0x21`,
// synchronizing all CPUs at a start barrier so they execute `work_fn`
// concurrently, waiting for all CPUs to complete, and returning all
// Application Processors cleanly to their `sti; hlt` idle state. Validates
// with `DCHECK` that `SmpInit` has completed and `work_fn != nullptr`.
void SmpRunOnAllCpus(SmpWorkFn work_fn, void* context);

#if __STDC_HOSTED__
// Host unit-test hooks for overriding hardware AP bring-up behavior, 32-bit
// ACPI table pointer base, and simulated PIT/APIC timer calibration ticks.
using SmpHostApBootSimFn = bool (*)(int cpu_index, CpuInfo* cpu_info);
void SmpSetHostTestHooks(uint8_t bsp_apic_id,      //
                         uintptr_t ebda_base,      //
                         uintptr_t bios_rom_base,  //
                         uintptr_t acpi32_arena,   //
                         SmpHostApBootSimFn sim_fn);
void SmpSetHostTimerCalibrationTicksForTest(uint64_t elapsed_apic_ticks);
int64_t SmpGetHostEoiCountForTest();
void SmpResetHostEoiCountForTest();
#endif

}  // namespace protos

#endif  // PROTOS_SMP_H_
