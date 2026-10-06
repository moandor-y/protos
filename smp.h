#ifndef PROTOS_SMP_H_
#define PROTOS_SMP_H_

#include <cstdint>

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

// Discovers all CPUs, initializes the BSP Local APIC, allocates dedicated
// 16 KiB per-CPU stacks from the PMM, wakes all Application Processors into
// 64-bit Long Mode via the real-mode trampoline at 0x8000, waits for every AP
// to signal that it is online, and parks all APs in a halted loop. Panics via
// `CHECK` if topology discovery, MMIO mapping, stack allocation, or AP
// bring-up fails.
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

// Returns the physical base address of the Local APIC MMIO region discovered
// during `SmpInit`. Validates with `DCHECK` that `SmpInit` has completed.
uintptr_t SmpLocalApicPhysAddr();

#if __STDC_HOSTED__
// Host unit-test hook for overriding hardware AP bring-up behavior and 32-bit
// ACPI table pointer base during host tests.
using SmpHostApBootSimFn = bool (*)(int cpu_index, CpuInfo* cpu_info);
void SmpSetHostTestHooks(uint8_t bsp_apic_id,      //
                         uintptr_t ebda_base,      //
                         uintptr_t bios_rom_base,  //
                         uintptr_t acpi32_arena,   //
                         SmpHostApBootSimFn sim_fn);
#endif

}  // namespace protos

#endif  // PROTOS_SMP_H_
