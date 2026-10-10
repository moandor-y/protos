#include "smp.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <type_traits>
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"
#include "idt.h"
#include "paging.h"
#include "pmm.h"
#include "spinlock.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

extern "C" {
#if !__STDC_HOSTED__
extern const uint8_t ap_trampoline_start[];
extern const uint8_t ap_trampoline_end[];
extern const uint8_t stack_bottom[];
extern const uint8_t stack_top[];
#endif

// Per-AP bootstrap handoff state read by the 16/32/64-bit trampoline in boot.S
// while the BSP brings up one Application Processor at a time.
// Top of the 16 KiB kernel stack allocated for the AP currently being booted.
volatile uint64_t g_ap_boot_stack = 0;
// Logical CPU index (1 .. cpu_count - 1) assigned to the AP being booted.
volatile int g_ap_boot_cpu_index = 0;
// Set to 1 by the AP trampoline in boot.S as soon as the AP enters 32-bit
// protected mode, allowing the BSP to skip sending a redundant second SIPI.
std::atomic<int> g_ap_boot_started{0};
static_assert(sizeof(g_ap_boot_started) == sizeof(int));
static_assert(std::is_standard_layout_v<std::atomic<int>>);
}

namespace protos {

namespace {

constexpr uint32_t kMultiboot2Magic = 0x36D76289;
constexpr uint32_t kMultiboot2TagEnd = 0;
constexpr uint32_t kMultiboot2TagAcpiOld = 14;
constexpr uint32_t kMultiboot2TagAcpiNew = 15;
constexpr int64_t kMaxMultibootStructureBytes = 4 * 1024 * 1024;
constexpr int64_t kMaxAcpiTableBytes = 256 * 1024;
constexpr int64_t kMaxRsdpBytes = 4096;
constexpr int64_t kEbdaScanBytes = 1024;
constexpr uintptr_t kEbdaMinAddress = 0x00080000;
constexpr uintptr_t kEbdaMaxAddress = 0x000A0000;
constexpr uintptr_t kBiosRomSearchBase = 0x000E0000;
constexpr int64_t kBiosRomScanBytes = 0x00020000;
constexpr int64_t kRsdpScanAlignment = 16;

constexpr uint8_t kMadtTypeLocalApic = 0;
constexpr uint8_t kMadtTypeLocalApicOverride = 5;
constexpr uint8_t kMadtTypeLocalX2Apic = 9;
constexpr uint8_t kInvalidXapicId = 0xFFu;
constexpr uint32_t kMadtLapicFlagEnabled = 1u << 0;
constexpr uint32_t kMadtLapicFlagOnlineCapable = 1u << 1;
constexpr uint32_t kMadtLapicUsableMask =
    kMadtLapicFlagEnabled | kMadtLapicFlagOnlineCapable;
constexpr int kMaxPitReloadCount = 0xFFFF;
constexpr uint8_t kPitSpeakerOut2Bit = 1u << 5;

#if !__STDC_HOSTED__
constexpr uint32_t kMsrIa32ApicBase = 0x0000001B;
constexpr uint32_t kMsrIa32Efer = 0xC0000080;
constexpr uint64_t kApicBaseGlobalEnable = 1ULL << 11;
constexpr uint64_t kEferLongModeActive = 1ULL << 10;
constexpr uint64_t kKernelCodeSelector = 0x08;

constexpr int kLapicRegId = 0x020;
constexpr int kLapicRegTpr = 0x080;
constexpr int kLapicRegEoi = 0x0B0;
constexpr int kLapicRegSvr = 0x0F0;
constexpr int kLapicRegEsr = 0x280;
constexpr int kLapicRegIcrLow = 0x300;
constexpr int kLapicRegIcrHigh = 0x310;
constexpr int kLapicRegLvtTimer = 0x320;
constexpr int kLapicRegTimerInitCount = 0x380;
constexpr int kLapicRegTimerCurrCount = 0x390;
constexpr int kLapicRegTimerDivConfig = 0x3E0;

constexpr uint32_t kLapicSvrSoftwareEnable = 1u << 8;
constexpr uint32_t kLapicSpuriousVector = 0xFFu;
constexpr uint32_t kLapicTimerDivBy16 = 0x03u;
constexpr uint32_t kLapicTimerMaskBit = 1u << 16;
constexpr uint32_t kLapicTimerPeriodicMode = 1u << 17;

constexpr uint32_t kIcrDeliveryFixed = 0x00000000u;
constexpr uint32_t kIcrDeliveryInit = 0x00000500u;
constexpr uint32_t kIcrDeliveryStartup = 0x00000600u;
constexpr uint32_t kIcrDeliveryPending = 1u << 12;
constexpr uint32_t kIcrLevelAssert = 1u << 14;
constexpr uint32_t kIcrTriggerLevel = 1u << 15;

constexpr uint16_t kPitChannel2DataPort = 0x42;
constexpr uint16_t kPitCommandPort = 0x43;
constexpr uint16_t kPitSpeakerPort = 0x61;
constexpr uint8_t kPitCmdChannel2Mode0 = 0xB0;
constexpr uint8_t kPitSpeakerGate2Bit = 1u << 0;
constexpr uint8_t kPitSpeakerEnableMask = 0x03u;

constexpr int64_t kMaxTrampolineBytes = 512;
constexpr int kIcrIdlePollLimit = 1000000;
constexpr int kInitDelayIterations = 50000;
constexpr int kFirstSipiPollLimit = 200000;
#endif
constexpr int kSecondSipiPollLimit = 50000000;
constexpr int kIpiRetryPollInterval = 4096;

struct [[gnu::packed]] Multiboot2InfoHeader {
  uint32_t total_size;
  uint32_t reserved;
};

struct [[gnu::packed]] Multiboot2Tag {
  uint32_t type;
  uint32_t size;
};

struct [[gnu::packed]] AcpiRsdpV1 {
  char signature[8];
  uint8_t checksum;
  char oem_id[6];
  uint8_t revision;
  uint32_t rsdt_address;
};

struct [[gnu::packed]] AcpiRsdpV2 {
  AcpiRsdpV1 v1;
  uint32_t length;
  uint64_t xsdt_address;
  uint8_t extended_checksum;
  uint8_t reserved[3];
};

struct [[gnu::packed]] AcpiSdtHeader {
  char signature[4];
  uint32_t length;
  uint8_t revision;
  uint8_t checksum;
  char oem_id[6];
  char oem_table_id[8];
  uint32_t oem_revision;
  uint32_t creator_id;
  uint32_t creator_revision;
};

struct [[gnu::packed]] AcpiMadt {
  AcpiSdtHeader header;
  uint32_t local_apic_address;
  uint32_t flags;
};

struct [[gnu::packed]] AcpiMadtEntryHeader {
  uint8_t type;
  uint8_t length;
};

struct [[gnu::packed]] AcpiMadtLocalApic {
  AcpiMadtEntryHeader header;
  uint8_t acpi_processor_id;
  uint8_t apic_id;
  uint32_t flags;
};

struct [[gnu::packed]] AcpiMadtLocalApicOverride {
  AcpiMadtEntryHeader header;
  uint16_t reserved;
  uint64_t local_apic_address;
};

struct [[gnu::packed]] AcpiMadtLocalX2Apic {
  AcpiMadtEntryHeader header;
  uint16_t reserved;
  uint32_t x2apic_id;
  uint32_t flags;
  uint32_t acpi_processor_uid;
};

// Discovered CPU topology and per-CPU runtime state populated during SmpInit().
SmpTopology g_topology = {};
// Cache-line-aligned per-CPU state blocks (`0 .. SmpCpuCount() - 1`).
std::unique_ptr<CpuLocal[]> g_cpu_locals;
// Calibrated periodic Local APIC timer initial count computed on the BSP.
uint32_t g_calibrated_timer_initial_count = 0;
// Number of CPUs (BSP + APs) that have completed initialization and come
// online.
std::atomic<int> g_online_cpu_count{0};
// Handshake flag set to 1 by the currently booting AP in ApKernelEntry() once
// it has initialized its Local APIC and recorded its hardware state, signaling
// the BSP that it may proceed to wake the next AP.
std::atomic<int> g_ap_boot_done{0};
// True once SmpInit() has completed topology discovery and AP bring-up.
bool g_smp_initialized = false;

// State for synchronous multi-CPU work dispatch in SmpRunOnAllCpus():
// Guards against concurrent or re-entrant SmpRunOnAllCpus() calls.
std::atomic<bool> g_dispatch_in_progress{false};
// Callback function and opaque context pointer for the current dispatch round.
std::atomic<SmpWorkFn> g_work_fn{nullptr};
std::atomic<void*> g_work_context{nullptr};
// Monotonically increasing dispatch epoch incremented by the BSP to announce a
// new work item before waking halted APs via maskable `0x21` IPIs.
std::atomic<int64_t> g_work_epoch{0};
// Set by the BSP to `g_work_epoch` once all APs have acknowledged the wakeup,
// releasing all CPUs to execute `g_work_fn` simultaneously.
std::atomic<int64_t> g_work_start_epoch{0};
// Per-CPU epoch acknowledgment written by AP `i` upon waking from `hlt`, so the
// BSP knows whether an AP received the wakeup IPI or needs a retry.
std::unique_ptr<std::atomic<int64_t>[]> g_ap_ack_epoch;
// Per-CPU completion epoch written by AP `i` after `g_work_fn` returns, before
// the AP goes back to `sti; hlt`.
std::unique_ptr<std::atomic<int64_t>[]> g_ap_done_epoch;

#if __STDC_HOSTED__
constexpr uint64_t kDefaultHostCalibrationTicks = 625000;

// Host unit-test overrides configured via SmpSetHostTestHooks().
uint8_t g_host_bsp_apic_id = 0;
uintptr_t g_host_ebda_base = 0;
uintptr_t g_host_bios_rom_base = 0;
uintptr_t g_host_acpi32_high_bits = 0;
SmpHostApBootSimFn g_host_ap_sim_fn = nullptr;
uint64_t g_host_calibration_ticks = kDefaultHostCalibrationTicks;
std::atomic<int64_t> g_host_eoi_count{0};

struct HostApRunner {
  int count = 0;
  std::unique_ptr<std::thread[]> threads;
  std::unique_ptr<int64_t[]> spawned_epoch;
  std::unique_ptr<int64_t[]> completed_epoch;

  void EnsureCapacity(const int cpu_count) {
    if (count >= cpu_count) {
      return;
    }
    Reset();
    count = cpu_count;
    threads.reset(new std::thread[cpu_count]);
    spawned_epoch.reset(new int64_t[cpu_count]());
    completed_epoch.reset(new int64_t[cpu_count]());
  }

  void Reset() {
    if (threads != nullptr) {
      for (int i = 0; i < count; ++i) {
        if (threads[i].joinable()) {
          threads[i].join();
        }
      }
    }
    threads.reset();
    spawned_epoch.reset();
    completed_epoch.reset();
    count = 0;
  }

  ~HostApRunner() { Reset(); }
};

HostApRunner g_host_ap_runner;
#endif

static constexpr int64_t AlignUp(const int64_t value, const int64_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

static uintptr_t ResolveAcpi32Address(const uint32_t addr32) {
  if (addr32 == 0) {
    return 0;
  }
#if __STDC_HOSTED__
  return g_host_acpi32_high_bits | static_cast<uintptr_t>(addr32);
#else
  return static_cast<uintptr_t>(addr32);
#endif
}

static void ConsoleWrite(const char* const str) {
  DCHECK(str != nullptr);
  UartWrite(str);
  VgaWrite(str);
}

static void ConsoleWriteHex(const uint64_t value) {
  UartWriteHex(value);
  VgaWriteHex(value);
}

static void ConsoleWriteDec(const uint64_t value) {
  UartWriteDec(value);
  VgaWriteDec(value);
}

static bool BytesMatch(const char* const actual,    //
                       const char* const expected,  //
                       const int length) {
  DCHECK(actual != nullptr);
  DCHECK(expected != nullptr);
  DCHECK(length > 0);
  for (int i = 0; i < length; ++i) {
    if (actual[i] != expected[i]) {
      return false;
    }
  }
  return true;
}

static bool VerifyChecksum(const void* const data, const int64_t length) {
  DCHECK(data != nullptr);
  DCHECK(length > 0);
  const uint8_t* const bytes = static_cast<const uint8_t*>(data);
  uint8_t sum = 0;
  for (int64_t i = 0; i < length; ++i) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return sum == 0;
}

static bool ValidateRsdpV1Bounded(const uintptr_t rsdp_addr,          //
                                  const uintptr_t rsdp_max_end_addr,  //
                                  const uintptr_t max_physical_addr,  //
                                  uintptr_t* const out_rsdt_addr) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_rsdt_addr != nullptr);
  *out_rsdt_addr = 0;

  constexpr int64_t kV1Size = sizeof(AcpiRsdpV1);
  const uintptr_t structure_limit = (rsdp_max_end_addr < max_physical_addr)
                                        ? rsdp_max_end_addr
                                        : max_physical_addr;
  if (rsdp_addr >= structure_limit ||
      (structure_limit - rsdp_addr) < static_cast<uintptr_t>(kV1Size) ||
      !PagingMapBootstrapRange(rsdp_addr, kV1Size)) {
    return false;
  }
  const AcpiRsdpV1* const rsdp = reinterpret_cast<const AcpiRsdpV1*>(rsdp_addr);
  if (!BytesMatch(rsdp->signature, "RSD PTR ", 8) ||
      !VerifyChecksum(rsdp, kV1Size)) {
    return false;
  }
  const uintptr_t rsdt_addr = ResolveAcpi32Address(rsdp->rsdt_address);
  if (rsdt_addr == 0 || rsdt_addr >= max_physical_addr) {
    return false;
  }
  *out_rsdt_addr = rsdt_addr;
  return true;
}

static bool ParseRsdpBounded(const uintptr_t rsdp_addr,          //
                             const uintptr_t rsdp_max_end_addr,  //
                             const uintptr_t max_physical_addr,  //
                             uintptr_t* const out_rsdt_addr,     //
                             uintptr_t* const out_xsdt_addr) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_rsdt_addr != nullptr);
  DCHECK(out_xsdt_addr != nullptr);
  *out_rsdt_addr = 0;
  *out_xsdt_addr = 0;

  constexpr int64_t kV1Size = sizeof(AcpiRsdpV1);
  constexpr int64_t kV2Size = sizeof(AcpiRsdpV2);
  const uintptr_t structure_limit = (rsdp_max_end_addr < max_physical_addr)
                                        ? rsdp_max_end_addr
                                        : max_physical_addr;
  if (rsdp_addr >= structure_limit ||
      (structure_limit - rsdp_addr) < static_cast<uintptr_t>(kV1Size) ||
      !PagingMapBootstrapRange(rsdp_addr, kV1Size)) {
    return false;
  }

  const AcpiRsdpV1* const rsdp_v1 =
      reinterpret_cast<const AcpiRsdpV1*>(rsdp_addr);
  if (!BytesMatch(rsdp_v1->signature, "RSD PTR ", 8) ||
      !VerifyChecksum(rsdp_v1, kV1Size)) {
    return false;
  }

  const uintptr_t rsdt_addr = ResolveAcpi32Address(rsdp_v1->rsdt_address);
  if (rsdt_addr != 0 && rsdt_addr < max_physical_addr) {
    *out_rsdt_addr = rsdt_addr;
  }

  if (rsdp_v1->revision >= 2) {
    if ((structure_limit - rsdp_addr) < static_cast<uintptr_t>(kV2Size) ||
        !PagingMapBootstrapRange(rsdp_addr, kV2Size)) {
      *out_rsdt_addr = 0;
      return false;
    }
    const AcpiRsdpV2* const rsdp_v2 =
        reinterpret_cast<const AcpiRsdpV2*>(rsdp_addr);
    const int64_t v2_len = rsdp_v2->length;
    if (v2_len < kV2Size || v2_len > kMaxRsdpBytes ||
        (structure_limit - rsdp_addr) < static_cast<uintptr_t>(v2_len) ||
        !PagingMapBootstrapRange(rsdp_addr, v2_len) ||
        !VerifyChecksum(rsdp_v2, v2_len)) {
      *out_rsdt_addr = 0;
      return false;
    }
    const uintptr_t xsdt_addr = rsdp_v2->xsdt_address;
    if (xsdt_addr != 0 && xsdt_addr < max_physical_addr) {
      *out_xsdt_addr = xsdt_addr;
    }
  }

  return *out_rsdt_addr != 0 || *out_xsdt_addr != 0;
}

static bool FindRsdpTagsInMultiboot2(const uintptr_t multiboot_info_addr,  //
                                     const uintptr_t max_physical_addr,    //
                                     uintptr_t* const out_new_addr,        //
                                     uintptr_t* const out_new_end_addr,    //
                                     uintptr_t* const out_old_addr,        //
                                     uintptr_t* const out_old_end_addr) {
  DCHECK(multiboot_info_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_new_addr != nullptr);
  DCHECK(out_new_end_addr != nullptr);
  DCHECK(out_old_addr != nullptr);
  DCHECK(out_old_end_addr != nullptr);
  *out_new_addr = 0;
  *out_new_end_addr = 0;
  *out_old_addr = 0;
  *out_old_end_addr = 0;

  constexpr int64_t kHeaderSize = sizeof(Multiboot2InfoHeader);
  if (multiboot_info_addr >= max_physical_addr ||
      (max_physical_addr - multiboot_info_addr) <
          static_cast<uintptr_t>(kHeaderSize) ||
      !PagingMapBootstrapRange(multiboot_info_addr, kHeaderSize)) {
    return false;
  }

  const Multiboot2InfoHeader* const header =
      reinterpret_cast<const Multiboot2InfoHeader*>(multiboot_info_addr);
  const int64_t total_size = header->total_size;
  if (total_size < kHeaderSize || total_size > kMaxMultibootStructureBytes ||
      (max_physical_addr - multiboot_info_addr) <
          static_cast<uintptr_t>(total_size) ||
      !PagingMapBootstrapRange(multiboot_info_addr, total_size)) {
    return false;
  }

  int64_t offset = kHeaderSize;
  constexpr int64_t kTagHeaderSize = sizeof(Multiboot2Tag);
  constexpr int64_t kMinOldTagSize = kTagHeaderSize + sizeof(AcpiRsdpV1);
  constexpr int64_t kMinNewTagSize = kTagHeaderSize + sizeof(AcpiRsdpV2);

  while (offset + kTagHeaderSize <= total_size) {
    const Multiboot2Tag* const tag =
        reinterpret_cast<const Multiboot2Tag*>(multiboot_info_addr + offset);
    const int64_t tag_size = tag->size;
    if (tag->type == kMultiboot2TagEnd || tag_size < kTagHeaderSize ||
        offset + tag_size > total_size) {
      break;
    }

    const uintptr_t payload_addr =
        multiboot_info_addr + offset + kTagHeaderSize;
    const uintptr_t tag_end_addr = multiboot_info_addr + offset + tag_size;
    if (tag->type == kMultiboot2TagAcpiNew && tag_size >= kMinNewTagSize &&
        *out_new_addr == 0) {
      uintptr_t rsdt = 0;
      uintptr_t xsdt = 0;
      if (ParseRsdpBounded(payload_addr,       //
                           tag_end_addr,       //
                           max_physical_addr,  //
                           &rsdt,              //
                           &xsdt)) {
        *out_new_addr = payload_addr;
        *out_new_end_addr = tag_end_addr;
      } else if (*out_old_addr == 0 &&
                 ValidateRsdpV1Bounded(payload_addr,       //
                                       tag_end_addr,       //
                                       max_physical_addr,  //
                                       &rsdt)) {
        *out_old_addr = payload_addr;
        *out_old_end_addr = tag_end_addr;
      }
    } else if (tag->type == kMultiboot2TagAcpiOld &&
               tag_size >= kMinOldTagSize && *out_old_addr == 0) {
      uintptr_t rsdt = 0;
      if (ValidateRsdpV1Bounded(payload_addr,       //
                                tag_end_addr,       //
                                max_physical_addr,  //
                                &rsdt)) {
        *out_old_addr = payload_addr;
        *out_old_end_addr = tag_end_addr;
      }
    }
    offset = AlignUp(offset + tag_size, 8);
  }

  return *out_new_addr != 0 || *out_old_addr != 0;
}

static const AcpiSdtHeader* MapAndValidateSdt(
    const uintptr_t table_addr,         //
    const uintptr_t max_physical_addr,  //
    const char* const expected_sig) {
  DCHECK(max_physical_addr > 0);
  DCHECK(expected_sig != nullptr);
  constexpr int64_t kHeaderSize = sizeof(AcpiSdtHeader);
  if (table_addr == 0 || table_addr >= max_physical_addr ||
      (max_physical_addr - table_addr) < static_cast<uintptr_t>(kHeaderSize) ||
      !PagingMapBootstrapRange(table_addr, kHeaderSize)) {
    return nullptr;
  }
  const AcpiSdtHeader* const header =
      reinterpret_cast<const AcpiSdtHeader*>(table_addr);
  if (!BytesMatch(header->signature, expected_sig, 4)) {
    return nullptr;
  }
  const int64_t total_len = header->length;
  if (total_len < kHeaderSize || total_len > kMaxAcpiTableBytes ||
      (max_physical_addr - table_addr) < static_cast<uintptr_t>(total_len) ||
      !PagingMapBootstrapRange(table_addr, total_len) ||
      !VerifyChecksum(header, total_len)) {
    return nullptr;
  }
  return header;
}

static bool IsValidLocalApicAddress(const uintptr_t addr,
                                    const uintptr_t max_physical_addr) {
  return addr != 0 && (addr & (kPageSize - 1)) == 0 &&
         addr < max_physical_addr &&
         (max_physical_addr - addr) >= static_cast<uintptr_t>(kPageSize);
}

static void AddDiscoveredCpu(SmpTopology* const topology,  //
                             const int capacity,           //
                             const uint8_t apic_id,        //
                             const uint8_t acpi_processor_id) {
  DCHECK(topology != nullptr);
  DCHECK(topology->cpus != nullptr);
  DCHECK(apic_id != kInvalidXapicId);
  for (int i = 0; i < topology->cpu_count; ++i) {
    if (topology->cpus[i].apic_id == apic_id) {
      return;
    }
  }
  DCHECK(topology->cpu_count < capacity);
  CpuInfo& cpu = topology->cpus[topology->cpu_count];
  cpu = {};
  cpu.apic_id = apic_id;
  cpu.acpi_processor_id = acpi_processor_id;
  ++topology->cpu_count;
}

static bool TryDiscoverFromRsdp(const uintptr_t rsdp_addr,          //
                                const uintptr_t rsdp_max_end_addr,  //
                                const uintptr_t max_physical_addr,  //
                                const uint8_t bsp_apic_id,          //
                                const bool force_v1_only,           //
                                SmpTopology* const out_topology) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidXapicId);
  DCHECK(out_topology != nullptr);
  uintptr_t rsdt_addr = 0;
  uintptr_t xsdt_addr = 0;
  if (force_v1_only) {
    if (!ValidateRsdpV1Bounded(rsdp_addr,          //
                               rsdp_max_end_addr,  //
                               max_physical_addr,  //
                               &rsdt_addr)) {
      return false;
    }
  } else if (!ParseRsdpBounded(rsdp_addr,          //
                               rsdp_max_end_addr,  //
                               max_physical_addr,  //
                               &rsdt_addr,         //
                               &xsdt_addr)) {
    if (!ValidateRsdpV1Bounded(rsdp_addr,          //
                               rsdp_max_end_addr,  //
                               max_physical_addr,  //
                               &rsdt_addr)) {
      return false;
    }
  }

  const uintptr_t madt_addr =
      SmpFindMadtInSdt(rsdt_addr, xsdt_addr, max_physical_addr);
  if (madt_addr != 0 &&
      SmpParseMadt(madt_addr, max_physical_addr, bsp_apic_id, out_topology)) {
    return true;
  }

  if (xsdt_addr != 0 && rsdt_addr != 0) {
    const uintptr_t rsdt_madt_addr =
        SmpFindMadtInSdt(rsdt_addr, 0, max_physical_addr);
    if (rsdt_madt_addr != 0 && rsdt_madt_addr != madt_addr) {
      return SmpParseMadt(rsdt_madt_addr,     //
                          max_physical_addr,  //
                          bsp_apic_id,        //
                          out_topology);
    }
  }
  return false;
}

static bool TryDiscoverFromMemoryRange(const uintptr_t range_start,        //
                                       const int64_t range_length,         //
                                       const uintptr_t max_physical_addr,  //
                                       const uint8_t bsp_apic_id,          //
                                       SmpTopology* const out_topology) {
  DCHECK(range_start != 0);
  DCHECK(range_length > 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidXapicId);
  DCHECK(out_topology != nullptr);
  if (range_start >= max_physical_addr ||
      (max_physical_addr - range_start) <
          static_cast<uintptr_t>(range_length)) {
    return false;
  }
  const uintptr_t range_end = range_start + range_length;
  uintptr_t cursor = AlignUp(range_start, kRsdpScanAlignment);
  while (cursor < range_end) {
    const uintptr_t rsdp_addr =
        SmpFindRsdpInMemoryRange(cursor, range_end - cursor);
    if (rsdp_addr == 0) {
      break;
    }
    if (TryDiscoverFromRsdp(rsdp_addr,          //
                            range_end,          //
                            max_physical_addr,  //
                            bsp_apic_id,        //
                            false,              //
                            out_topology)) {
      return true;
    }
    cursor = rsdp_addr + kRsdpScanAlignment;
  }
  return false;
}

static void InitCpuLocalSlot(const int cpu_index,         //
                             const uint8_t apic_id,       //
                             const bool online,           //
                             const uintptr_t stack_base,  //
                             const uintptr_t stack_top) {
  DCHECK(g_cpu_locals != nullptr);
  DCHECK(cpu_index >= 0 && cpu_index < g_topology.cpu_count);
  CpuLocal& slot = g_cpu_locals[cpu_index];
  slot.self = &slot;
  slot.cpu_id = cpu_index;
  slot.apic_id = apic_id;
  slot.online = online;
  slot.stack_base = stack_base;
  slot.stack_top = stack_top;
  slot.top_held_lock = nullptr;
  slot.timer_ticks.store(0, std::memory_order_relaxed);
  slot.ipi_count.store(0, std::memory_order_relaxed);
}

static void ResetSmpState() {
#if __STDC_HOSTED__
  g_host_ap_runner.Reset();
  ResetCpuLocalForTest();
  g_host_eoi_count.store(0, std::memory_order_relaxed);
#else
  internal::g_cpu_local_bound.store(false, std::memory_order_release);
#endif
  g_topology = {};
  g_cpu_locals.reset();
  g_calibrated_timer_initial_count = 0;
  g_online_cpu_count.store(0, std::memory_order_seq_cst);
  g_ap_boot_done.store(0, std::memory_order_seq_cst);
  g_ap_boot_started.store(0, std::memory_order_seq_cst);
  g_ap_boot_stack = 0;
  g_ap_boot_cpu_index = 0;
  g_smp_initialized = false;
  g_dispatch_in_progress.store(false, std::memory_order_seq_cst);
  g_work_fn.store(nullptr, std::memory_order_seq_cst);
  g_work_context.store(nullptr, std::memory_order_seq_cst);
  g_work_epoch.store(0, std::memory_order_seq_cst);
  g_work_start_epoch.store(0, std::memory_order_seq_cst);
  g_ap_ack_epoch.reset();
  g_ap_done_epoch.reset();
}

#if !__STDC_HOSTED__
static inline void Outb(const uint16_t port, const uint8_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

static inline uint8_t Inb(const uint16_t port) {
  uint8_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

static uint64_t ReadMsr(const uint32_t msr) {
  uint32_t low = 0;
  uint32_t high = 0;
  asm volatile("rdmsr" : "=a"(low), "=d"(high) : "c"(msr));
  return (static_cast<uint64_t>(high) << 32) | low;
}

static void WriteMsr(const uint32_t msr, const uint64_t value) {
  const uint32_t low = static_cast<uint32_t>(value & 0xFFFFFFFFu);
  const uint32_t high = static_cast<uint32_t>(value >> 32);
  asm volatile("wrmsr" : : "c"(msr), "a"(low), "d"(high) : "memory");
}

static uintptr_t ReadRsp() {
  uint64_t rsp = 0;
  asm volatile("mov %0, rsp" : "=r"(rsp));
  return rsp;
}

static uintptr_t ReadCr3() {
  uint64_t cr3 = 0;
  asm volatile("mov %0, cr3" : "=r"(cr3));
  return cr3;
}

static void MarkMmioPageUncacheable(const uintptr_t phys_addr) {
  DCHECK(phys_addr != 0);
  DCHECK(phys_addr < kMaxCanonicalIdentityAddress);
  CHECK(PagingMarkPageUncacheable(phys_addr));
}

static uint16_t ReadCs() {
  uint16_t cs = 0;
  asm volatile("mov %0, cs" : "=r"(cs));
  return cs;
}

static bool IsHardwareLongModeActive() {
  const uint64_t efer = ReadMsr(kMsrIa32Efer);
  return (efer & kEferLongModeActive) != 0 && ReadCs() == kKernelCodeSelector;
}

static uint8_t ReadCpuidInitialApicId() {
  uint32_t eax = 1;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
  asm volatile("cpuid" : "+a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx) : : "cc");
  return static_cast<uint8_t>((ebx >> 24) & 0xFFu);
}

static uint32_t LapicRead(const uintptr_t lapic_base, const int reg_offset) {
  DCHECK(lapic_base != 0);
  DCHECK(reg_offset >= 0);
  const volatile uint32_t* const reg =
      reinterpret_cast<const volatile uint32_t*>(lapic_base + reg_offset);
  return *reg;
}

static void LapicWrite(const uintptr_t lapic_base,  //
                       const int reg_offset,        //
                       const uint32_t value) {
  DCHECK(lapic_base != 0);
  DCHECK(reg_offset >= 0);
  volatile uint32_t* const reg =
      reinterpret_cast<volatile uint32_t*>(lapic_base + reg_offset);
  *reg = value;
  (void)LapicRead(lapic_base, kLapicRegId);
}

static void EnableLocalApic(const uintptr_t lapic_base) {
  DCHECK(lapic_base != 0);
  const uint64_t apic_base_msr = ReadMsr(kMsrIa32ApicBase);
  if ((apic_base_msr & kApicBaseGlobalEnable) == 0) {
    WriteMsr(kMsrIa32ApicBase, apic_base_msr | kApicBaseGlobalEnable);
  }
  LapicWrite(lapic_base, kLapicRegTpr, 0);
  const uint32_t svr = LapicRead(lapic_base, kLapicRegSvr);
  LapicWrite(lapic_base,    //
             kLapicRegSvr,  //
             svr | kLapicSvrSoftwareEnable | kLapicSpuriousVector);
}

static uint8_t ReadLapicId(const uintptr_t lapic_base) {
  DCHECK(lapic_base != 0);
  return static_cast<uint8_t>((LapicRead(lapic_base, kLapicRegId) >> 24) &
                              0xFFu);
}

static bool WaitForIcrIdle(const uintptr_t lapic_base) {
  DCHECK(lapic_base != 0);
  for (int i = 0; i < kIcrIdlePollLimit; ++i) {
    if ((LapicRead(lapic_base, kLapicRegIcrLow) & kIcrDeliveryPending) == 0) {
      return true;
    }
    asm volatile("pause" : : : "memory");
  }
  return (LapicRead(lapic_base, kLapicRegIcrLow) & kIcrDeliveryPending) == 0;
}

static uint8_t ReadPitSpeakerPort() { return Inb(kPitSpeakerPort); }

static uint32_t ReadLapicTimerCurrentCount(const uintptr_t lapic_base) {
  return LapicRead(lapic_base, kLapicRegTimerCurrCount);
}

static uint32_t CalibrateLocalApicTimer(const uintptr_t lapic_base) {
  DCHECK(lapic_base != 0);
  LapicWrite(lapic_base, kLapicRegTimerDivConfig, kLapicTimerDivBy16);
  LapicWrite(lapic_base,         //
             kLapicRegLvtTimer,  //
             kLapicTimerMaskBit | static_cast<uint32_t>(kVectorApicTimer));

  const uint8_t speaker_initial = Inb(kPitSpeakerPort);
  const uint8_t gate_disabled =
      static_cast<uint8_t>(speaker_initial & ~kPitSpeakerEnableMask);
  Outb(kPitSpeakerPort, gate_disabled);

  Outb(kPitCommandPort, kPitCmdChannel2Mode0);
  Outb(kPitChannel2DataPort,
       static_cast<uint8_t>(kPitCalibrationReloadCount & 0xFF));
  Outb(kPitChannel2DataPort,
       static_cast<uint8_t>((kPitCalibrationReloadCount >> 8) & 0xFF));

  // Rising edge on GATE2 starts the 8254 Channel 2 Mode 0 one-shot countdown.
  Outb(kPitSpeakerPort,
       static_cast<uint8_t>(gate_disabled | kPitSpeakerGate2Bit));
  LapicWrite(lapic_base, kLapicRegTimerInitCount, 0xFFFFFFFFu);

  const uint32_t initial_count =
      CalibrateApicTimerCount(lapic_base,                //
                              kPitCalibrationPollLimit,  //
                              ReadPitSpeakerPort,        //
                              ReadLapicTimerCurrentCount);

  LapicWrite(lapic_base, kLapicRegTimerInitCount, 0);
  Outb(kPitSpeakerPort, gate_disabled);
  CHECK(initial_count > 0);
  return initial_count;
}

static void StartLocalApicTimer(const uintptr_t lapic_base,
                                const uint32_t initial_count) {
  DCHECK(lapic_base != 0);
  DCHECK(initial_count > 0);
  LapicWrite(lapic_base, kLapicRegTimerDivConfig, kLapicTimerDivBy16);
  LapicWrite(lapic_base,         //
             kLapicRegLvtTimer,  //
             kLapicTimerPeriodicMode | static_cast<uint32_t>(kVectorApicTimer));
  LapicWrite(lapic_base, kLapicRegTimerInitCount, initial_count);
}

static void SendFixedIpi(const uintptr_t lapic_base,  //
                         const uint8_t apic_id,       //
                         const uint8_t vector) {
  DCHECK(lapic_base != 0);
  DCHECK(apic_id != kInvalidXapicId);
  DCHECK(vector >= static_cast<uint8_t>(kCpuExceptionCount));
  const uint32_t icr_dest = static_cast<uint32_t>(apic_id) << 24;
  LapicWrite(lapic_base, kLapicRegEsr, 0);
  (void)LapicRead(lapic_base, kLapicRegEsr);
  LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
  LapicWrite(
      lapic_base,       //
      kLapicRegIcrLow,  //
      kIcrDeliveryFixed | kIcrLevelAssert | static_cast<uint32_t>(vector));
  CHECK(WaitForIcrIdle(lapic_base));
}

static void DelayLoop(const int iterations) {
  DCHECK(iterations >= 0);
  for (int i = 0; i < iterations; ++i) {
    asm volatile("pause" : : : "memory");
  }
}

static void InstallApTrampoline(uint8_t* const saved_bytes) {
  DCHECK(saved_bytes != nullptr);
  const uintptr_t start_addr = reinterpret_cast<uintptr_t>(ap_trampoline_start);
  const uintptr_t end_addr = reinterpret_cast<uintptr_t>(ap_trampoline_end);
  const int64_t trampoline_bytes = end_addr - start_addr;
  DCHECK(trampoline_bytes > 0 && trampoline_bytes <= kMaxTrampolineBytes);
  uint8_t* const dst = reinterpret_cast<uint8_t*>(kApTrampolinePhysAddr);
  const uint8_t* const src = ap_trampoline_start;
  for (int64_t i = 0; i < trampoline_bytes; ++i) {
    saved_bytes[i] = dst[i];
    dst[i] = src[i];
  }
  std::atomic_thread_fence(std::memory_order_seq_cst);
}

static void RestoreApTrampoline(const uint8_t* const saved_bytes) {
  DCHECK(saved_bytes != nullptr);
  const uintptr_t start_addr = reinterpret_cast<uintptr_t>(ap_trampoline_start);
  const uintptr_t end_addr = reinterpret_cast<uintptr_t>(ap_trampoline_end);
  const int64_t trampoline_bytes = end_addr - start_addr;
  DCHECK(trampoline_bytes > 0 && trampoline_bytes <= kMaxTrampolineBytes);
  uint8_t* const dst = reinterpret_cast<uint8_t*>(kApTrampolinePhysAddr);
  for (int64_t i = 0; i < trampoline_bytes; ++i) {
    dst[i] = saved_bytes[i];
  }
  std::atomic_thread_fence(std::memory_order_seq_cst);
}

static void WakeApplicationProcessor(const uintptr_t lapic_base,  //
                                     const int cpu_index,         //
                                     const uint8_t apic_id,       //
                                     const uintptr_t stack_top_addr) {
  DCHECK(lapic_base != 0);
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  DCHECK(apic_id != kInvalidXapicId);
  DCHECK(stack_top_addr != 0 && (stack_top_addr & 0xF) == 0);
  g_ap_boot_stack = stack_top_addr;
  g_ap_boot_cpu_index = cpu_index;
  g_ap_boot_started.store(0, std::memory_order_seq_cst);
  g_ap_boot_done.store(0, std::memory_order_seq_cst);
  const uint32_t icr_dest = static_cast<uint32_t>(apic_id) << 24;

  // Send INIT IPI (assert, then de-assert).
  LapicWrite(lapic_base, kLapicRegEsr, 0);
  (void)LapicRead(lapic_base, kLapicRegEsr);
  LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
  LapicWrite(lapic_base,       //
             kLapicRegIcrLow,  //
             kIcrDeliveryInit | kIcrLevelAssert | kIcrTriggerLevel);
  CHECK(WaitForIcrIdle(lapic_base));

  LapicWrite(lapic_base, kLapicRegEsr, 0);
  (void)LapicRead(lapic_base, kLapicRegEsr);
  LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
  LapicWrite(lapic_base, kLapicRegIcrLow, kIcrDeliveryInit | kIcrTriggerLevel);
  CHECK(WaitForIcrIdle(lapic_base));

  DelayLoop(kInitDelayIterations);

  // Send first Startup IPI (SIPI) targeting vector 0x08 (0x8000).
  LapicWrite(lapic_base, kLapicRegEsr, 0);
  (void)LapicRead(lapic_base, kLapicRegEsr);
  LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
  LapicWrite(lapic_base,       //
             kLapicRegIcrLow,  //
             kIcrDeliveryStartup | kApTrampolineVector);
  CHECK(WaitForIcrIdle(lapic_base));

  for (int i = 0; i < kFirstSipiPollLimit; ++i) {
    if (g_ap_boot_done.load(std::memory_order_acquire) != 0) {
      return;
    }
    if (g_ap_boot_started.load(std::memory_order_acquire) != 0) {
      break;
    }
    asm volatile("pause" : : : "memory");
  }

  // Send second Startup IPI (SIPI) only if the AP has not yet started
  // executing the trampoline on the first SIPI.
  if (g_ap_boot_done.load(std::memory_order_acquire) == 0 &&
      g_ap_boot_started.load(std::memory_order_acquire) == 0) {
    LapicWrite(lapic_base, kLapicRegEsr, 0);
    (void)LapicRead(lapic_base, kLapicRegEsr);
    LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
    LapicWrite(lapic_base,       //
               kLapicRegIcrLow,  //
               kIcrDeliveryStartup | kApTrampolineVector);
    CHECK(WaitForIcrIdle(lapic_base));
  }

  for (int i = 0; i < kSecondSipiPollLimit; ++i) {
    if (g_ap_boot_done.load(std::memory_order_acquire) != 0) {
      return;
    }
    asm volatile("pause" : : : "memory");
  }
  CHECK(g_ap_boot_done.load(std::memory_order_acquire) != 0);
}

#endif

static bool RunPendingApWork(const int cpu_index,
                             int64_t* const completed_epoch) {
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  DCHECK(completed_epoch != nullptr);
  const int64_t epoch = g_work_epoch.load(std::memory_order_acquire);
  if (epoch <= *completed_epoch) {
    return false;
  }
#if !__STDC_HOSTED__
  asm volatile("sti" : : : "memory", "cc");
#endif
  g_ap_ack_epoch[cpu_index].store(epoch, std::memory_order_release);
  while (g_work_start_epoch.load(std::memory_order_acquire) != epoch) {
    asm volatile("pause" : : : "memory");
  }
  const SmpWorkFn work_fn = g_work_fn.load(std::memory_order_acquire);
  void* const context = g_work_context.load(std::memory_order_acquire);
  DCHECK(work_fn != nullptr);
  work_fn(cpu_index, context);
  *completed_epoch = epoch;
  g_ap_done_epoch[cpu_index].store(epoch, std::memory_order_release);
  return true;
}

static void SendWakeupIpi(const uintptr_t lapic_base,  //
                          const int cpu_index,         //
                          const uint8_t apic_id) {
  DCHECK(lapic_base != 0);
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  DCHECK(apic_id != kInvalidXapicId);
#if !__STDC_HOSTED__
  (void)cpu_index;
  SendFixedIpi(lapic_base, apic_id, kVectorWakeupIpi);
#else
  (void)lapic_base;
  (void)apic_id;
  g_host_ap_runner.EnsureCapacity(g_topology.cpu_count);
  const int64_t target_epoch = g_work_epoch.load(std::memory_order_acquire);
  if (g_host_ap_runner.spawned_epoch[cpu_index] == target_epoch) {
    return;
  }
  if (g_host_ap_runner.threads[cpu_index].joinable()) {
    g_host_ap_runner.threads[cpu_index].join();
  }
  g_host_ap_runner.spawned_epoch[cpu_index] = target_epoch;
  g_host_ap_runner.threads[cpu_index] = std::thread([cpu_index]() {
    BindCpuLocal(&g_cpu_locals[cpu_index]);
    SetInterruptsEnabledForTest(true);
    InterruptFrame ipi_frame = {};
    ipi_frame.vector = kVectorWakeupIpi;
    IdtDispatch(&ipi_frame);
    InterruptFrame timer_frame = {};
    timer_frame.vector = kVectorApicTimer;
    IdtDispatch(&timer_frame);
    while (!RunPendingApWork(cpu_index,
                             &g_host_ap_runner.completed_epoch[cpu_index])) {
      asm volatile("pause" : : : "memory");
    }
  });
#endif
}

}  // namespace

extern "C" void ApKernelEntry(const int cpu_index) {
#if !__STDC_HOSTED__
  g_ap_boot_started.store(1, std::memory_order_release);
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  DCHECK(lapic_base != 0);
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);

  IdtLoad();
  g_cpu_locals[cpu_index].online = true;
  BindCpuLocal(&g_cpu_locals[cpu_index]);
  EnableLocalApic(lapic_base);

  CpuInfo& cpu = g_topology.cpus[cpu_index];
  cpu.observed_apic_id = ReadLapicId(lapic_base);
  cpu.observed_rsp = ReadRsp();
  cpu.observed_cr3 = ReadCr3();
  cpu.long_mode_active = IsHardwareLongModeActive();
  cpu.online = true;

  StartLocalApicTimer(lapic_base, g_calibrated_timer_initial_count);

  g_online_cpu_count.fetch_add(1, std::memory_order_seq_cst);
  g_ap_boot_done.store(1, std::memory_order_release);

  int64_t completed_epoch = 0;
  for (;;) {
    asm volatile("cli" : : : "memory", "cc");
    if (RunPendingApWork(cpu_index, &completed_epoch)) {
      continue;
    }
    const std::shared_ptr<TaskScheduler>& scheduler = GetTaskScheduler();
    if (scheduler != nullptr && scheduler->PollIdleCpu(cpu_index)) {
      continue;
    }
    asm volatile("sti; hlt" : : : "memory", "cc");
  }
#else
  (void)cpu_index;
#endif
}

uintptr_t SmpFindRsdpInMultiboot2(const uintptr_t multiboot_info_addr,
                                  const uintptr_t max_physical_addr) {
  DCHECK(multiboot_info_addr != 0);
  DCHECK(max_physical_addr > 0);
  uintptr_t rsdp_new_addr = 0;
  uintptr_t rsdp_new_end_addr = 0;
  uintptr_t rsdp_old_addr = 0;
  uintptr_t rsdp_old_end_addr = 0;
  if (!FindRsdpTagsInMultiboot2(multiboot_info_addr,  //
                                max_physical_addr,    //
                                &rsdp_new_addr,       //
                                &rsdp_new_end_addr,   //
                                &rsdp_old_addr,       //
                                &rsdp_old_end_addr)) {
    return 0;
  }
  return (rsdp_new_addr != 0) ? rsdp_new_addr : rsdp_old_addr;
}

uintptr_t SmpFindRsdpInMemoryRange(const uintptr_t range_start,
                                   const int64_t range_length) {
  DCHECK(range_start != 0);
  DCHECK(range_length > 0);
  constexpr int64_t kV1Size = sizeof(AcpiRsdpV1);
  if (range_length < kV1Size || range_start >= kMaxCanonicalIdentityAddress ||
      (kMaxCanonicalIdentityAddress - range_start) <
          static_cast<uintptr_t>(range_length) ||
      !PagingMapBootstrapRange(range_start, range_length)) {
    return 0;
  }

  const uintptr_t aligned_start = AlignUp(range_start, kRsdpScanAlignment);
  const uintptr_t range_end = range_start + range_length;
  for (uintptr_t addr = aligned_start; addr + kV1Size <= range_end;
       addr += kRsdpScanAlignment) {
    uintptr_t rsdt = 0;
    uintptr_t xsdt = 0;
    if (ParseRsdpBounded(addr,                          //
                         range_end,                     //
                         kMaxCanonicalIdentityAddress,  //
                         &rsdt,                         //
                         &xsdt)) {
      return addr;
    }
  }
  return 0;
}

bool SmpParseRsdp(const uintptr_t rsdp_addr,          //
                  const uintptr_t max_physical_addr,  //
                  uintptr_t* const out_rsdt_addr,     //
                  uintptr_t* const out_xsdt_addr) {
  DCHECK(rsdp_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(out_rsdt_addr != nullptr);
  DCHECK(out_xsdt_addr != nullptr);
  return ParseRsdpBounded(rsdp_addr,          //
                          max_physical_addr,  //
                          max_physical_addr,  //
                          out_rsdt_addr,      //
                          out_xsdt_addr);
}

uintptr_t SmpFindMadtInSdt(const uintptr_t rsdt_addr,  //
                           const uintptr_t xsdt_addr,  //
                           const uintptr_t max_physical_addr) {
  DCHECK(rsdt_addr != 0 || xsdt_addr != 0);
  DCHECK(max_physical_addr > 0);
  constexpr int64_t kHeaderSize = sizeof(AcpiSdtHeader);

  if (xsdt_addr != 0) {
    const AcpiSdtHeader* const xsdt =
        MapAndValidateSdt(xsdt_addr, max_physical_addr, "XSDT");
    if (xsdt != nullptr) {
      const int64_t payload_bytes = xsdt->length - kHeaderSize;
      const int entry_count = payload_bytes / sizeof(uint64_t);
      const uint8_t* const entries_base =
          reinterpret_cast<const uint8_t*>(xsdt) + kHeaderSize;
      for (int i = 0; i < entry_count; ++i) {
        uint64_t entry_phys = 0;
        const uint8_t* const src = entries_base + i * sizeof(uint64_t);
        for (int b = 0; b < static_cast<int>(sizeof(uint64_t)); ++b) {
          entry_phys |= static_cast<uint64_t>(src[b]) << (b * 8);
        }
        const AcpiSdtHeader* const candidate =
            MapAndValidateSdt(entry_phys, max_physical_addr, "APIC");
        if (candidate != nullptr && candidate->length >= sizeof(AcpiMadt)) {
          return entry_phys;
        }
      }
    }
  }

  if (rsdt_addr != 0) {
    const AcpiSdtHeader* const rsdt =
        MapAndValidateSdt(rsdt_addr, max_physical_addr, "RSDT");
    if (rsdt != nullptr) {
      const int64_t payload_bytes = rsdt->length - kHeaderSize;
      const int entry_count = payload_bytes / sizeof(uint32_t);
      const uint8_t* const entries_base =
          reinterpret_cast<const uint8_t*>(rsdt) + kHeaderSize;
      for (int i = 0; i < entry_count; ++i) {
        uint32_t raw_entry = 0;
        const uint8_t* const src = entries_base + i * sizeof(uint32_t);
        for (int b = 0; b < static_cast<int>(sizeof(uint32_t)); ++b) {
          raw_entry |= static_cast<uint32_t>(src[b]) << (b * 8);
        }
        const uintptr_t entry_phys = ResolveAcpi32Address(raw_entry);
        const AcpiSdtHeader* const candidate =
            MapAndValidateSdt(entry_phys, max_physical_addr, "APIC");
        if (candidate != nullptr && candidate->length >= sizeof(AcpiMadt)) {
          return entry_phys;
        }
      }
    }
  }

  return 0;
}

bool SmpParseMadt(const uintptr_t madt_addr,          //
                  const uintptr_t max_physical_addr,  //
                  const uint8_t bsp_apic_id,          //
                  SmpTopology* const out_topology) {
  DCHECK(madt_addr != 0);
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidXapicId);
  DCHECK(out_topology != nullptr);
  *out_topology = {};

  const AcpiSdtHeader* const header =
      MapAndValidateSdt(madt_addr, max_physical_addr, "APIC");
  constexpr int64_t kMadtHeaderSize = sizeof(AcpiMadt);
  if (header == nullptr || header->length < kMadtHeaderSize) {
    return false;
  }

  const AcpiMadt* const madt = reinterpret_cast<const AcpiMadt*>(header);
  const int64_t madt_length = madt->header.length;
  out_topology->local_apic_phys_addr =
      IsValidLocalApicAddress(madt->local_apic_address, max_physical_addr)
          ? static_cast<uintptr_t>(madt->local_apic_address)
          : kDefaultLocalApicPhysAddr;
  out_topology->bsp_apic_id = bsp_apic_id;

  int candidate_count = 0;
  int64_t offset = kMadtHeaderSize;
  constexpr int64_t kEntryHeaderSize = sizeof(AcpiMadtEntryHeader);
  while (offset + kEntryHeaderSize <= madt_length) {
    const AcpiMadtEntryHeader* const entry =
        reinterpret_cast<const AcpiMadtEntryHeader*>(madt_addr + offset);
    const int64_t entry_len = entry->length;
    if (entry_len < kEntryHeaderSize || offset + entry_len > madt_length) {
      break;
    }

    if (entry->type == kMadtTypeLocalApic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApic))) {
        break;
      }
      const AcpiMadtLocalApic* const lapic =
          reinterpret_cast<const AcpiMadtLocalApic*>(entry);
      if ((lapic->flags & kMadtLapicUsableMask) != 0 &&
          lapic->apic_id != kInvalidXapicId) {
        ++candidate_count;
      }
    } else if (entry->type == kMadtTypeLocalApicOverride) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApicOverride))) {
        break;
      }
      const AcpiMadtLocalApicOverride* const override_entry =
          reinterpret_cast<const AcpiMadtLocalApicOverride*>(entry);
      if (IsValidLocalApicAddress(override_entry->local_apic_address,
                                  max_physical_addr)) {
        out_topology->local_apic_phys_addr = override_entry->local_apic_address;
      }
    } else if (entry->type == kMadtTypeLocalX2Apic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalX2Apic))) {
        break;
      }
      const AcpiMadtLocalX2Apic* const x2apic =
          reinterpret_cast<const AcpiMadtLocalX2Apic*>(entry);
      if ((x2apic->flags & kMadtLapicUsableMask) != 0 &&
          x2apic->x2apic_id < kInvalidXapicId) {
        ++candidate_count;
      }
    }

    offset += entry_len;
  }

  if (candidate_count <= 0) {
    *out_topology = {};
    return false;
  }

  const int capacity = candidate_count + 1;
  out_topology->cpus.reset(new CpuInfo[capacity]());
  if (out_topology->cpus == nullptr) {
    *out_topology = {};
    return false;
  }

  offset = kMadtHeaderSize;
  while (offset + kEntryHeaderSize <= madt_length) {
    const AcpiMadtEntryHeader* const entry =
        reinterpret_cast<const AcpiMadtEntryHeader*>(madt_addr + offset);
    const int64_t entry_len = entry->length;
    if (entry_len < kEntryHeaderSize || offset + entry_len > madt_length) {
      break;
    }

    if (entry->type == kMadtTypeLocalApic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApic))) {
        break;
      }
      const AcpiMadtLocalApic* const lapic =
          reinterpret_cast<const AcpiMadtLocalApic*>(entry);
      if ((lapic->flags & kMadtLapicUsableMask) != 0 &&
          lapic->apic_id != kInvalidXapicId) {
        AddDiscoveredCpu(out_topology,    //
                         capacity,        //
                         lapic->apic_id,  //
                         lapic->acpi_processor_id);
      }
    } else if (entry->type == kMadtTypeLocalApicOverride) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalApicOverride))) {
        break;
      }
    } else if (entry->type == kMadtTypeLocalX2Apic) {
      if (entry_len < static_cast<int64_t>(sizeof(AcpiMadtLocalX2Apic))) {
        break;
      }
      const AcpiMadtLocalX2Apic* const x2apic =
          reinterpret_cast<const AcpiMadtLocalX2Apic*>(entry);
      if ((x2apic->flags & kMadtLapicUsableMask) != 0 &&
          x2apic->x2apic_id < kInvalidXapicId) {
        AddDiscoveredCpu(out_topology,                             //
                         capacity,                                 //
                         static_cast<uint8_t>(x2apic->x2apic_id),  //
                         static_cast<uint8_t>(x2apic->acpi_processor_uid));
      }
    }

    offset += entry_len;
  }

  if (out_topology->cpu_count <= 0) {
    *out_topology = {};
    return false;
  }

  int bsp_index = -1;
  for (int i = 0; i < out_topology->cpu_count; ++i) {
    if (out_topology->cpus[i].apic_id == bsp_apic_id) {
      bsp_index = i;
      break;
    }
  }

  if (bsp_index > 0) {
    const CpuInfo bsp_info = out_topology->cpus[bsp_index];
    for (int i = bsp_index; i > 0; --i) {
      out_topology->cpus[i] = out_topology->cpus[i - 1];
    }
    out_topology->cpus[0] = bsp_info;
  } else if (bsp_index < 0) {
    DCHECK(out_topology->cpu_count < capacity);
    for (int i = out_topology->cpu_count; i > 0; --i) {
      out_topology->cpus[i] = out_topology->cpus[i - 1];
    }
    ++out_topology->cpu_count;
    out_topology->cpus[0] = {};
    out_topology->cpus[0].apic_id = bsp_apic_id;
    out_topology->cpus[0].acpi_processor_id = 0;
  }

  out_topology->cpus[0].is_bsp = true;
  out_topology->cpus[0].online = true;
  out_topology->cpus[0].observed_apic_id = bsp_apic_id;
  return true;
}

bool SmpDiscoverTopology(const uint32_t multiboot_magic,      //
                         const uint64_t multiboot_info_addr,  //
                         const uintptr_t ebda_base,           //
                         const uintptr_t bios_rom_base,       //
                         const uintptr_t max_physical_addr,   //
                         const uint8_t bsp_apic_id,           //
                         SmpTopology* const out_topology) {
  DCHECK(max_physical_addr > 0);
  DCHECK(bsp_apic_id != kInvalidXapicId);
  DCHECK(out_topology != nullptr);
  *out_topology = {};

  if (multiboot_magic == kMultiboot2Magic && multiboot_info_addr != 0) {
    uintptr_t rsdp_new_addr = 0;
    uintptr_t rsdp_new_end_addr = 0;
    uintptr_t rsdp_old_addr = 0;
    uintptr_t rsdp_old_end_addr = 0;
    if (FindRsdpTagsInMultiboot2(multiboot_info_addr,  //
                                 max_physical_addr,    //
                                 &rsdp_new_addr,       //
                                 &rsdp_new_end_addr,   //
                                 &rsdp_old_addr,       //
                                 &rsdp_old_end_addr)) {
      if (rsdp_new_addr != 0 && TryDiscoverFromRsdp(rsdp_new_addr,      //
                                                    rsdp_new_end_addr,  //
                                                    max_physical_addr,  //
                                                    bsp_apic_id,        //
                                                    false,              //
                                                    out_topology)) {
        return true;
      }
      if (rsdp_old_addr != 0 && TryDiscoverFromRsdp(rsdp_old_addr,      //
                                                    rsdp_old_end_addr,  //
                                                    max_physical_addr,  //
                                                    bsp_apic_id,        //
                                                    true,               //
                                                    out_topology)) {
        return true;
      }
    }
  }

  if (ebda_base != 0) {
    int64_t ebda_scan_len = kEbdaScanBytes;
    if (ebda_base >= kEbdaMinAddress && ebda_base < kEbdaMaxAddress) {
      const int64_t max_ebda_bytes = kEbdaMaxAddress - ebda_base;
      if (ebda_scan_len > max_ebda_bytes) {
        ebda_scan_len = max_ebda_bytes;
      }
    }
    if (TryDiscoverFromMemoryRange(ebda_base,          //
                                   ebda_scan_len,      //
                                   max_physical_addr,  //
                                   bsp_apic_id,        //
                                   out_topology)) {
      return true;
    }
  }

  if (bios_rom_base != 0) {
    if (TryDiscoverFromMemoryRange(bios_rom_base,      //
                                   kBiosRomScanBytes,  //
                                   max_physical_addr,  //
                                   bsp_apic_id,        //
                                   out_topology)) {
      return true;
    }
  }

  return false;
}

uint32_t ComputeApicTimerInitialCount(const uint64_t elapsed_apic_ticks,  //
                                      const int pit_reload_count,         //
                                      const int target_hz) {
  DCHECK(pit_reload_count > 0 && pit_reload_count <= kMaxPitReloadCount);
  DCHECK(target_hz > 0 && target_hz <= kPitBaseFrequencyHz);
  if (elapsed_apic_ticks == 0) {
    return 0;
  }
  const uint64_t freq_hz = static_cast<uint64_t>(kPitBaseFrequencyHz);
  if (elapsed_apic_ticks > UINT64_MAX / freq_hz) {
    return 0xFFFFFFFFu;
  }
  const uint64_t numerator = elapsed_apic_ticks * freq_hz;
  const uint64_t denominator = static_cast<uint64_t>(pit_reload_count) *
                               static_cast<uint64_t>(target_hz);
  const uint64_t ticks_per_period = numerator / denominator;
  if (ticks_per_period > 0xFFFFFFFFULL) {
    return 0xFFFFFFFFu;
  }
  return static_cast<uint32_t>(ticks_per_period);
}

uint32_t CalibrateApicTimerCount(
    const uintptr_t lapic_base,               //
    const int max_polls,                      //
    const PitSpeakerReadFn read_pit_speaker,  //
    const ApicTimerCurrentCountReadFn read_apic_current_count) {
  DCHECK(max_polls > 0);
  DCHECK(read_pit_speaker != nullptr);
  DCHECK(read_apic_current_count != nullptr);

  bool terminal_count_reached = false;
  for (int poll = 0; poll < max_polls; ++poll) {
    if ((read_pit_speaker() & kPitSpeakerOut2Bit) != 0) {
      terminal_count_reached = true;
      break;
    }
    asm volatile("pause" : : : "memory");
  }

  const uint32_t current_count = read_apic_current_count(lapic_base);
  if (!terminal_count_reached) {
    return kFallbackApicTimerInitialCount;
  }

  const uint64_t elapsed_ticks = 0xFFFFFFFFULL - current_count;
  const uint32_t initial_count =
      ComputeApicTimerInitialCount(elapsed_ticks,               //
                                   kPitCalibrationReloadCount,  //
                                   kApicTimerTargetHz);
  return (initial_count > 0) ? initial_count : kFallbackApicTimerInitialCount;
}

void SmpInit(const uint32_t multiboot_magic,
             const uint64_t multiboot_info_addr) {
  ResetSmpState();
  if (!IdtIsInitialized()) {
    IdtInit();
  }

#if !__STDC_HOSTED__
  const uint8_t bsp_initial_apic_id = ReadCpuidInitialApicId();
  uintptr_t bda_ebda_addr = 0x040E;
  asm volatile("" : "+r"(bda_ebda_addr));
  const uint16_t ebda_segment =
      *reinterpret_cast<const volatile uint16_t*>(bda_ebda_addr);
  const uintptr_t raw_ebda_base = static_cast<uintptr_t>(ebda_segment) << 4;
  const uintptr_t ebda_base =
      (raw_ebda_base >= kEbdaMinAddress && raw_ebda_base < kEbdaMaxAddress)
          ? raw_ebda_base
          : 0;
  const uintptr_t bios_rom_base = kBiosRomSearchBase;
#else
  const uint8_t bsp_initial_apic_id = g_host_bsp_apic_id;
  const uintptr_t ebda_base = g_host_ebda_base;
  const uintptr_t bios_rom_base = g_host_bios_rom_base;
#endif

  CHECK(SmpDiscoverTopology(multiboot_magic,               //
                            multiboot_info_addr,           //
                            ebda_base,                     //
                            bios_rom_base,                 //
                            kMaxCanonicalIdentityAddress,  //
                            bsp_initial_apic_id,           //
                            &g_topology));

  const int cpu_count = g_topology.cpu_count;
  CHECK(cpu_count >= 1);
  g_cpu_locals.reset(new CpuLocal[cpu_count]());
  CHECK(g_cpu_locals != nullptr);
  g_ap_ack_epoch.reset(new std::atomic<int64_t>[cpu_count]());
  CHECK(g_ap_ack_epoch != nullptr);
  g_ap_done_epoch.reset(new std::atomic<int64_t>[cpu_count]());
  CHECK(g_ap_done_epoch != nullptr);

  CHECK(PagingMapBootstrapRange(g_topology.local_apic_phys_addr, kPageSize));

#if !__STDC_HOSTED__
  MarkMmioPageUncacheable(g_topology.local_apic_phys_addr);
  EnableLocalApic(g_topology.local_apic_phys_addr);
  g_topology.cpus[0].observed_apic_id =
      ReadLapicId(g_topology.local_apic_phys_addr);
  g_topology.cpus[0].stack_base = reinterpret_cast<uintptr_t>(stack_bottom);
  g_topology.cpus[0].stack_top = reinterpret_cast<uintptr_t>(stack_top);
  g_topology.cpus[0].observed_rsp = ReadRsp();
  g_topology.cpus[0].observed_cr3 = ReadCr3();
  g_topology.cpus[0].long_mode_active = IsHardwareLongModeActive();
#else
  g_topology.cpus[0].observed_apic_id = bsp_initial_apic_id;
  g_topology.cpus[0].stack_base = 0x100000;
  g_topology.cpus[0].stack_top = 0x100000 + kApStackSize;
  g_topology.cpus[0].observed_rsp = 0x100000 + kApStackSize - 64;
  g_topology.cpus[0].observed_cr3 = 0x105000;
  g_topology.cpus[0].long_mode_active = true;
#endif
  g_topology.cpus[0].is_bsp = true;
  g_topology.cpus[0].online = true;
  InitCpuLocalSlot(0,                              //
                   g_topology.cpus[0].apic_id,     //
                   true,                           //
                   g_topology.cpus[0].stack_base,  //
                   g_topology.cpus[0].stack_top);
  BindCpuLocal(&g_cpu_locals[0]);
  g_online_cpu_count.store(1, std::memory_order_seq_cst);

#if !__STDC_HOSTED__
  g_calibrated_timer_initial_count =
      CalibrateLocalApicTimer(g_topology.local_apic_phys_addr);
#else
  g_calibrated_timer_initial_count =
      ComputeApicTimerInitialCount(g_host_calibration_ticks,    //
                                   kPitCalibrationReloadCount,  //
                                   kApicTimerTargetHz);
  CHECK(g_calibrated_timer_initial_count > 0);
#endif

  ConsoleWrite("[SMP] Discovered CPUs: ");
  ConsoleWriteDec(g_topology.cpu_count);
  ConsoleWrite(", BSP APIC ID: ");
  ConsoleWriteDec(g_topology.bsp_apic_id);
  ConsoleWrite(", LAPIC base: ");
  ConsoleWriteHex(g_topology.local_apic_phys_addr);
  ConsoleWrite("\n");

  UartWrite("[SMP] CPU 0 (APIC ID ");
  UartWriteDec(g_topology.cpus[0].apic_id);
  UartWrite(", BSP): online\n");

  if (g_topology.cpu_count > 1) {
    for (int i = 1; i < g_topology.cpu_count; ++i) {
      const uintptr_t stack_phys = PmmAllocFrames(kApStackFrames);
      CHECK(stack_phys != 0);
      g_topology.cpus[i].stack_base = stack_phys;
      g_topology.cpus[i].stack_top = stack_phys + kApStackSize;
      InitCpuLocalSlot(i,                              //
                       g_topology.cpus[i].apic_id,     //
                       false,                          //
                       g_topology.cpus[i].stack_base,  //
                       g_topology.cpus[i].stack_top);
    }

#if !__STDC_HOSTED__
    uint8_t saved_trampoline[kMaxTrampolineBytes] = {};
    InstallApTrampoline(saved_trampoline);
#endif

    for (int i = 1; i < g_topology.cpu_count; ++i) {
#if !__STDC_HOSTED__
      WakeApplicationProcessor(g_topology.local_apic_phys_addr,  //
                               i,                                //
                               g_topology.cpus[i].apic_id,       //
                               g_topology.cpus[i].stack_top);
#else
      g_ap_boot_stack = g_topology.cpus[i].stack_top;
      g_ap_boot_cpu_index = i;
      g_ap_boot_started.store(0, std::memory_order_seq_cst);
      bool woke_ok = false;
      if (g_host_ap_sim_fn != nullptr) {
        woke_ok = g_host_ap_sim_fn(i, &g_topology.cpus[i]);
      } else {
        g_topology.cpus[i].observed_apic_id = g_topology.cpus[i].apic_id;
        g_topology.cpus[i].observed_rsp = g_topology.cpus[i].stack_top - 64;
        g_topology.cpus[i].observed_cr3 = g_topology.cpus[0].observed_cr3;
        g_topology.cpus[i].long_mode_active = true;
        g_topology.cpus[i].online = true;
        woke_ok = true;
      }
      CHECK(woke_ok);
      IdtLoad();
      g_cpu_locals[i].online = true;
      BindCpuLocal(&g_cpu_locals[i]);
      InterruptFrame ap_timer_frame = {};
      ap_timer_frame.vector = kVectorApicTimer;
      IdtDispatch(&ap_timer_frame);
      BindCpuLocal(&g_cpu_locals[0]);
      g_online_cpu_count.fetch_add(1, std::memory_order_seq_cst);
#endif
      CHECK(g_topology.cpus[i].online);
      CHECK(g_cpu_locals[i].online);

      UartWrite("[SMP] CPU ");
      UartWriteDec(i);
      UartWrite(" (APIC ID ");
      UartWriteDec(g_topology.cpus[i].apic_id);
      UartWrite(", AP): online\n");
    }

#if !__STDC_HOSTED__
    RestoreApTrampoline(saved_trampoline);
#endif
  }

#if !__STDC_HOSTED__
  StartLocalApicTimer(g_topology.local_apic_phys_addr,
                      g_calibrated_timer_initial_count);
#else
  BindCpuLocal(&g_cpu_locals[0]);
  InterruptFrame bsp_timer_frame = {};
  bsp_timer_frame.vector = kVectorApicTimer;
  IdtDispatch(&bsp_timer_frame);
#endif

  g_smp_initialized = true;
  CHECK(SmpOnlineCpuCount() == g_topology.cpu_count);

#if !__STDC_HOSTED__
  asm volatile("sti" : : : "memory", "cc");
#else
  SetInterruptsEnabledForTest(true);
#endif

  UartWrite("[SMP] Local APIC timer calibrated: initial_count=");
  UartWriteDec(g_calibrated_timer_initial_count);
  UartWrite(", target_hz=");
  UartWriteDec(kApicTimerTargetHz);
  UartWrite("\n");

  UartWrite("[SMP] Online CPUs: ");
  UartWriteDec(SmpOnlineCpuCount());
  UartWrite("/");
  UartWriteDec(g_topology.cpu_count);
  UartWrite("\n");
}

int SmpCpuCount() {
  DCHECK(g_smp_initialized);
  return g_topology.cpu_count;
}

int SmpOnlineCpuCount() {
  DCHECK(g_smp_initialized);
  return g_online_cpu_count.load(std::memory_order_acquire);
}

const CpuInfo* SmpGetCpuInfo(const int index) {
  DCHECK(g_smp_initialized);
  DCHECK(index >= 0 && index < g_topology.cpu_count);
  return &g_topology.cpus[index];
}

CpuLocal* SmpGetCpuLocal(const int cpu_index) {
  DCHECK(g_smp_initialized);
  DCHECK(cpu_index >= 0 && cpu_index < g_topology.cpu_count);
  return &g_cpu_locals[cpu_index];
}

uintptr_t SmpLocalApicPhysAddr() {
  DCHECK(g_smp_initialized);
  return g_topology.local_apic_phys_addr;
}

uint32_t SmpCalibratedTimerInitialCount() {
  DCHECK(g_smp_initialized);
  return g_calibrated_timer_initial_count;
}

void SmpSendLocalApicEoi() {
#if !__STDC_HOSTED__
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  DCHECK(lapic_base != 0);
  LapicWrite(lapic_base, kLapicRegEoi, 0);
#else
  g_host_eoi_count.fetch_add(1, std::memory_order_relaxed);
#endif
}

void SmpSendIpi(const int target_cpu_index, const uint8_t vector) {
  DCHECK(g_smp_initialized);
  DCHECK(target_cpu_index >= 0 && target_cpu_index < g_topology.cpu_count);
  DCHECK(vector >= static_cast<uint8_t>(kCpuExceptionCount));
#if !__STDC_HOSTED__
  SendFixedIpi(g_topology.local_apic_phys_addr,            //
               g_topology.cpus[target_cpu_index].apic_id,  //
               vector);
#else
  if (vector == kVectorWakeupIpi) {
    g_cpu_locals[target_cpu_index].ipi_count.fetch_add(
        1, std::memory_order_relaxed);
    SmpSendLocalApicEoi();
  } else if (vector == kVectorApicTimer) {
    g_cpu_locals[target_cpu_index].timer_ticks.fetch_add(
        1, std::memory_order_relaxed);
    SmpSendLocalApicEoi();
  } else if (vector != kVectorSpurious) {
    SmpSendLocalApicEoi();
  }
#endif
}

void SmpRunOnAllCpus(const SmpWorkFn work_fn, void* const context) {
  DCHECK(g_smp_initialized);
  DCHECK(work_fn != nullptr);
  const int cpu_count = g_topology.cpu_count;
  DCHECK(cpu_count >= 1);
  DCHECK(SmpOnlineCpuCount() == cpu_count);
  DCHECK(!g_dispatch_in_progress.exchange(true, std::memory_order_acq_rel));

#if __STDC_HOSTED__
  BindCpuLocal(&g_cpu_locals[0]);
  InterruptFrame bsp_timer_frame = {};
  bsp_timer_frame.vector = kVectorApicTimer;
  IdtDispatch(&bsp_timer_frame);
#endif

  if (cpu_count == 1) {
    work_fn(0, context);
    g_dispatch_in_progress.store(false, std::memory_order_release);
    return;
  }

  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  DCHECK(lapic_base != 0);
  g_work_fn.store(work_fn, std::memory_order_release);
  g_work_context.store(context, std::memory_order_release);
  const int64_t epoch =
      g_work_epoch.fetch_add(1, std::memory_order_acq_rel) + 1;

  for (int i = 1; i < cpu_count; ++i) {
    SendWakeupIpi(lapic_base, i, g_topology.cpus[i].apic_id);
  }

  for (int poll = 0;; ++poll) {
    bool all_acked = true;
    for (int i = 1; i < cpu_count; ++i) {
      if (g_ap_ack_epoch[i].load(std::memory_order_acquire) != epoch) {
        all_acked = false;
        if (poll > 0 && (poll % kIpiRetryPollInterval) == 0) {
          SendWakeupIpi(lapic_base, i, g_topology.cpus[i].apic_id);
        }
      }
    }
    if (all_acked) {
      break;
    }
    CHECK(poll < kSecondSipiPollLimit);
    asm volatile("pause" : : : "memory");
  }

  g_work_start_epoch.store(epoch, std::memory_order_release);
  work_fn(0, context);

  for (int i = 1; i < cpu_count; ++i) {
    while (g_ap_done_epoch[i].load(std::memory_order_acquire) != epoch) {
      asm volatile("pause" : : : "memory");
    }
  }

  g_dispatch_in_progress.store(false, std::memory_order_release);
}

#if __STDC_HOSTED__
void SmpSetHostTestHooks(const uint8_t bsp_apic_id,      //
                         const uintptr_t ebda_base,      //
                         const uintptr_t bios_rom_base,  //
                         const uintptr_t acpi32_arena,   //
                         const SmpHostApBootSimFn sim_fn) {
  DCHECK(bsp_apic_id != kInvalidXapicId);
  ResetSmpState();
  g_host_bsp_apic_id = bsp_apic_id;
  g_host_ebda_base = ebda_base;
  g_host_bios_rom_base = bios_rom_base;
  g_host_acpi32_high_bits = acpi32_arena & ~0xFFFFFFFFULL;
  g_host_ap_sim_fn = sim_fn;
  g_host_calibration_ticks = kDefaultHostCalibrationTicks;
}

void SmpSetHostTimerCalibrationTicksForTest(const uint64_t elapsed_apic_ticks) {
  g_host_calibration_ticks = elapsed_apic_ticks;
}

int64_t SmpGetHostEoiCountForTest() {
  return g_host_eoi_count.load(std::memory_order_relaxed);
}

void SmpResetHostEoiCountForTest() {
  g_host_eoi_count.store(0, std::memory_order_relaxed);
}
#endif

}  // namespace protos
