#include "smp.h"

#include <cstdint>

#include "check.h"
#include "paging.h"
#include "pmm.h"
#include "uart.h"
#include "vga.h"

extern "C" {
#if !__STDC_HOSTED__
extern const uint8_t ap_trampoline_start[];
extern const uint8_t ap_trampoline_end[];
extern const uint8_t stack_bottom[];
extern const uint8_t stack_top[];
#endif

volatile uint64_t g_ap_boot_stack = 0;
volatile int g_ap_boot_cpu_index = 0;
volatile int g_ap_boot_started = 0;
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

#if !__STDC_HOSTED__
constexpr uint32_t kMsrIa32ApicBase = 0x0000001B;
constexpr uint32_t kMsrIa32Efer = 0xC0000080;
constexpr uint64_t kApicBaseGlobalEnable = 1ULL << 11;
constexpr uint64_t kEferLongModeActive = 1ULL << 10;
constexpr uint64_t kKernelCodeSelector = 0x08;

constexpr uint64_t kPtePresent = 1ULL << 0;
constexpr uint64_t kPteWriteThrough = 1ULL << 3;
constexpr uint64_t kPteCacheDisable = 1ULL << 4;
constexpr uint64_t kPteAddressMask = 0x000FFFFFFFFFF000ULL;

constexpr int kLapicRegId = 0x020;
constexpr int kLapicRegTpr = 0x080;
constexpr int kLapicRegSvr = 0x0F0;
constexpr int kLapicRegEsr = 0x280;
constexpr int kLapicRegIcrLow = 0x300;
constexpr int kLapicRegIcrHigh = 0x310;

constexpr uint32_t kLapicSvrSoftwareEnable = 1u << 8;
constexpr uint32_t kLapicSpuriousVector = 0xFFu;
constexpr uint32_t kIcrDeliveryInit = 0x00000500u;
constexpr uint32_t kIcrDeliveryStartup = 0x00000600u;
constexpr uint32_t kIcrDeliveryPending = 1u << 12;
constexpr uint32_t kIcrLevelAssert = 1u << 14;
constexpr uint32_t kIcrTriggerLevel = 1u << 15;

constexpr int64_t kMaxTrampolineBytes = 512;
constexpr int kIcrIdlePollLimit = 1000000;
constexpr int kInitDelayIterations = 50000;
constexpr int kFirstSipiPollLimit = 200000;
constexpr int kSecondSipiPollLimit = 50000000;
#endif

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

SmpTopology g_topology = {};
int g_online_cpu_count = 0;
int g_ap_boot_done = 0;
bool g_smp_initialized = false;

#if __STDC_HOSTED__
uint8_t g_host_bsp_apic_id = 0;
uintptr_t g_host_ebda_base = 0;
uintptr_t g_host_bios_rom_base = 0;
uintptr_t g_host_acpi32_high_bits = 0;
SmpHostApBootSimFn g_host_ap_sim_fn = nullptr;
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

static void AddDiscoveredCpu(SmpTopology* const topology,      //
                             const uint8_t apic_id,            //
                             const uint8_t acpi_processor_id,  //
                             const uint8_t bsp_apic_id) {
  DCHECK(topology != nullptr);
  DCHECK(apic_id != kInvalidXapicId);
  DCHECK(bsp_apic_id != kInvalidXapicId);
  for (int i = 0; i < topology->cpu_count; ++i) {
    if (topology->cpus[i].apic_id == apic_id) {
      return;
    }
  }
  if (topology->cpu_count >= kMaxCpus) {
    if (apic_id == bsp_apic_id) {
      CpuInfo& bsp_slot = topology->cpus[kMaxCpus - 1];
      bsp_slot = {};
      bsp_slot.apic_id = apic_id;
      bsp_slot.acpi_processor_id = acpi_processor_id;
    }
    return;
  }
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

static void ResetSmpState() {
  g_topology = {};
  __atomic_store_n(&g_online_cpu_count, 0, __ATOMIC_SEQ_CST);
  __atomic_store_n(&g_ap_boot_done, 0, __ATOMIC_SEQ_CST);
  __atomic_store_n(&g_ap_boot_started, 0, __ATOMIC_SEQ_CST);
  g_ap_boot_stack = 0;
  g_ap_boot_cpu_index = 0;
  g_smp_initialized = false;
}

#if !__STDC_HOSTED__
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

static void WriteCr3(const uintptr_t cr3_phys) {
  const uint64_t value = cr3_phys;
  asm volatile("mov cr3, %0" : : "r"(value) : "memory");
}

static void MarkMmioPageUncacheable(const uintptr_t phys_addr) {
  DCHECK(phys_addr != 0);
  DCHECK(phys_addr < kMaxCanonicalIdentityAddress);
  const uintptr_t pml4_phys = ReadCr3() & kPteAddressMask;
  DCHECK(pml4_phys != 0);
  const int pml4_idx = (phys_addr >> 39) & 0x1FF;
  const int pdpt_idx = (phys_addr >> 30) & 0x1FF;
  const int pd_idx = (phys_addr >> 21) & 0x1FF;

  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);
  DCHECK((pml4[pml4_idx] & kPtePresent) != 0);
  uint64_t* const pdpt =
      reinterpret_cast<uint64_t*>(pml4[pml4_idx] & kPteAddressMask);
  DCHECK((pdpt[pdpt_idx] & kPtePresent) != 0);
  uint64_t* const pd =
      reinterpret_cast<uint64_t*>(pdpt[pdpt_idx] & kPteAddressMask);
  DCHECK((pd[pd_idx] & kPtePresent) != 0);
  constexpr uint64_t kUcFlags = kPteWriteThrough | kPteCacheDisable;
  if ((pd[pd_idx] & kUcFlags) != kUcFlags) {
    pd[pd_idx] |= kUcFlags;
    WriteCr3(pml4_phys);
  }
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
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
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
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
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
  __atomic_store_n(&g_ap_boot_started, 0, __ATOMIC_SEQ_CST);
  __atomic_store_n(&g_ap_boot_done, 0, __ATOMIC_SEQ_CST);
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
    if (__atomic_load_n(&g_ap_boot_done, __ATOMIC_ACQUIRE) != 0) {
      return;
    }
    if (__atomic_load_n(&g_ap_boot_started, __ATOMIC_ACQUIRE) != 0) {
      break;
    }
    asm volatile("pause" : : : "memory");
  }

  // Send second Startup IPI (SIPI) only if the AP has not yet started
  // executing the trampoline on the first SIPI.
  if (__atomic_load_n(&g_ap_boot_done, __ATOMIC_ACQUIRE) == 0 &&
      __atomic_load_n(&g_ap_boot_started, __ATOMIC_ACQUIRE) == 0) {
    LapicWrite(lapic_base, kLapicRegEsr, 0);
    (void)LapicRead(lapic_base, kLapicRegEsr);
    LapicWrite(lapic_base, kLapicRegIcrHigh, icr_dest);
    LapicWrite(lapic_base,       //
               kLapicRegIcrLow,  //
               kIcrDeliveryStartup | kApTrampolineVector);
    CHECK(WaitForIcrIdle(lapic_base));
  }

  for (int i = 0; i < kSecondSipiPollLimit; ++i) {
    if (__atomic_load_n(&g_ap_boot_done, __ATOMIC_ACQUIRE) != 0) {
      return;
    }
    asm volatile("pause" : : : "memory");
  }
  CHECK(__atomic_load_n(&g_ap_boot_done, __ATOMIC_ACQUIRE) != 0);
}
#endif

}  // namespace

extern "C" void ApKernelEntry(const int cpu_index) {
#if !__STDC_HOSTED__
  __atomic_store_n(&g_ap_boot_started, 1, __ATOMIC_RELEASE);
  const uintptr_t lapic_base = g_topology.local_apic_phys_addr;
  DCHECK(lapic_base != 0);
  DCHECK(cpu_index > 0 && cpu_index < g_topology.cpu_count);
  EnableLocalApic(lapic_base);

  CpuInfo& cpu = g_topology.cpus[cpu_index];
  cpu.observed_apic_id = ReadLapicId(lapic_base);
  cpu.observed_rsp = ReadRsp();
  cpu.observed_cr3 = ReadCr3();
  cpu.long_mode_active = IsHardwareLongModeActive();
  cpu.online = true;

  __atomic_add_fetch(&g_online_cpu_count, 1, __ATOMIC_SEQ_CST);
  __atomic_store_n(&g_ap_boot_done, 1, __ATOMIC_RELEASE);

  for (;;) {
    asm volatile("cli; hlt");
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
        AddDiscoveredCpu(out_topology,              //
                         lapic->apic_id,            //
                         lapic->acpi_processor_id,  //
                         bsp_apic_id);
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
        AddDiscoveredCpu(out_topology,                                      //
                         static_cast<uint8_t>(x2apic->x2apic_id),           //
                         static_cast<uint8_t>(x2apic->acpi_processor_uid),  //
                         bsp_apic_id);
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
    const int shift_limit = (out_topology->cpu_count < kMaxCpus)
                                ? out_topology->cpu_count
                                : (kMaxCpus - 1);
    for (int i = shift_limit; i > 0; --i) {
      out_topology->cpus[i] = out_topology->cpus[i - 1];
    }
    if (out_topology->cpu_count < kMaxCpus) {
      ++out_topology->cpu_count;
    }
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

void SmpInit(const uint32_t multiboot_magic,
             const uint64_t multiboot_info_addr) {
  ResetSmpState();

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
  __atomic_store_n(&g_online_cpu_count, 1, __ATOMIC_SEQ_CST);

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
      __atomic_store_n(&g_ap_boot_started, 0, __ATOMIC_SEQ_CST);
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
      __atomic_add_fetch(&g_online_cpu_count, 1, __ATOMIC_SEQ_CST);
#endif
      CHECK(g_topology.cpus[i].online);

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

  g_smp_initialized = true;
  CHECK(SmpOnlineCpuCount() == g_topology.cpu_count);
  ConsoleWrite("[SMP] Online CPUs: ");
  ConsoleWriteDec(SmpOnlineCpuCount());
  ConsoleWrite("/");
  ConsoleWriteDec(g_topology.cpu_count);
  ConsoleWrite("\n");
}

int SmpCpuCount() {
  DCHECK(g_smp_initialized);
  return g_topology.cpu_count;
}

int SmpOnlineCpuCount() {
  DCHECK(g_smp_initialized);
  return __atomic_load_n(&g_online_cpu_count, __ATOMIC_ACQUIRE);
}

const CpuInfo* SmpGetCpuInfo(const int index) {
  DCHECK(g_smp_initialized);
  DCHECK(index >= 0 && index < g_topology.cpu_count);
  return &g_topology.cpus[index];
}

uintptr_t SmpLocalApicPhysAddr() {
  DCHECK(g_smp_initialized);
  return g_topology.local_apic_phys_addr;
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
}
#endif

}  // namespace protos
