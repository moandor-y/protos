#include "paging.h"

#include <cstdint>

#include "pmm.h"
#include "spinlock.h"

namespace protos {

namespace {

constexpr int kEntriesPerPageTable = 512;
constexpr int kMaxBootstrapPageTables = 16;
constexpr uint64_t kPtePresent = 1ULL << 0;
constexpr uint64_t kPteWritable = 1ULL << 1;
constexpr uint64_t kPteWriteThrough = 1ULL << 3;
constexpr uint64_t kPteCacheDisable = 1ULL << 4;
constexpr uint64_t kPteHugePage = 1ULL << 7;
constexpr uint64_t kPteAddressMask = 0x000FFFFFFFFFF000ULL;
constexpr uint64_t kHugePageAddressMask = 0x000FFFFFFFE00000ULL;

IrqSpinLock g_paging_lock;

// Static fallback pool of 4 KiB-aligned page tables in `.bss` used when
// mapping high-memory bootloader structures (e.g., UEFI Multiboot2 info,
// GOP framebuffer, or PMM bitmap above 64 MiB) before `PmmInit` completes.
alignas(4096) uint64_t
    g_bootstrap_page_tables[kMaxBootstrapPageTables][kEntriesPerPageTable];
int g_bootstrap_page_tables_used = 0;

uintptr_t g_identity_mapped_limit = kBootstrapIdentityMapSize;

// Rounds `value` up to the nearest multiple of `alignment` (power of two).
static constexpr uintptr_t AlignUp(const uintptr_t value,
                                   const uintptr_t alignment) {
  return (value + alignment - 1) & ~(alignment - 1);
}

// Reads the physical base address of the active PML4 table from CR3.
static uintptr_t ReadCr3() {
  uint64_t cr3 = 0;
  asm volatile("mov %0, cr3" : "=r"(cr3));
  return cr3 & kPteAddressMask;
}

// Writes `cr3_phys` to CR3, flushing non-global TLB entries.
static void WriteCr3(const uintptr_t cr3_phys) {
  const uint64_t value = cr3_phys;
  asm volatile("mov cr3, %0" : : "r"(value) : "memory");
}

// Allocates a single 4 KiB page-table frame (from the PMM when `use_pmm` is
// true, falling back to the static bootstrap pool in `.bss` when `use_pmm` is
// false or the PMM has no free low-memory frames) and zeroes all 512 64-bit
// page-table entries. Returns the physical address of the table, or 0 on OOM.
static uintptr_t AllocZeroedPageTable(const bool use_pmm) {
  uintptr_t frame_phys = use_pmm ? PmmAllocFrame() : 0;
  if (frame_phys == 0) {
    if (g_bootstrap_page_tables_used >= kMaxBootstrapPageTables) {
      return 0;
    }
    frame_phys = reinterpret_cast<uintptr_t>(
        g_bootstrap_page_tables[g_bootstrap_page_tables_used]);
    ++g_bootstrap_page_tables_used;
  }
  uint64_t* const entries = reinterpret_cast<uint64_t*>(frame_phys);
  for (int i = 0; i < kEntriesPerPageTable; ++i) {
    entries[i] = 0;
  }
  return frame_phys;
}

static bool MapHugePageRangeLocked(const uintptr_t phys_addr,  //
                                   const int64_t size,         //
                                   const bool use_pmm) {
  if (size <= 0) {
    return size == 0;
  }
  if (phys_addr >= kMaxCanonicalIdentityAddress ||
      (kMaxCanonicalIdentityAddress - phys_addr) <
          static_cast<uintptr_t>(size)) {
    return false;
  }

  const uintptr_t end_addr = phys_addr + size;
  if (end_addr > kMaxCanonicalIdentityAddress - kHugePageSize) {
    return false;
  }
  const uintptr_t start_page = phys_addr & ~(kHugePageSize - 1);
  const uintptr_t end_page = AlignUp(end_addr, kHugePageSize);

  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);
  bool tlb_flush_needed = false;

  for (uintptr_t addr = start_page; addr < end_page; addr += kHugePageSize) {
    const int pml4_idx = (addr >> 39) & 0x1FF;
    const int pdpt_idx = (addr >> 30) & 0x1FF;
    const int pd_idx = (addr >> 21) & 0x1FF;

    if ((pml4[pml4_idx] & kPtePresent) == 0) {
      const uintptr_t new_pdpt_phys = AllocZeroedPageTable(use_pmm);
      if (new_pdpt_phys == 0) {
        return false;
      }
      pml4[pml4_idx] = new_pdpt_phys | kPtePresent | kPteWritable;
      tlb_flush_needed = true;
    }

    uint64_t* const pdpt =
        reinterpret_cast<uint64_t*>(pml4[pml4_idx] & kPteAddressMask);
    if ((pdpt[pdpt_idx] & kPtePresent) == 0) {
      const uintptr_t new_pd_phys = AllocZeroedPageTable(use_pmm);
      if (new_pd_phys == 0) {
        return false;
      }
      pdpt[pdpt_idx] = new_pd_phys | kPtePresent | kPteWritable;
      tlb_flush_needed = true;
    }

    uint64_t* const pd =
        reinterpret_cast<uint64_t*>(pdpt[pdpt_idx] & kPteAddressMask);
    if ((pd[pd_idx] & kPtePresent) == 0) {
      pd[pd_idx] = addr | kPtePresent | kPteWritable | kPteHugePage;
      tlb_flush_needed = true;
    }
  }

  if (tlb_flush_needed) {
    WriteCr3(pml4_phys);
  }
  return true;
}

}  // namespace

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  return MapHugePageRangeLocked(phys_addr, size, false);
}

bool PagingExtendIdentityMap(const uintptr_t max_physical_addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if (max_physical_addr == 0 ||
      max_physical_addr > kMaxCanonicalIdentityAddress - kHugePageSize) {
    return false;
  }

  const uintptr_t target_end = AlignUp(max_physical_addr, kHugePageSize);
  if (!MapHugePageRangeLocked(0, target_end, true)) {
    return false;
  }
  if (target_end > g_identity_mapped_limit) {
    g_identity_mapped_limit = target_end;
  }
  return true;
}

bool PagingMarkPageUncacheable(const uintptr_t phys_addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  if (phys_addr == 0 || phys_addr >= kMaxCanonicalIdentityAddress) {
    return false;
  }
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }

  const int pml4_idx = (phys_addr >> 39) & 0x1FF;
  const int pdpt_idx = (phys_addr >> 30) & 0x1FF;
  const int pd_idx = (phys_addr >> 21) & 0x1FF;
  const int pt_idx = (phys_addr >> 12) & 0x1FF;

  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);
  if ((pml4[pml4_idx] & kPtePresent) == 0) {
    return false;
  }
  uint64_t* const pdpt =
      reinterpret_cast<uint64_t*>(pml4[pml4_idx] & kPteAddressMask);
  if ((pdpt[pdpt_idx] & kPtePresent) == 0) {
    return false;
  }
  uint64_t* const pd =
      reinterpret_cast<uint64_t*>(pdpt[pdpt_idx] & kPteAddressMask);
  const uint64_t pde = pd[pd_idx];
  if ((pde & kPtePresent) == 0) {
    return false;
  }

  constexpr uint64_t kUcFlags = kPteWriteThrough | kPteCacheDisable;
  if ((pde & kPteHugePage) != 0) {
    const uintptr_t pt_phys = AllocZeroedPageTable(true);
    if (pt_phys == 0) {
      return false;
    }
    uint64_t* const pt = reinterpret_cast<uint64_t*>(pt_phys);
    const uintptr_t huge_base = pde & kHugePageAddressMask;
    const uint64_t base_flags = pde & (kPtePresent | kPteWritable | kUcFlags);
    for (int i = 0; i < kEntriesPerPageTable; ++i) {
      pt[i] = (huge_base + static_cast<uintptr_t>(i) * kPageSize) | base_flags;
    }
    pt[pt_idx] |= kUcFlags;
    pd[pd_idx] = pt_phys | kPtePresent | kPteWritable;
    WriteCr3(pml4_phys);
    return true;
  }

  uint64_t* const pt = reinterpret_cast<uint64_t*>(pde & kPteAddressMask);
  if ((pt[pt_idx] & kPtePresent) == 0) {
    return false;
  }
  if ((pt[pt_idx] & kUcFlags) != kUcFlags) {
    pt[pt_idx] |= kUcFlags;
    WriteCr3(pml4_phys);
  }
  return true;
}

bool PagingIsIdentityMapped(const uintptr_t addr) {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  // Reject non-canonical lower-half addresses beyond the 48-bit identity limit.
  if (addr >= kMaxCanonicalIdentityAddress) {
    return false;
  }
  // Locate the active top-level PML4 table from CR3.
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }

  // Extract the 9-bit page table indices for PML4 (bits 47:39), PDPT (bits
  // 38:30), and PD (bits 29:21) from the virtual address.
  const int pml4_idx = (addr >> 39) & 0x1FF;
  const int pdpt_idx = (addr >> 30) & 0x1FF;
  const int pd_idx = (addr >> 21) & 0x1FF;

  // Check that the PML4 entry is present and writable.
  const uint64_t* const pml4 = reinterpret_cast<const uint64_t*>(pml4_phys);
  const uint64_t pml4e = pml4[pml4_idx];
  if ((pml4e & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }

  // Follow the PML4 entry to the PDPT and check that the PDPT entry is present
  // and writable.
  const uint64_t* const pdpt =
      reinterpret_cast<const uint64_t*>(pml4e & kPteAddressMask);
  const uint64_t pdpte = pdpt[pdpt_idx];
  if ((pdpte & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }

  // Follow the PDPT entry to the Page Directory (PD) and verify that the PD
  // entry is present and writable.
  const uint64_t* const pd =
      reinterpret_cast<const uint64_t*>(pdpte & kPteAddressMask);
  const uint64_t pde = pd[pd_idx];
  if ((pde & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }

  if ((pde & kPteHugePage) != 0) {
    // Confirm identity mapping by checking that the physical 2 MiB huge-page
    // base address encoded in the PD entry matches the 2 MiB-aligned virtual
    // address.
    const uintptr_t mapped_base = pde & kHugePageAddressMask;
    const uintptr_t expected_base = addr & ~(kHugePageSize - 1);
    return mapped_base == expected_base;
  }

  // 4 KiB page table (`PS == 0`): walk the PT entry for `addr`.
  const int pt_idx = (addr >> 12) & 0x1FF;
  const uint64_t* const pt =
      reinterpret_cast<const uint64_t*>(pde & kPteAddressMask);
  const uint64_t pte = pt[pt_idx];
  if ((pte & (kPtePresent | kPteWritable)) != (kPtePresent | kPteWritable)) {
    return false;
  }
  const uintptr_t mapped_page = pte & kPteAddressMask;
  const uintptr_t expected_page =
      addr & ~(static_cast<uintptr_t>(kPageSize) - 1);
  return mapped_page == expected_page;
}

uintptr_t PagingIdentityMappedLimit() {
  const IrqSpinLockGuard lock_guard(g_paging_lock);
  return g_identity_mapped_limit;
}

}  // namespace protos
