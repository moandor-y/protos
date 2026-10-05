#include "paging.h"

#include <cstddef>
#include <cstdint>

#include "pmm.h"

namespace protos {

namespace {

constexpr size_t kEntriesPerPageTable = 512;
constexpr uint64_t kPtePresent = 1 << 0;
constexpr uint64_t kPteWritable = 1 << 1;
constexpr uint64_t kPteHugePage = 1 << 7;
constexpr uint64_t kPteAddressMask = 0x000FFFFFFFFFF000;
constexpr uint64_t kHugePageAddressMask = 0x000FFFFFFFE00000;
// Maximum canonical lower-half 48-bit physical/virtual address limit (128 TiB).
constexpr uintptr_t kMaxCanonicalIdentityAddress = 0x0000800000000000;

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
  return static_cast<uintptr_t>(cr3 & kPteAddressMask);
}

// Writes `cr3_phys` to CR3, flushing non-global TLB entries.
static void WriteCr3(const uintptr_t cr3_phys) {
  const uint64_t value = static_cast<uint64_t>(cr3_phys);
  asm volatile("mov cr3, %0" : : "r"(value) : "memory");
}

// Allocates a single 4 KiB frame from the PMM and zeroes all 512 64-bit page
// table entries. Returns the physical address of the table, or 0 on OOM.
static uintptr_t AllocZeroedPageTable() {
  const uintptr_t frame_phys = PmmAllocFrame();
  if (frame_phys == 0) {
    return 0;
  }
  uint64_t* const entries = reinterpret_cast<uint64_t*>(frame_phys);
  for (size_t i = 0; i < kEntriesPerPageTable; ++i) {
    entries[i] = 0;
  }
  return frame_phys;
}

}  // namespace

bool PagingExtendIdentityMap(const uintptr_t max_physical_addr) {
  if (max_physical_addr == 0 ||
      max_physical_addr > kMaxCanonicalIdentityAddress - kHugePageSize) {
    return false;
  }

  const uintptr_t target_end = AlignUp(max_physical_addr, kHugePageSize);
  const uintptr_t pml4_phys = ReadCr3();
  if (pml4_phys == 0) {
    return false;
  }
  uint64_t* const pml4 = reinterpret_cast<uint64_t*>(pml4_phys);

  for (uintptr_t addr = 0; addr < target_end; addr += kHugePageSize) {
    const size_t pml4_idx = (addr >> 39) & 0x1FF;
    const size_t pdpt_idx = (addr >> 30) & 0x1FF;
    const size_t pd_idx = (addr >> 21) & 0x1FF;

    if ((pml4[pml4_idx] & kPtePresent) == 0) {
      const uintptr_t new_pdpt_phys = AllocZeroedPageTable();
      if (new_pdpt_phys == 0) {
        return false;
      }
      pml4[pml4_idx] =
          static_cast<uint64_t>(new_pdpt_phys) | kPtePresent | kPteWritable;
    }

    uint64_t* const pdpt =
        reinterpret_cast<uint64_t*>(pml4[pml4_idx] & kPteAddressMask);
    if ((pdpt[pdpt_idx] & kPtePresent) == 0) {
      const uintptr_t new_pd_phys = AllocZeroedPageTable();
      if (new_pd_phys == 0) {
        return false;
      }
      pdpt[pdpt_idx] =
          static_cast<uint64_t>(new_pd_phys) | kPtePresent | kPteWritable;
    }

    uint64_t* const pd =
        reinterpret_cast<uint64_t*>(pdpt[pdpt_idx] & kPteAddressMask);
    if ((pd[pd_idx] & kPtePresent) == 0) {
      pd[pd_idx] = static_cast<uint64_t>(addr) | kPtePresent | kPteWritable |
                   kPteHugePage;
    }
  }

  WriteCr3(pml4_phys);
  if (target_end > g_identity_mapped_limit) {
    g_identity_mapped_limit = target_end;
  }
  return true;
}

bool PagingIsIdentityMapped(const uintptr_t addr) {
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
  const size_t pml4_idx = (addr >> 39) & 0x1FF;
  const size_t pdpt_idx = (addr >> 30) & 0x1FF;
  const size_t pd_idx = (addr >> 21) & 0x1FF;

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
  // entry is a present, writable 2 MiB huge page (PS=1).
  const uint64_t* const pd =
      reinterpret_cast<const uint64_t*>(pdpte & kPteAddressMask);
  const uint64_t pde = pd[pd_idx];
  {
    constexpr uint64_t kRequiredFlags =
        kPtePresent | kPteWritable | kPteHugePage;
    if ((pde & kRequiredFlags) != kRequiredFlags) {
      return false;
    }
  }

  // Confirm identity mapping by checking that the physical 2 MiB huge-page
  // base address encoded in the PD entry matches the 2 MiB-aligned virtual
  // address.
  const uintptr_t mapped_base =
      static_cast<uintptr_t>(pde & kHugePageAddressMask);
  const uintptr_t expected_base = addr & ~(kHugePageSize - 1);
  return mapped_base == expected_base;
}

uintptr_t PagingIdentityMappedLimit() { return g_identity_mapped_limit; }

}  // namespace protos
