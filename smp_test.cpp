#include "smp.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "paging.h"
#include "pmm.h"
#include "uart.h"
#include "vga.h"

namespace protos {
namespace {

namespace t = ::testing;

constexpr uint32_t kMultiboot1Magic = 0x2BADB002;
constexpr uint32_t kMultiboot2Magic = 0x36D76289;
constexpr int64_t kFakeRamFrames = 256;
constexpr int64_t kFakeRamBytes = kFakeRamFrames * kPageSize;

alignas(kPageSize) uint8_t g_fake_ram[kFakeRamBytes];

struct [[gnu::packed]] RawRsdpV1 {
  char signature[8];
  uint8_t checksum;
  char oem_id[6];
  uint8_t revision;
  uint32_t rsdt_address;
};

struct [[gnu::packed]] RawRsdpV2 {
  RawRsdpV1 v1;
  uint32_t length;
  uint64_t xsdt_address;
  uint8_t extended_checksum;
  uint8_t reserved[3];
};

struct [[gnu::packed]] RawSdtHeader {
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

struct [[gnu::packed]] RawMadtHeader {
  RawSdtHeader sdt;
  uint32_t local_apic_address;
  uint32_t flags;
};

struct [[gnu::packed]] RawMadtLapic {
  uint8_t type;
  uint8_t length;
  uint8_t acpi_processor_id;
  uint8_t apic_id;
  uint32_t flags;
};

struct [[gnu::packed]] RawMadtLapicOverride {
  uint8_t type;
  uint8_t length;
  uint16_t reserved;
  uint64_t local_apic_address;
};

struct [[gnu::packed]] RawMadtX2Apic {
  uint8_t type;
  uint8_t length;
  uint16_t reserved;
  uint32_t x2apic_id;
  uint32_t flags;
  uint32_t acpi_processor_uid;
};

struct FakeHostState {
  bool map_ok = true;
  uintptr_t fail_map_addr = 0;
  int64_t next_pmm_frame = 32;
  int64_t max_pmm_frame = kFakeRamFrames;
  int pmm_alloc_calls = 0;
  int fail_pmm_on_call = -1;
  std::string uart_log;
  std::string vga_log;
};

FakeHostState g_state;

static uintptr_t BaseAddr() { return reinterpret_cast<uintptr_t>(g_fake_ram); }

static uintptr_t OffsetAddr(const int64_t offset) {
  return BaseAddr() + offset;
}

static void ResetState(const uintptr_t acpi32_arena = 0) {
  g_state = FakeHostState{};
  std::memset(g_fake_ram, 0, sizeof(g_fake_ram));
  SmpSetHostTestHooks(0, 0, 0, acpi32_arena, nullptr);
}

static uint8_t ComputeChecksumByte(const void* const data,
                                   const int64_t length) {
  const uint8_t* const bytes = static_cast<const uint8_t*>(data);
  uint8_t sum = 0;
  for (int64_t i = 0; i < length; ++i) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return static_cast<uint8_t>(0u - sum);
}

static void WriteRsdpV1(const uintptr_t addr, const uint32_t rsdt_addr) {
  RawRsdpV1* const rsdp = reinterpret_cast<RawRsdpV1*>(addr);
  std::memset(rsdp, 0, sizeof(RawRsdpV1));
  std::memcpy(rsdp->signature, "RSD PTR ", 8);
  std::memcpy(rsdp->oem_id, "PROTOS", 6);
  rsdp->revision = 0;
  rsdp->rsdt_address = rsdt_addr;
  rsdp->checksum = ComputeChecksumByte(rsdp, sizeof(RawRsdpV1));
}

static void WriteRsdpV2(const uintptr_t addr,      //
                        const uint32_t rsdt_addr,  //
                        const uint64_t xsdt_addr) {
  RawRsdpV2* const rsdp = reinterpret_cast<RawRsdpV2*>(addr);
  std::memset(rsdp, 0, sizeof(RawRsdpV2));
  std::memcpy(rsdp->v1.signature, "RSD PTR ", 8);
  std::memcpy(rsdp->v1.oem_id, "PROTOS", 6);
  rsdp->v1.revision = 2;
  rsdp->v1.rsdt_address = rsdt_addr;
  rsdp->v1.checksum = ComputeChecksumByte(&rsdp->v1, sizeof(RawRsdpV1));
  rsdp->length = sizeof(RawRsdpV2);
  rsdp->xsdt_address = xsdt_addr;
  rsdp->extended_checksum = ComputeChecksumByte(rsdp, sizeof(RawRsdpV2));
}

static void FinalizeSdt(RawSdtHeader* const sdt,  //
                        const char* const sig,    //
                        const int64_t total_length) {
  std::memcpy(sdt->signature, sig, 4);
  sdt->length = static_cast<uint32_t>(total_length);
  sdt->revision = 1;
  std::memcpy(sdt->oem_id, "PROTOS", 6);
  std::memcpy(sdt->oem_table_id, "SMPTABLE", 8);
  sdt->checksum = 0;
  sdt->checksum = ComputeChecksumByte(sdt, total_length);
}

static void WriteRsdt(const uintptr_t addr,
                      const std::vector<uint32_t>& entries) {
  RawSdtHeader* const sdt = reinterpret_cast<RawSdtHeader*>(addr);
  const int64_t total_len =
      sizeof(RawSdtHeader) + entries.size() * sizeof(uint32_t);
  std::memset(sdt, 0, total_len);
  uint8_t* const payload =
      reinterpret_cast<uint8_t*>(addr) + sizeof(RawSdtHeader);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const uint32_t val = entries[i];
    std::memcpy(payload + i * sizeof(uint32_t), &val, sizeof(uint32_t));
  }
  FinalizeSdt(sdt, "RSDT", total_len);
}

static void WriteXsdt(const uintptr_t addr,
                      const std::vector<uint64_t>& entries) {
  RawSdtHeader* const sdt = reinterpret_cast<RawSdtHeader*>(addr);
  const int64_t total_len =
      sizeof(RawSdtHeader) + entries.size() * sizeof(uint64_t);
  std::memset(sdt, 0, total_len);
  uint8_t* const payload =
      reinterpret_cast<uint8_t*>(addr) + sizeof(RawSdtHeader);
  for (int i = 0; i < static_cast<int>(entries.size()); ++i) {
    const uint64_t val = entries[i];
    std::memcpy(payload + i * sizeof(uint64_t), &val, sizeof(uint64_t));
  }
  FinalizeSdt(sdt, "XSDT", total_len);
}

static void WriteMadt(const uintptr_t addr,            //
                      const uint32_t local_apic_addr,  //
                      const std::vector<uint8_t>& records) {
  RawMadtHeader* const madt = reinterpret_cast<RawMadtHeader*>(addr);
  const int64_t total_len = sizeof(RawMadtHeader) + records.size();
  std::memset(madt, 0, total_len);
  madt->local_apic_address = local_apic_addr;
  madt->flags = 1;
  if (!records.empty()) {
    std::memcpy(reinterpret_cast<uint8_t*>(addr) + sizeof(RawMadtHeader),  //
                records.data(),                                            //
                records.size());
  }
  FinalizeSdt(&madt->sdt, "APIC", total_len);
}

static void AppendLapicRecord(std::vector<uint8_t>* const out,  //
                              const uint8_t acpi_id,            //
                              const uint8_t apic_id,            //
                              const uint32_t flags) {
  RawMadtLapic rec = {};
  rec.type = 0;
  rec.length = sizeof(RawMadtLapic);
  rec.acpi_processor_id = acpi_id;
  rec.apic_id = apic_id;
  rec.flags = flags;
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&rec);
  out->insert(out->end(), bytes, bytes + sizeof(rec));
}

static void AppendLapicOverrideRecord(std::vector<uint8_t>* const out,
                                      const uint64_t lapic_phys_addr) {
  RawMadtLapicOverride rec = {};
  rec.type = 5;
  rec.length = sizeof(RawMadtLapicOverride);
  rec.reserved = 0;
  rec.local_apic_address = lapic_phys_addr;
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&rec);
  out->insert(out->end(), bytes, bytes + sizeof(rec));
}

static void AppendX2ApicRecord(std::vector<uint8_t>* const out,  //
                               const uint32_t x2apic_id,         //
                               const uint32_t uid,               //
                               const uint32_t flags) {
  RawMadtX2Apic rec = {};
  rec.type = 9;
  rec.length = sizeof(RawMadtX2Apic);
  rec.reserved = 0;
  rec.x2apic_id = x2apic_id;
  rec.flags = flags;
  rec.acpi_processor_uid = uid;
  const uint8_t* const bytes = reinterpret_cast<const uint8_t*>(&rec);
  out->insert(out->end(), bytes, bytes + sizeof(rec));
}

}  // namespace

bool PagingMapBootstrapRange(const uintptr_t phys_addr, const int64_t size) {
  (void)size;
  if (!g_state.map_ok) {
    return false;
  }
  if (g_state.fail_map_addr != 0 && phys_addr == g_state.fail_map_addr) {
    return false;
  }
  return true;
}

uintptr_t PmmAllocFrames(const int64_t count) {
  ++g_state.pmm_alloc_calls;
  if (g_state.fail_pmm_on_call == g_state.pmm_alloc_calls) {
    return 0;
  }
  if (count <= 0 || g_state.next_pmm_frame + count > g_state.max_pmm_frame) {
    return 0;
  }
  const uintptr_t addr = BaseAddr() + g_state.next_pmm_frame * kPageSize;
  g_state.next_pmm_frame += count;
  return addr;
}

void UartWrite(const char* const str) { g_state.uart_log.append(str); }

void UartWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llx",     //
                static_cast<unsigned long long>(value));
  g_state.uart_log.append(buf);
}

void UartWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_state.uart_log.append(buf);
}

void VgaWrite(const char* const str) { g_state.vga_log.append(str); }

void VgaWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llx",     //
                static_cast<unsigned long long>(value));
  g_state.vga_log.append(buf);
}

void VgaWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  g_state.vga_log.append(buf);
}

namespace {

TEST(SmpTest, ParseRsdpValidatesV1AndV2AndRejectsMalformed) {
  ResetState();
  const uintptr_t v1_addr = OffsetAddr(0x100);
  const uintptr_t v2_addr = OffsetAddr(0x200);
  WriteRsdpV1(v1_addr, 0x12345000);
  WriteRsdpV2(v2_addr, 0x12345000, 0x9876543200ULL);

  uintptr_t rsdt = 0;
  uintptr_t xsdt = 0;
  EXPECT_TRUE(
      SmpParseRsdp(v1_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  EXPECT_THAT(rsdt, t::Eq(0x12345000u));
  EXPECT_THAT(xsdt, t::Eq(0u));

  EXPECT_TRUE(
      SmpParseRsdp(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  EXPECT_THAT(rsdt, t::Eq(0x12345000u));
  EXPECT_THAT(xsdt, t::Eq(0x9876543200ULL));

  // Verify that an extended RSDP v2 with length > 36 (e.g. 40 bytes with
  // non-zero trailing vendor bytes and a valid 40-byte extended checksum) is
  // accepted per ACPI 2.0+ specification.
  RawRsdpV2* const raw_v2 = reinterpret_cast<RawRsdpV2*>(v2_addr);
  raw_v2->length = 40;
  reinterpret_cast<uint8_t*>(v2_addr)[36] = 0x37;
  reinterpret_cast<uint8_t*>(v2_addr)[37] = 0x13;
  raw_v2->extended_checksum = 0;
  raw_v2->extended_checksum = ComputeChecksumByte(raw_v2, 40);
  EXPECT_TRUE(
      SmpParseRsdp(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  EXPECT_THAT(xsdt, t::Eq(0x9876543200ULL));
  WriteRsdpV2(v2_addr, 0x12345000, 0x9876543200ULL);

  // Out-of-bounds physical limit must fail cleanly.
  EXPECT_FALSE(SmpParseRsdp(v1_addr, v1_addr + 10, &rsdt, &xsdt));

  // Corrupted v2 extended checksum must be rejected.
  raw_v2->extended_checksum ^= 0xFF;
  EXPECT_FALSE(
      SmpParseRsdp(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
  raw_v2->extended_checksum ^= 0xFF;

  // Invalid v2 length (< 36) must be rejected.
  raw_v2->length = 20;
  raw_v2->extended_checksum = ComputeChecksumByte(raw_v2, sizeof(RawRsdpV2));
  EXPECT_FALSE(
      SmpParseRsdp(v2_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));

  // Corrupted v1 checksum or signature must be rejected.
  RawRsdpV1* const raw_v1 = reinterpret_cast<RawRsdpV1*>(v1_addr);
  raw_v1->checksum ^= 0x01;
  EXPECT_FALSE(
      SmpParseRsdp(v1_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));

  WriteRsdpV1(v1_addr, 0);
  EXPECT_FALSE(
      SmpParseRsdp(v1_addr, kMaxCanonicalIdentityAddress, &rsdt, &xsdt));
}

TEST(SmpTest, FindRsdpInMemoryRangeHonorsAlignmentAndChecksums) {
  ResetState();
  const uintptr_t scan_base = OffsetAddr(0x1000);

  // Place a misaligned RSDP at +8 (must be skipped) and a corrupted RSDP at
  // +32 (must be skipped), followed by a valid RSDP at +64.
  WriteRsdpV1(scan_base + 8, 0x11110000);
  WriteRsdpV1(scan_base + 32, 0x22220000);
  reinterpret_cast<RawRsdpV1*>(scan_base + 32)->checksum ^= 0x55;
  WriteRsdpV1(scan_base + 64, 0x33330000);

  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base, 1024), t::Eq(scan_base + 64));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base, 64), t::Eq(0u));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base, 16), t::Eq(0u));

  // Verify that a 36-byte v2 RSDP placed at +64 is rejected if the scan range
  // ends at +64 + 24 (truncating the v2 structure before 36 bytes), or if its
  // v2 extended checksum is corrupted even when its v1 checksum is valid.
  WriteRsdpV2(scan_base + 64, 0x33330000, 0x44440000ULL);
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base + 64, 24), t::Eq(0u));
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base + 64, 36),
              t::Eq(scan_base + 64));
  reinterpret_cast<RawRsdpV2*>(scan_base + 64)->extended_checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMemoryRange(scan_base + 64, 36), t::Eq(0u));
}

TEST(SmpTest, FindRsdpInMultiboot2PrefersAcpiNewAndFallsBackToOld) {
  ResetState();
  const uintptr_t mb_base = OffsetAddr(0x2000);
  uint8_t* const buf = reinterpret_cast<uint8_t*>(mb_base);

  // Build a Multiboot2 info structure with:
  // - dummy tag (type 6, size 12, padded to 16)
  // - Tag 14 (ACPI_OLD, size 28, padded to 32)
  // - Tag 15 (ACPI_NEW, size 44, padded to 48)
  // - End tag (type 0, size 8)
  int64_t pos = 8;

  // Dummy tag at offset 8
  *reinterpret_cast<uint32_t*>(buf + pos) = 6;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 12;
  pos += 16;

  // Tag 14 (ACPI_OLD) at offset 24
  const uintptr_t old_rsdp_addr = mb_base + pos + 8;
  *reinterpret_cast<uint32_t*>(buf + pos) = 14;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8 + sizeof(RawRsdpV1);
  WriteRsdpV1(old_rsdp_addr, 0xAAAA0000);
  pos += 32;

  // Tag 15 (ACPI_NEW) at offset 56
  const uintptr_t new_rsdp_addr = mb_base + pos + 8;
  *reinterpret_cast<uint32_t*>(buf + pos) = 15;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8 + sizeof(RawRsdpV2);
  WriteRsdpV2(new_rsdp_addr, 0xAAAA0000, 0xBBBB0000ULL);
  pos += 48;

  // End tag
  *reinterpret_cast<uint32_t*>(buf + pos) = 0;
  *reinterpret_cast<uint32_t*>(buf + pos + 4) = 8;
  pos += 8;

  *reinterpret_cast<uint32_t*>(buf) = static_cast<uint32_t>(pos);
  *reinterpret_cast<uint32_t*>(buf + 4) = 0;

  // When both Tag 14 and Tag 15 are valid, Tag 15 (ACPI 2.0+) is preferred.
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(new_rsdp_addr));

  // If Tag 15 claims an RSDP v2 length (e.g. 512) larger than the Multiboot2
  // tag payload size (36), it must be rejected without reading out-of-bounds,
  // falling back to Tag 14.
  reinterpret_cast<RawRsdpV2*>(new_rsdp_addr)->length = 512;
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(old_rsdp_addr));
  WriteRsdpV2(new_rsdp_addr, 0xAAAA0000, 0xBBBB0000ULL);

  // If Tag 15 has a corrupted extended checksum, fallback to Tag 14 succeeds.
  reinterpret_cast<RawRsdpV2*>(new_rsdp_addr)->extended_checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(old_rsdp_addr));

  // If Tag 14 is also corrupted, but Tag 15's v1 header (rsdt_address != 0) is
  // still valid, Tag 15's v1 header is used as fallback.
  reinterpret_cast<RawRsdpV1*>(old_rsdp_addr)->checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(new_rsdp_addr));

  // If Tag 15's v1 checksum is also corrupted, returns 0.
  reinterpret_cast<RawRsdpV2*>(new_rsdp_addr)->v1.checksum ^= 0xFF;
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(0u));

  // Malformed Multiboot2 total_size (< 8 or > 4 MiB) must return 0.
  *reinterpret_cast<uint32_t*>(buf) = 4;
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(0u));
  *reinterpret_cast<uint32_t*>(buf) = 8 * 1024 * 1024;
  EXPECT_THAT(SmpFindRsdpInMultiboot2(mb_base, kMaxCanonicalIdentityAddress),
              t::Eq(0u));
}

TEST(SmpTest, FindMadtInSdtSearchesXsdtAndFallsBackToRsdt) {
  ResetState(BaseAddr());
  const uintptr_t facp_addr = OffsetAddr(0x3000);
  const uintptr_t bad_madt_addr = OffsetAddr(0x3200);
  const uintptr_t good_madt_addr = OffsetAddr(0x3400);
  const uintptr_t rsdt_addr = OffsetAddr(0x3600);
  const uintptr_t xsdt_addr = OffsetAddr(0x3800);

  // Non-MADT table ("FACP").
  FinalizeSdt(reinterpret_cast<RawSdtHeader*>(facp_addr),  //
              "FACP",                                      //
              sizeof(RawSdtHeader));

  // Corrupted MADT ("APIC" with bad checksum).
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  WriteMadt(bad_madt_addr, kDefaultLocalApicPhysAddr, recs);
  reinterpret_cast<RawSdtHeader*>(bad_madt_addr)->checksum ^= 0x42;

  // Valid MADT ("APIC").
  WriteMadt(good_madt_addr, kDefaultLocalApicPhysAddr, recs);

  WriteXsdt(xsdt_addr, {facp_addr, bad_madt_addr, good_madt_addr});
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(facp_addr),
                        static_cast<uint32_t>(bad_madt_addr),
                        static_cast<uint32_t>(good_madt_addr)});

  // Valid XSDT resolves good_madt_addr (skipping FACP and bad_madt_addr).
  EXPECT_THAT(SmpFindMadtInSdt(0, xsdt_addr, kMaxCanonicalIdentityAddress),
              t::Eq(good_madt_addr));

  // If XSDT checksum is corrupted, falls back to 32-bit RSDT and resolves
  // good_madt_addr.
  reinterpret_cast<RawSdtHeader*>(xsdt_addr)->checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindMadtInSdt(rsdt_addr, xsdt_addr, kMaxCanonicalIdentityAddress),
      t::Eq(good_madt_addr));

  // If RSDT checksum is also corrupted, returns 0.
  reinterpret_cast<RawSdtHeader*>(rsdt_addr)->checksum ^= 0xFF;
  EXPECT_THAT(
      SmpFindMadtInSdt(rsdt_addr, xsdt_addr, kMaxCanonicalIdentityAddress),
      t::Eq(0u));
}

TEST(SmpTest, ParseMadtHandlesBspReorderingFilteringDuplicatesAndOverrides) {
  ResetState();
  const uintptr_t madt_addr = OffsetAddr(0x4000);

  std::vector<uint8_t> recs;
  // CPU with APIC ID 2 (Enabled)
  AppendLapicRecord(&recs, 10, 2, 1);
  // Disabled CPU with APIC ID 9 (flags == 0, must be ignored)
  AppendLapicRecord(&recs, 11, 9, 0);
  // Broadcast/invalid 8-bit xAPIC ID 0xFF in Type 0 (must be ignored)
  AppendLapicRecord(&recs, 99, 0xFF, 1);
  // Unknown entry type 2 (Interrupt Source Override, 10 bytes, must be skipped)
  const uint8_t iso_entry[10] = {2, 10, 0, 0, 2, 0, 0, 0, 0, 0};
  recs.insert(recs.end(), iso_entry, iso_entry + sizeof(iso_entry));
  // BSP with APIC ID 4 (Online-Capable flag == 2)
  AppendLapicRecord(&recs, 12, 4, 2);
  // Duplicate APIC ID 2 (must be deduplicated)
  AppendLapicRecord(&recs, 13, 2, 1);
  // CPU with APIC ID 7 via Local x2APIC (Type 9)
  AppendX2ApicRecord(&recs, 7, 14, 1);
  // x2APIC entry with ID >= 0xFF (must be ignored in xAPIC mode)
  AppendX2ApicRecord(&recs, 255, 15, 1);
  // 64-bit Local APIC Address Override (Type 5)
  AppendLapicOverrideRecord(&recs, 0xFEE01000ULL);
  // Misaligned 64-bit Local APIC Address Override (must be ignored)
  AppendLapicOverrideRecord(&recs, 0xFEE02123ULL);

  // Also pass a misaligned 32-bit local_apic_address in the MADT header to
  // verify it falls back to kDefaultLocalApicPhysAddr before the valid Type 5
  // override replaces it with 0xFEE01000.
  WriteMadt(madt_addr, 0xFEE00123u, recs);

  SmpTopology topo = {};
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 4, &topo));
  EXPECT_THAT(topo.local_apic_phys_addr, t::Eq(0xFEE01000ULL));
  EXPECT_THAT(topo.bsp_apic_id, t::Eq(4));
  ASSERT_THAT(topo.cpu_count, t::Eq(3));

  // BSP (APIC ID 4) must be placed at index 0, preserving relative order of APs
  // (APIC ID 2 at index 1, APIC ID 7 at index 2).
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(4));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
  EXPECT_TRUE(topo.cpus[0].online);

  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2));
  EXPECT_THAT(topo.cpus[1].acpi_processor_id, t::Eq(10));
  EXPECT_FALSE(topo.cpus[1].is_bsp);

  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(7));
  EXPECT_THAT(topo.cpus[2].acpi_processor_id, t::Eq(14));
  EXPECT_FALSE(topo.cpus[2].is_bsp);

  // If bsp_apic_id (e.g. 99) was omitted from the MADT, it is inserted at
  // index 0 and existing CPUs are shifted right.
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 99, &topo));
  ASSERT_THAT(topo.cpu_count, t::Eq(4));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(99));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
  EXPECT_THAT(topo.cpus[1].apic_id, t::Eq(2));
  EXPECT_THAT(topo.cpus[2].apic_id, t::Eq(4));
  EXPECT_THAT(topo.cpus[3].apic_id, t::Eq(7));
}

TEST(SmpTest, ParseMadtHandlesMalformedEntriesAndClampsMaxCpus) {
  ResetState();
  const uintptr_t madt_addr = OffsetAddr(0x5000);

  // Entry with length == 0 after two valid CPUs must terminate parsing cleanly
  // without an infinite loop.
  std::vector<uint8_t> recs;
  AppendLapicRecord(&recs, 0, 0, 1);
  AppendLapicRecord(&recs, 1, 1, 1);
  const uint8_t zero_len_entry[2] = {0, 0};
  recs.insert(recs.end(), zero_len_entry, zero_len_entry + 2);
  AppendLapicRecord(&recs, 2, 2, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);

  SmpTopology topo = {};
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  EXPECT_THAT(topo.cpu_count, t::Eq(2));

  // Truncated Type 0 entry (length == 4 < 8) stops parsing cleanly.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 1);
  const uint8_t short_type0[4] = {0, 4, 1, 1};
  recs.insert(recs.end(), short_type0, short_type0 + sizeof(short_type0));
  AppendLapicRecord(&recs, 2, 2, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  EXPECT_THAT(topo.cpu_count, t::Eq(1));

  // MADT with only disabled CPUs must return false.
  recs.clear();
  AppendLapicRecord(&recs, 0, 0, 0);
  AppendLapicRecord(&recs, 1, 1, 0);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  EXPECT_FALSE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));

  // MADT with more than kMaxCpus enabled processors must clamp to kMaxCpus,
  // and if the BSP appears after the first kMaxCpus entries, its real MADT
  // acpi_processor_id must still be preserved at index 0.
  recs.clear();
  for (int i = 1; i <= kMaxCpus + 10; ++i) {
    AppendLapicRecord(&recs,                    //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  AppendLapicRecord(&recs, 0x77, 0, 1);
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  ASSERT_TRUE(SmpParseMadt(madt_addr, kMaxCanonicalIdentityAddress, 0, &topo));
  EXPECT_THAT(topo.cpu_count, t::Eq(kMaxCpus));
  EXPECT_THAT(topo.cpus[0].apic_id, t::Eq(0));
  EXPECT_THAT(topo.cpus[0].acpi_processor_id, t::Eq(0x77));
  EXPECT_TRUE(topo.cpus[0].is_bsp);
}

TEST(SmpTest, DiscoverTopologyAndSmpInitEndToEndAndPanicsOnFailure) {
  ResetState(BaseAddr());
  const uintptr_t mb_base = OffsetAddr(0x6000);
  const uintptr_t rsdt_addr = OffsetAddr(0x6100);
  const uintptr_t xsdt_addr = OffsetAddr(0x6200);
  const uintptr_t bad_rsdt_addr = OffsetAddr(0x6300);
  const uintptr_t madt_addr = OffsetAddr(0x6400);
  const uintptr_t bios_rom_base = OffsetAddr(0x7000);

  std::vector<uint8_t> recs;
  for (int i = 0; i < 4; ++i) {
    AppendLapicRecord(&recs,                    //
                      static_cast<uint8_t>(i),  //
                      static_cast<uint8_t>(i),  //
                      1);
  }
  WriteMadt(madt_addr, kDefaultLocalApicPhysAddr, recs);
  WriteXsdt(xsdt_addr, {madt_addr});
  WriteRsdt(rsdt_addr, {static_cast<uint32_t>(madt_addr)});
  WriteRsdt(bad_rsdt_addr, {});

  // Build Multiboot2 structure with Tag 15 (ACPI_NEW) and Tag 14 (ACPI_OLD).
  uint8_t* const mb_buf = reinterpret_cast<uint8_t*>(mb_base);
  *reinterpret_cast<uint32_t*>(mb_buf) = 96;
  *reinterpret_cast<uint32_t*>(mb_buf + 4) = 0;
  *reinterpret_cast<uint32_t*>(mb_buf + 8) = 15;
  *reinterpret_cast<uint32_t*>(mb_buf + 12) = 8 + sizeof(RawRsdpV2);
  WriteRsdpV2(mb_base + 16, 0, xsdt_addr);
  *reinterpret_cast<uint32_t*>(mb_buf + 56) = 14;
  *reinterpret_cast<uint32_t*>(mb_buf + 60) = 8 + sizeof(RawRsdpV1);
  WriteRsdpV1(mb_base + 64, static_cast<uint32_t>(rsdt_addr));
  *reinterpret_cast<uint32_t*>(mb_buf + 88) = 0;
  *reinterpret_cast<uint32_t*>(mb_buf + 92) = 8;

  // Place a stale RSDP v1 pointing to an empty RSDT at bios_rom_base + 64,
  // followed by the valid RSDP v1 at bios_rom_base + 128, to verify that
  // SmpDiscoverTopology continues scanning the BIOS ROM range past a stale
  // RSDP whose SDT lacks a MADT.
  WriteRsdpV1(bios_rom_base + 64, static_cast<uint32_t>(bad_rsdt_addr));
  WriteRsdpV1(bios_rom_base + 128, static_cast<uint32_t>(rsdt_addr));
  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);

  // Verify SmpInit succeeds via Multiboot2 Tag 15 and brings all 4 CPUs online.
  SmpInit(kMultiboot2Magic, mb_base);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpLocalApicPhysAddr(), t::Eq(kDefaultLocalApicPhysAddr));
  EXPECT_THAT(g_state.pmm_alloc_calls, t::Eq(3));

  // If Tag 15's XSDT is corrupted and Tag 15 has rsdt_address == 0,
  // SmpDiscoverTopology must fall back to Multiboot2 Tag 14 (ACPI_OLD).
  reinterpret_cast<RawSdtHeader*>(xsdt_addr)->checksum ^= 0xFF;
  SmpInit(kMultiboot2Magic, mb_base);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));
  reinterpret_cast<RawSdtHeader*>(xsdt_addr)->checksum ^= 0xFF;

  for (int i = 0; i < 4; ++i) {
    const CpuInfo* const info = SmpGetCpuInfo(i);
    ASSERT_THAT(info, t::NotNull());
    EXPECT_THAT(info->apic_id, t::Eq(i));
    EXPECT_THAT(info->observed_apic_id, t::Eq(i));
    EXPECT_THAT(info->is_bsp, t::Eq(i == 0));
    EXPECT_TRUE(info->online);
    EXPECT_TRUE(info->long_mode_active);
    EXPECT_THAT(info->stack_top - info->stack_base, t::Eq(kApStackSize));
  }
  EXPECT_DEATH(SmpGetCpuInfo(-1), "Check failed");
  EXPECT_DEATH(SmpGetCpuInfo(4), "Check failed");

  // Verify SmpInit also succeeds via Legacy BIOS ROM scan (skipping the stale
  // RSDP at +64 and discovering the valid RSDP v1 -> RSDT at +128) under
  // Multiboot1.
  SmpInit(kMultiboot1Magic, 0);
  EXPECT_THAT(SmpCpuCount(), t::Eq(4));
  EXPECT_THAT(SmpOnlineCpuCount(), t::Eq(4));

  // Verify SmpInit panics via CHECK if topology discovery fails, if mapping
  // the Local APIC MMIO page fails, if PMM stack allocation fails, or if an AP
  // fails to come online.
  SmpSetHostTestHooks(0, 0, 0, BaseAddr(), nullptr);
  EXPECT_DEATH(SmpInit(kMultiboot1Magic, 0), "Check failed");

  SmpSetHostTestHooks(0, 0, bios_rom_base, BaseAddr(), nullptr);
  g_state.fail_map_addr = kDefaultLocalApicPhysAddr;
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, mb_base), "Check failed");
  g_state.fail_map_addr = 0;

  g_state.pmm_alloc_calls = 0;
  g_state.fail_pmm_on_call = 2;
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, mb_base), "Check failed");
  g_state.fail_pmm_on_call = -1;

  SmpSetHostTestHooks(0,              //
                      0,              //
                      bios_rom_base,  //
                      BaseAddr(),     //
                      [](const int cpu_index, CpuInfo* const cpu_info) {
                        if (cpu_index == 2) {
                          return false;
                        }
                        cpu_info->observed_apic_id = cpu_info->apic_id;
                        cpu_info->observed_rsp = cpu_info->stack_top - 64;
                        cpu_info->observed_cr3 = 0x105000;
                        cpu_info->long_mode_active = true;
                        cpu_info->online = true;
                        return true;
                      });
  EXPECT_DEATH(SmpInit(kMultiboot2Magic, mb_base), "Check failed");
}

TEST(SmpTest, PreconditionViolationsTriggerDcheck) {
  ResetState(BaseAddr());
  const uintptr_t addr = OffsetAddr(0x100);
  uintptr_t rsdt = 0;
  uintptr_t xsdt = 0;
  SmpTopology topo = {};

  EXPECT_DEATH(SmpFindRsdpInMultiboot2(0, kMaxCanonicalIdentityAddress),
               "Check failed");
  EXPECT_DEATH(SmpFindRsdpInMultiboot2(addr, 0), "Check failed");

  EXPECT_DEATH(SmpFindRsdpInMemoryRange(0, 1024), "Check failed");
  EXPECT_DEATH(SmpFindRsdpInMemoryRange(addr, 0), "Check failed");
  EXPECT_DEATH(SmpFindRsdpInMemoryRange(addr, -16), "Check failed");

  EXPECT_DEATH(SmpParseRsdp(0, kMaxCanonicalIdentityAddress, &rsdt, &xsdt),
               "Check failed");
  EXPECT_DEATH(SmpParseRsdp(addr, 0, &rsdt, &xsdt), "Check failed");
  EXPECT_DEATH(SmpParseRsdp(addr, kMaxCanonicalIdentityAddress, nullptr, &xsdt),
               "Check failed");
  EXPECT_DEATH(SmpParseRsdp(addr, kMaxCanonicalIdentityAddress, &rsdt, nullptr),
               "Check failed");

  EXPECT_DEATH(SmpFindMadtInSdt(0, 0, kMaxCanonicalIdentityAddress),
               "Check failed");
  EXPECT_DEATH(SmpFindMadtInSdt(addr, 0, 0), "Check failed");

  EXPECT_DEATH(SmpParseMadt(0, kMaxCanonicalIdentityAddress, 0, &topo),
               "Check failed");
  EXPECT_DEATH(SmpParseMadt(addr, 0, 0, &topo), "Check failed");
  EXPECT_DEATH(SmpParseMadt(addr, kMaxCanonicalIdentityAddress, 0xFF, &topo),
               "Check failed");
  EXPECT_DEATH(SmpParseMadt(addr, kMaxCanonicalIdentityAddress, 0, nullptr),
               "Check failed");

  EXPECT_DEATH(SmpDiscoverTopology(kMultiboot2Magic, 0, 0, 0, 0, 0, &topo),
               "Check failed");
  EXPECT_DEATH(SmpDiscoverTopology(kMultiboot2Magic,              //
                                   0,                             //
                                   0,                             //
                                   0,                             //
                                   kMaxCanonicalIdentityAddress,  //
                                   0xFF,                          //
                                   &topo),
               "Check failed");
  EXPECT_DEATH(SmpDiscoverTopology(kMultiboot2Magic,              //
                                   0,                             //
                                   0,                             //
                                   0,                             //
                                   kMaxCanonicalIdentityAddress,  //
                                   0,                             //
                                   nullptr),
               "Check failed");

  // Calling SMP query accessors before SmpInit succeeds must trigger DCHECK.
  EXPECT_DEATH(SmpCpuCount(), "Check failed");
  EXPECT_DEATH(SmpOnlineCpuCount(), "Check failed");
  EXPECT_DEATH(SmpGetCpuInfo(0), "Check failed");
  EXPECT_DEATH(SmpLocalApicPhysAddr(), "Check failed");
}

}  // namespace
}  // namespace protos
