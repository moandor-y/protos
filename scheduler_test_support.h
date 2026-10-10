#ifndef PROTOS_SCHEDULER_TEST_SUPPORT_H_
#define PROTOS_SCHEDULER_TEST_SUPPORT_H_

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>

#include "check.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "smp.h"
#include "spinlock.h"
#include "task.h"
#include "uart.h"
#include "vga.h"

namespace protos {

constexpr int64_t kInitialFakeFreeFrames = 16384;
constexpr int64_t kInitialFakeHeapBytes = 4 * 1024 * 1024;
constexpr uintptr_t kFakeStackBase = 0x200000;

struct FakeSchedulerEnv {
  int cpu_count = 1;
  bool idt_initialized = true;
  std::unique_ptr<CpuLocal[]> cpu_locals;
  std::unique_ptr<CpuInfo[]> cpu_infos;
  std::unique_ptr<bool[]> null_cpu_local;

  std::atomic<int64_t> allocated_frames{0};
  std::atomic<int64_t> allocated_heap_bytes{0};
  std::atomic<int64_t> allocated_heap_blocks{0};

  std::atomic<int64_t> eoi_count{0};
  std::unique_ptr<std::atomic<int64_t>[]> ipi_sent_count;
  std::unique_ptr<std::atomic<uint8_t>[]> last_ipi_vector;

  std::shared_ptr<TaskScheduler> active_scheduler;

  std::mutex log_mutex;
  std::string uart_log;
  std::string vga_log;
  std::string uart_panic_log;
  std::string vga_panic_log;
};

inline FakeSchedulerEnv g_env;

struct HeapAllocHeader {
  int64_t payload_size = 0;
  void* raw_base = nullptr;
  uint64_t magic = 0;
};
constexpr uint64_t kHeapHeaderMagic = 0xC0DEC0DE12345678ULL;

[[gnu::weak, gnu::noinline, gnu::noipa]] void TaskResetForTest() {}
[[gnu::weak, gnu::noinline, gnu::noipa]] void IdtResetForTest() {}
[[gnu::weak, gnu::noinline, gnu::noipa]] void ResetRdrandHookForTest() {}

[[gnu::weak, gnu::noinline, gnu::noipa]] bool IdtIsInitialized() {
  return g_env.idt_initialized;
}

[[gnu::weak, gnu::noinline, gnu::noipa]] const std::shared_ptr<TaskScheduler>&
GetTaskScheduler() {
  return g_env.active_scheduler;
}

inline void SetupEnv(const int num_cpus = 1) {
  CHECK(num_cpus >= 1);
  TaskResetForTest();
  IdtResetForTest();
  ResetCpuLocalForTest();
  ResetRdrandHookForTest();
  SetInterruptsEnabledForTest(true);

  g_env.active_scheduler.reset();
  g_env.idt_initialized = true;
  g_env.cpu_count = num_cpus;
  g_env.allocated_frames.store(0, std::memory_order_relaxed);
  g_env.allocated_heap_bytes.store(0, std::memory_order_relaxed);
  g_env.allocated_heap_blocks.store(0, std::memory_order_relaxed);
  g_env.eoi_count.store(0, std::memory_order_relaxed);
  {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    g_env.uart_log.clear();
    g_env.vga_log.clear();
    g_env.uart_panic_log.clear();
    g_env.vga_panic_log.clear();
  }

  g_env.cpu_locals.reset(new CpuLocal[num_cpus]());
  g_env.cpu_infos.reset(new CpuInfo[num_cpus]());
  g_env.null_cpu_local.reset(new bool[num_cpus]());
  g_env.ipi_sent_count.reset(new std::atomic<int64_t>[num_cpus]());
  g_env.last_ipi_vector.reset(new std::atomic<uint8_t>[num_cpus]());

  for (int i = 0; i < num_cpus; ++i) {
    const uintptr_t stack_base =
        kFakeStackBase + static_cast<uintptr_t>(i) * kTaskStackSize;
    const uintptr_t stack_top = stack_base + kTaskStackSize;

    CpuLocal& local = g_env.cpu_locals[i];
    local.self = &local;
    local.cpu_id = i;
    local.apic_id = static_cast<uint8_t>(i);
    local.online = true;
    local.stack_base = stack_base;
    local.stack_top = stack_top;

    CpuInfo& info = g_env.cpu_infos[i];
    info.apic_id = static_cast<uint8_t>(i);
    info.acpi_processor_id = static_cast<uint8_t>(i);
    info.observed_apic_id = static_cast<uint8_t>(i);
    info.is_bsp = (i == 0);
    info.online = true;
    info.long_mode_active = true;
    info.stack_base = stack_base;
    info.stack_top = stack_top;
  }

  BindCpuLocal(&g_env.cpu_locals[0]);
}

inline void ResetTestEnv() { SetupEnv(1); }

inline void NoopTask(void* const /*arg*/) {}

class MockTaskScheduler : public TaskScheduler {
 public:
  MOCK_METHOD(void, Init, (), (override));
  MOCK_METHOD(Task*, CreateTask, (TaskFn, void*, int64_t), (override));
  MOCK_METHOD(Task*, CreateTaskOnCpu, (TaskFn, void*, int64_t, int),
              (override));
  MOCK_METHOD(void, Yield, (), (override));
  MOCK_METHOD(void, Join, (Task*), (override));
  MOCK_METHOD(int, ReapZombies, (), (override));
  MOCK_METHOD(bool, PollIdleCpu, (int), (override));
  MOCK_METHOD(int, RunqueueLoad, (int), (const, override));
  MOCK_METHOD(bool, StealTask, (int, int), (override));
  MOCK_METHOD(void, SetPreemptEnabled, (bool), (override));
  MOCK_METHOD(bool, IsPreemptEnabled, (), (const, override));
  MOCK_METHOD(void, OnTimerInterrupt, (InterruptFrame*), (override));
  MOCK_METHOD(void, OnWakeupIpi, (InterruptFrame*), (override));
  MOCK_METHOD(Task*, CurrentTask, (), (const, override));
};

[[gnu::used]] inline uintptr_t PmmAllocFrames(const int64_t count) {
  DCHECK(count > 0);
  const int64_t bytes = count * kPageSize;
  void* const mem = std::aligned_alloc(static_cast<size_t>(kPageSize),
                                       static_cast<size_t>(bytes));
  if (mem == nullptr) {
    return 0;
  }
  g_env.allocated_frames.fetch_add(count, std::memory_order_acq_rel);
  return reinterpret_cast<uintptr_t>(mem);
}

[[gnu::used]] inline uintptr_t PmmAllocFrame() { return PmmAllocFrames(1); }

[[gnu::used]] inline void PmmFreeFrames(const uintptr_t base_addr,
                                        const int64_t count) {
  DCHECK(base_addr != 0);
  DCHECK((base_addr & (kPageSize - 1)) == 0);
  DCHECK(count > 0);
  std::free(reinterpret_cast<void*>(base_addr));
  const int64_t prev =
      g_env.allocated_frames.fetch_sub(count, std::memory_order_acq_rel);
  DCHECK(prev >= count);
}

[[gnu::used]] inline void PmmFreeFrame(const uintptr_t frame_addr) {
  PmmFreeFrames(frame_addr, 1);
}

[[gnu::used]] inline int64_t PmmFreeFrameCount() {
  return kInitialFakeFreeFrames -
         g_env.allocated_frames.load(std::memory_order_acquire);
}

[[gnu::used]] inline void* KmallocAligned(const int64_t size,
                                          const int64_t alignment) {
  if (size < 0 || alignment < 1 || (alignment & (alignment - 1)) != 0 ||
      alignment > kPageSize) {
    return nullptr;
  }
  const int64_t payload = (size > 0) ? size : kHeapAlignment;
  const int64_t align =
      (alignment > kHeapAlignment) ? alignment : kHeapAlignment;
  const int64_t header_size = static_cast<int64_t>(sizeof(HeapAllocHeader));
  const int64_t header_offset = (header_size + align - 1) & ~(align - 1);
  const int64_t total_bytes =
      (header_offset + payload + align - 1) & ~(align - 1);

  void* const raw = std::aligned_alloc(static_cast<size_t>(align),
                                       static_cast<size_t>(total_bytes));
  if (raw == nullptr) {
    return nullptr;
  }
  uint8_t* const user_ptr = static_cast<uint8_t*>(raw) + header_offset;
  HeapAllocHeader* const hdr =
      reinterpret_cast<HeapAllocHeader*>(user_ptr - sizeof(HeapAllocHeader));
  hdr->payload_size = payload;
  hdr->raw_base = raw;
  hdr->magic = kHeapHeaderMagic;

  g_env.allocated_heap_bytes.fetch_add(payload, std::memory_order_acq_rel);
  g_env.allocated_heap_blocks.fetch_add(1, std::memory_order_acq_rel);
  return user_ptr;
}

[[gnu::used]] inline void* Kmalloc(const int64_t size) {
  return KmallocAligned(size, kHeapAlignment);
}

[[gnu::used]] inline void Kfree(void* const ptr) {
  if (ptr == nullptr) {
    return;
  }
  uint8_t* const user_ptr = static_cast<uint8_t*>(ptr);
  HeapAllocHeader* const hdr =
      reinterpret_cast<HeapAllocHeader*>(user_ptr - sizeof(HeapAllocHeader));
  DCHECK(hdr->magic == kHeapHeaderMagic);
  const int64_t payload = hdr->payload_size;
  void* const raw = hdr->raw_base;
  hdr->magic = 0;
  std::free(raw);

  const int64_t prev_bytes =
      g_env.allocated_heap_bytes.fetch_sub(payload, std::memory_order_acq_rel);
  const int64_t prev_blocks =
      g_env.allocated_heap_blocks.fetch_sub(1, std::memory_order_acq_rel);
  DCHECK(prev_bytes >= payload);
  DCHECK(prev_blocks >= 1);
}

[[gnu::used]] inline int64_t HeapTotalFreeBytes() {
  return kInitialFakeHeapBytes -
         g_env.allocated_heap_bytes.load(std::memory_order_acquire);
}

[[gnu::used]] inline int SmpCpuCount() { return g_env.cpu_count; }

[[gnu::used]] inline int SmpOnlineCpuCount() {
  int online = 0;
  for (int i = 0; i < g_env.cpu_count; ++i) {
    if (g_env.cpu_locals[i].online) {
      ++online;
    }
  }
  return online;
}

[[gnu::used]] inline const CpuInfo* SmpGetCpuInfo(const int index) {
  DCHECK(index >= 0 && index < g_env.cpu_count);
  return &g_env.cpu_infos[index];
}

[[gnu::used]] inline CpuLocal* SmpGetCpuLocal(const int cpu_index) {
  DCHECK(cpu_index >= 0 && cpu_index < g_env.cpu_count);
  if (g_env.null_cpu_local != nullptr && g_env.null_cpu_local[cpu_index]) {
    return nullptr;
  }
  return &g_env.cpu_locals[cpu_index];
}

[[gnu::used]] inline void SmpSendLocalApicEoi() {
  g_env.eoi_count.fetch_add(1, std::memory_order_acq_rel);
}

[[gnu::used]] inline void SmpSendIpi(const int target_cpu_index,
                                     const uint8_t vector) {
  DCHECK(target_cpu_index >= 0 && target_cpu_index < g_env.cpu_count);
  DCHECK(vector >= static_cast<uint8_t>(kCpuExceptionCount));
  g_env.ipi_sent_count[target_cpu_index].fetch_add(1,
                                                   std::memory_order_acq_rel);
  g_env.last_ipi_vector[target_cpu_index].store(vector,
                                                std::memory_order_release);
}

[[gnu::used]] inline void UartWrite(const char* const str) {
  if (str != nullptr) {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    g_env.uart_log.append(str);
  }
}

[[gnu::used]] inline void UartWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llX",     //
                static_cast<unsigned long long>(value));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.uart_log.append(buf);
}

[[gnu::used]] inline void UartWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.uart_log.append(buf);
}

[[gnu::used]] inline void VgaWrite(const char* const str) {
  if (str != nullptr) {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    g_env.vga_log.append(str);
  }
}

[[gnu::used]] inline void VgaWriteHex(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "0x%llX",     //
                static_cast<unsigned long long>(value));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.vga_log.append(buf);
}

[[gnu::used]] inline void VgaWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.vga_log.append(buf);
}

[[gnu::used]] inline void UartPanicWrite(const char* const str) {
  if (str != nullptr) {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    g_env.uart_panic_log.append(str);
  }
}

[[gnu::used]] inline void UartPanicWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.uart_panic_log.append(buf);
}

[[gnu::used]] inline void VgaPanicWrite(const char* const str) {
  if (str != nullptr) {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    g_env.vga_panic_log.append(str);
  }
}

[[gnu::used]] inline void VgaPanicWriteDec(const uint64_t value) {
  char buf[32];
  std::snprintf(buf,          //
                sizeof(buf),  //
                "%llu",       //
                static_cast<unsigned long long>(value));
  const std::lock_guard<std::mutex> lock(g_env.log_mutex);
  g_env.vga_panic_log.append(buf);
}

}  // namespace protos

#endif  // PROTOS_SCHEDULER_TEST_SUPPORT_H_
