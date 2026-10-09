#include "stress_test.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif
#if __STDC_HOSTED__
#include <thread>
#endif

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

namespace {

constexpr uint32_t kCpuidLeafFeatures = 1u;
constexpr uint32_t kCpuidEcxRdrandBit = 1u << 30;
constexpr int kRdrandMaxRetries = 10;

constexpr int kMaxStressWorkers = 64;
constexpr int kMaxStressRounds = 4096;
constexpr int kMaxLiveAllocsPerWorker = 8;
constexpr int kMaxHeapAllocBytesLimit = 16384;
constexpr int kMaxPmmAllocFramesLimit = 8;
constexpr int kMaxActiveDetachedChildren = 64;
constexpr int kPreemptProbeMaxSpins = 50000000;

#if !__STDC_HOSTED__
constexpr int kLapicRegIdOffset = 0x020;
constexpr int kLapicRegTimerInitCountOffset = 0x380;
#endif

constexpr uint64_t kHeapObjectMagic = 0x5354524553534F42ULL;  // "STRESSOB"
constexpr uint64_t kHeapObjectPoison = 0xDEAD535452455353ULL;

std::atomic<uint64_t> g_entropy_call_seq{1};
std::atomic<uint64_t> g_stress_pass_count{0};

#if __STDC_HOSTED__
std::atomic<bool> g_rdrand_hook_active{false};
std::atomic<bool> g_rdrand_force_supported{false};
std::atomic<RdrandTestHookFn> g_rdrand_hook_fn{nullptr};
#endif

#if !__STDC_HOSTED__
static void ProgramLocalApicTimerInitCount(const uint32_t init_count) {
  const uintptr_t lapic_base = SmpLocalApicPhysAddr();
  if (lapic_base == 0 || init_count == 0) {
    return;
  }
  volatile uint32_t* const reg = reinterpret_cast<volatile uint32_t*>(
      lapic_base + kLapicRegTimerInitCountOffset);
  *reg = init_count;
  const volatile uint32_t* const id_reg =
      reinterpret_cast<const volatile uint32_t*>(lapic_base +
                                                 kLapicRegIdOffset);
  (void)*id_reg;
}

static void ProgramLapicTimerOnCpuCallback(const int /*cpu_index*/,
                                           void* const context) {
  DCHECK(context != nullptr);
  const uint32_t target_count = *static_cast<const uint32_t*>(context);
  ProgramLocalApicTimerInitCount(target_count);
}
#endif

enum class StressAllocKind : uint8_t {
  kNone = 0,
  kKmalloc = 1,
  kKmallocAligned = 2,
  kCppNewDelete = 3,
  kPmmFrames = 4,
};

struct StressHeapObject {
  uint64_t magic = 0;
  uint64_t seed_tag = 0;
  int worker_id = 0;
  int round_created = 0;
  uint64_t payload[16] = {};
  uint64_t checksum = 0;

  static void* operator new(const size_t size) {
    return Kmalloc(static_cast<int64_t>(size));
  }

  static void operator delete(void* const ptr) noexcept { Kfree(ptr); }

  static void operator delete(void* const ptr, const size_t /*size*/) noexcept {
    Kfree(ptr);
  }
};

struct LiveAllocation {
  StressAllocKind kind = StressAllocKind::kNone;
  void* heap_ptr = nullptr;
  StressHeapObject* obj_ptr = nullptr;
  uintptr_t pmm_phys_addr = 0;
  int64_t byte_size = 0;
  int64_t alignment = 0;
  int64_t frame_count = 0;
  bool single_frame_api = false;
  uint64_t pattern_seed = 0;
};

struct StressSharedState {
  Task* bootstrap_task = nullptr;
  int cpu_count = 1;
  int num_workers = 1;
  int rounds_per_worker = 1;
  int max_live_allocs_per_worker = 1;
  int effective_max_heap_bytes = 256;
  int max_pmm_alloc_frames = 1;
  uint64_t master_seed = 1;

  std::atomic<uint64_t> cpu_participation_mask{0};
  std::atomic<int64_t> cooperative_yields{0};
  std::atomic<int64_t> preemptive_spin_bursts{0};
  std::atomic<int64_t> timer_preemptions_observed{0};
  std::atomic<int64_t> wakeup_ipis_sent{0};
  std::atomic<int64_t> task_migrations_observed{0};
  std::atomic<int64_t> child_tasks_joined{0};
  std::atomic<int64_t> child_tasks_detached{0};
  std::atomic<int64_t> detached_children_finished{0};
  std::atomic<int64_t> active_detached_children{0};
  std::atomic<int64_t> zombies_reaped{0};
  std::atomic<int64_t> kmalloc_ops{0};
  std::atomic<int64_t> kmalloc_aligned_ops{0};
  std::atomic<int64_t> cpp_new_delete_ops{0};
  std::atomic<int64_t> pmm_alloc_ops{0};
  std::atomic<int64_t> payload_verifications{0};
  std::atomic<int64_t> payload_corruptions{0};
  std::atomic<int64_t> stack_canary_corruptions{0};
  std::atomic<int> workers_completed{0};
  std::atomic<bool> invariant_failed{false};
};

struct DetachedChildContext {
  StressSharedState* shared = nullptr;
  uint64_t seed = 0;
  int initial_cpu = 0;
  bool allow_yield = false;
};

struct JoinedChildContext {
  StressSharedState* shared = nullptr;
  Task* parent_task = nullptr;
  uint64_t expected_token = 0;
  std::atomic<uint64_t> observed_token{0};
  int initial_cpu = 0;
  bool allow_yield = false;
};

struct PreemptProbeContext {
  StressSharedState* shared = nullptr;
  int initial_cpu = 0;
  std::atomic<bool> started{false};
};

struct CpuParticipationBarrierContext {
  StressSharedState* shared = nullptr;
  std::atomic<uint64_t>* barrier_mask = nullptr;
  uint64_t expected_cpu_mask = 0;
  int initial_cpu = 0;
};

struct StressWorkerContext {
  StressSharedState* shared = nullptr;
  Task* task = nullptr;
  int worker_index = 0;
  int initial_cpu = 0;
  uint64_t worker_seed = 0;
  LiveAllocation live_allocs[kMaxLiveAllocsPerWorker] = {};
  std::atomic<bool> done{false};
};

static uint8_t ComputePatternByte(const uint64_t pattern_seed,
                                  const int64_t index) {
  const uint64_t mixed =
      MixEntropy64(pattern_seed, static_cast<uint64_t>(index));
  return static_cast<uint8_t>((mixed ^ (mixed >> 24)) & 0xFFu);
}

static uint64_t ComputePatternWord(const uint64_t pattern_seed,
                                   const int64_t word_index) {
  return MixEntropy64(pattern_seed ^ 0xA5A5A5A55A5A5A5AULL,
                      static_cast<uint64_t>(word_index));
}

static void FillByteBufferPattern(uint8_t* const buf,       //
                                  const int64_t byte_size,  //
                                  const uint64_t pattern_seed) {
  DCHECK(buf != nullptr);
  DCHECK(byte_size >= 0);
  const int64_t word_count = byte_size / 8;
  uint64_t* const words = reinterpret_cast<uint64_t*>(buf);
  for (int64_t w = 0; w < word_count; ++w) {
    words[w] = ComputePatternWord(pattern_seed, w);
  }
  for (int64_t b = word_count * 8; b < byte_size; ++b) {
    buf[b] = ComputePatternByte(pattern_seed, b);
  }
}

static bool VerifyAndPoisonByteBuffer(uint8_t* const buf,       //
                                      const int64_t byte_size,  //
                                      const uint64_t pattern_seed) {
  DCHECK(buf != nullptr);
  DCHECK(byte_size >= 0);
  bool ok = true;
  const int64_t word_count = byte_size / 8;
  uint64_t* const words = reinterpret_cast<uint64_t*>(buf);
  for (int64_t w = 0; w < word_count; ++w) {
    if (words[w] != ComputePatternWord(pattern_seed, w)) {
      ok = false;
    }
    words[w] = 0xDEADBEEFCAFEBABEULL ^ static_cast<uint64_t>(w);
  }
  for (int64_t b = word_count * 8; b < byte_size; ++b) {
    if (buf[b] != ComputePatternByte(pattern_seed, b)) {
      ok = false;
    }
    buf[b] = 0x5Au;
  }
  return ok;
}

static void FillFrameRunPattern(const uintptr_t phys_addr,  //
                                const int64_t frame_count,  //
                                const uint64_t pattern_seed) {
  DCHECK(phys_addr != 0);
  DCHECK(frame_count > 0);
  uint64_t* const words = reinterpret_cast<uint64_t*>(phys_addr);
  const int64_t total_words =
      (frame_count * kPageSize) / static_cast<int64_t>(sizeof(uint64_t));
  // Fill every word of the allocated physical frame run.
  for (int64_t w = 0; w < total_words; ++w) {
    words[w] = ComputePatternWord(pattern_seed, w);
  }
}

static bool VerifyAndPoisonFrameRun(const uintptr_t phys_addr,  //
                                    const int64_t frame_count,  //
                                    const uint64_t pattern_seed) {
  DCHECK(phys_addr != 0);
  DCHECK(frame_count > 0);
  uint64_t* const words = reinterpret_cast<uint64_t*>(phys_addr);
  const int64_t total_words =
      (frame_count * kPageSize) / static_cast<int64_t>(sizeof(uint64_t));
  bool ok = true;
  for (int64_t w = 0; w < total_words; ++w) {
    if (words[w] != ComputePatternWord(pattern_seed, w)) {
      ok = false;
    }
    words[w] = 0xBADF00D0DEADCAFEULL ^ static_cast<uint64_t>(w);
  }
  return ok;
}

static void VerifyAndReleaseSlot(StressSharedState* const shared,
                                 LiveAllocation* const slot) {
  DCHECK(shared != nullptr);
  DCHECK(slot != nullptr);
  if (slot->kind == StressAllocKind::kNone) {
    return;
  }

  bool ok = true;
  switch (slot->kind) {
    case StressAllocKind::kKmalloc: {
      DCHECK(slot->heap_ptr != nullptr);
      const int64_t effective_bytes =
          (slot->byte_size > 0) ? slot->byte_size : kHeapAlignment;
      ok = VerifyAndPoisonByteBuffer(static_cast<uint8_t*>(slot->heap_ptr),  //
                                     effective_bytes,                        //
                                     slot->pattern_seed);
      Kfree(slot->heap_ptr);
      break;
    }
    case StressAllocKind::kKmallocAligned: {
      DCHECK(slot->heap_ptr != nullptr);
      const uintptr_t eff_align = static_cast<uintptr_t>(
          (slot->alignment > kHeapAlignment) ? slot->alignment
                                             : kHeapAlignment);
      if ((reinterpret_cast<uintptr_t>(slot->heap_ptr) & (eff_align - 1u)) !=
          0) {
        ok = false;
      }
      const int64_t effective_bytes =
          (slot->byte_size > 0) ? slot->byte_size : kHeapAlignment;
      if (!VerifyAndPoisonByteBuffer(static_cast<uint8_t*>(slot->heap_ptr),  //
                                     effective_bytes,                        //
                                     slot->pattern_seed)) {
        ok = false;
      }
      Kfree(slot->heap_ptr);
      break;
    }
    case StressAllocKind::kCppNewDelete: {
      StressHeapObject* const obj = slot->obj_ptr;
      DCHECK(obj != nullptr);
      uint64_t expected_checksum =
          obj->seed_tag ^ kHeapObjectMagic ^
          static_cast<uint64_t>(obj->worker_id) ^
          (static_cast<uint64_t>(obj->round_created) << 32);
      for (int i = 0; i < 16; ++i) {
        const uint64_t expected_word =
            ComputePatternWord(slot->pattern_seed, i);
        if (obj->payload[i] != expected_word) {
          ok = false;
        }
        expected_checksum = MixEntropy64(expected_checksum, expected_word);
        obj->payload[i] = kHeapObjectPoison;
      }
      if (obj->magic != kHeapObjectMagic ||
          obj->seed_tag != slot->pattern_seed ||
          obj->checksum != expected_checksum) {
        ok = false;
      }
      obj->magic = kHeapObjectPoison;
      obj->checksum = 0;
      delete obj;
      break;
    }
    case StressAllocKind::kPmmFrames: {
      DCHECK(slot->pmm_phys_addr != 0);
      DCHECK(slot->frame_count > 0);
      if ((slot->pmm_phys_addr & static_cast<uintptr_t>(kPageSize - 1)) != 0) {
        ok = false;
      }
      if (!VerifyAndPoisonFrameRun(slot->pmm_phys_addr,  //
                                   slot->frame_count,    //
                                   slot->pattern_seed)) {
        ok = false;
      }
      if (slot->single_frame_api) {
        DCHECK(slot->frame_count == 1);
        PmmFreeFrame(slot->pmm_phys_addr);
      } else {
        PmmFreeFrames(slot->pmm_phys_addr, slot->frame_count);
      }
      break;
    }
    case StressAllocKind::kNone:
      break;
  }

  shared->payload_verifications.fetch_add(1, std::memory_order_acq_rel);
  if (!ok) {
    shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
  }
  *slot = LiveAllocation{};
}

static void AllocateIntoSlot(StressSharedState* const shared,  //
                             LiveAllocation* const slot,       //
                             Prng64* const prng,               //
                             const int worker_id,              //
                             const int round,                  //
                             const int alloc_kind_index) {
  DCHECK(shared != nullptr);
  DCHECK(slot != nullptr);
  DCHECK(prng != nullptr);
  DCHECK(slot->kind == StressAllocKind::kNone);

  const uint64_t pattern_seed = MixEntropy64(
      prng->NextU64(), (static_cast<uint64_t>(worker_id + 1) << 32) ^
                           static_cast<uint64_t>(round + 1));

  switch (alloc_kind_index & 3) {
    case 0: {
      const int64_t byte_size =
          prng->NextChance(1, 16)
              ? 0
              : prng->NextRange(16, shared->effective_max_heap_bytes);
      void* const ptr = Kmalloc(byte_size);
      if (ptr == nullptr || (reinterpret_cast<uintptr_t>(ptr) &
                             static_cast<uintptr_t>(kHeapAlignment - 1)) != 0) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        if (ptr != nullptr) {
          Kfree(ptr);
        }
        return;
      }
      const int64_t effective_bytes =
          (byte_size > 0) ? byte_size : kHeapAlignment;
      FillByteBufferPattern(static_cast<uint8_t*>(ptr),  //
                            effective_bytes,             //
                            pattern_seed);
      slot->kind = StressAllocKind::kKmalloc;
      slot->heap_ptr = ptr;
      slot->byte_size = byte_size;
      slot->alignment = kHeapAlignment;
      slot->pattern_seed = pattern_seed;
      shared->kmalloc_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
    case 1: {
      const int64_t byte_size =
          prng->NextRange(16, shared->effective_max_heap_bytes);
      const int64_t alignment = prng->NextPowerOfTwoAlignment(1, kPageSize);
      void* const ptr = KmallocAligned(byte_size, alignment);
      const uintptr_t eff_align = static_cast<uintptr_t>(
          (alignment > kHeapAlignment) ? alignment : kHeapAlignment);
      if (ptr == nullptr ||
          (reinterpret_cast<uintptr_t>(ptr) & (eff_align - 1u)) != 0) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        if (ptr != nullptr) {
          Kfree(ptr);
        }
        return;
      }
      FillByteBufferPattern(static_cast<uint8_t*>(ptr),  //
                            byte_size,                   //
                            pattern_seed);
      slot->kind = StressAllocKind::kKmallocAligned;
      slot->heap_ptr = ptr;
      slot->byte_size = byte_size;
      slot->alignment = alignment;
      slot->pattern_seed = pattern_seed;
      shared->kmalloc_aligned_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
    case 2: {
      StressHeapObject* const obj = new StressHeapObject();
      if (obj == nullptr) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        return;
      }
      obj->magic = kHeapObjectMagic;
      obj->seed_tag = pattern_seed;
      obj->worker_id = worker_id;
      obj->round_created = round;
      uint64_t checksum = pattern_seed ^ kHeapObjectMagic ^
                          static_cast<uint64_t>(worker_id) ^
                          (static_cast<uint64_t>(round) << 32);
      for (int i = 0; i < 16; ++i) {
        const uint64_t word = ComputePatternWord(pattern_seed, i);
        obj->payload[i] = word;
        checksum = MixEntropy64(checksum, word);
      }
      obj->checksum = checksum;
      slot->kind = StressAllocKind::kCppNewDelete;
      slot->obj_ptr = obj;
      slot->byte_size = static_cast<int64_t>(sizeof(StressHeapObject));
      slot->pattern_seed = pattern_seed;
      shared->cpp_new_delete_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
    case 3: {
      const bool single_frame = prng->NextBool();
      const int64_t frame_count =
          single_frame ? 1 : prng->NextRange(1, shared->max_pmm_alloc_frames);
      const uintptr_t phys_addr =
          single_frame ? PmmAllocFrame() : PmmAllocFrames(frame_count);
      if (phys_addr == 0 ||
          (phys_addr & static_cast<uintptr_t>(kPageSize - 1)) != 0) {
        shared->payload_corruptions.fetch_add(1, std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        if (phys_addr != 0) {
          PmmFreeFrames(phys_addr, frame_count);
        }
        return;
      }
      FillFrameRunPattern(phys_addr, frame_count, pattern_seed);
      slot->kind = StressAllocKind::kPmmFrames;
      slot->pmm_phys_addr = phys_addr;
      slot->frame_count = frame_count;
      slot->single_frame_api = single_frame;
      slot->pattern_seed = pattern_seed;
      shared->pmm_alloc_ops.fetch_add(1, std::memory_order_acq_rel);
      break;
    }
  }
}

static void RecordCpuAndMigration(StressSharedState* const shared,
                                  int* const last_cpu) {
  DCHECK(shared != nullptr);
  DCHECK(last_cpu != nullptr);
  const int cpu = CurrentCpuId();
  if (cpu >= 0 && cpu < 64) {
    shared->cpu_participation_mask.fetch_or(1ULL << static_cast<uint32_t>(cpu),
                                            std::memory_order_acq_rel);
  }
  if (cpu != *last_cpu) {
    shared->task_migrations_observed.fetch_add(1, std::memory_order_acq_rel);
    *last_cpu = cpu;
  }
}

static void PreemptProbeChildEntry(void* const raw_arg) {
  PreemptProbeContext* const probe = static_cast<PreemptProbeContext*>(raw_arg);
  if (probe == nullptr || probe->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = probe->shared;
  int last_cpu = probe->initial_cpu;
  RecordCpuAndMigration(shared, &last_cpu);
  shared->timer_preemptions_observed.fetch_add(1, std::memory_order_acq_rel);
  probe->started.store(true, std::memory_order_release);
}

static void CpuParticipationBarrierEntry(void* const raw_arg) {
  CpuParticipationBarrierContext* const ctx =
      static_cast<CpuParticipationBarrierContext*>(raw_arg);
  if (ctx == nullptr || ctx->shared == nullptr ||
      ctx->barrier_mask == nullptr) {
    return;
  }
  StressSharedState* const shared = ctx->shared;
  int last_cpu = ctx->initial_cpu;
  RecordCpuAndMigration(shared, &last_cpu);
  if (last_cpu >= 0 && last_cpu < 64) {
    ctx->barrier_mask->fetch_or(1ULL << static_cast<uint32_t>(last_cpu),
                                std::memory_order_acq_rel);
  }
  while ((ctx->barrier_mask->load(std::memory_order_acquire) &
          ctx->expected_cpu_mask) != ctx->expected_cpu_mask) {
    asm volatile("pause" : : : "memory");
#if __STDC_HOSTED__
    std::this_thread::yield();
#endif
  }
}

static void JoinedChildEntry(void* const raw_arg) {
  JoinedChildContext* const arg = static_cast<JoinedChildContext*>(raw_arg);
  if (arg == nullptr || arg->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = arg->shared;
  const std::shared_ptr<TaskScheduler>& scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();

  alignas(16) volatile uint64_t child_canaries[4] = {
      arg->expected_token ^ 0x1111111111111111ULL,
      arg->expected_token ^ 0x2222222222222222ULL,
      arg->expected_token ^ 0x4444444444444444ULL,
      arg->expected_token ^ 0x8888888888888888ULL,
  };
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&child_canaries[0]));
  if (self == nullptr || self == shared->bootstrap_task ||
      self == arg->parent_task || (canary_addr & 0xFu) != 0 ||
      !AreInterruptsEnabled()) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    return;
  }

  int last_cpu = arg->initial_cpu;
  RecordCpuAndMigration(shared, &last_cpu);

  if (arg->allow_yield) {
    scheduler->Yield();
    shared->cooperative_yields.fetch_add(1, std::memory_order_acq_rel);
    RecordCpuAndMigration(shared, &last_cpu);
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
      return;
    }
  }

  if (child_canaries[0] != (arg->expected_token ^ 0x1111111111111111ULL) ||
      child_canaries[1] != (arg->expected_token ^ 0x2222222222222222ULL) ||
      child_canaries[2] != (arg->expected_token ^ 0x4444444444444444ULL) ||
      child_canaries[3] != (arg->expected_token ^ 0x8888888888888888ULL)) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    return;
  }

  arg->observed_token.store(arg->expected_token, std::memory_order_release);
}

static void DetachedChildEntry(void* const raw_arg) {
  std::unique_ptr<DetachedChildContext> arg(
      static_cast<DetachedChildContext*>(raw_arg));
  if (arg == nullptr || arg->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = arg->shared;
  const std::shared_ptr<TaskScheduler>& scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();

  const uint64_t seed = arg->seed;
  const int initial_cpu = arg->initial_cpu;
  const bool allow_yield = arg->allow_yield;
  arg.reset();

  alignas(16) volatile uint64_t child_canaries[4] = {
      seed ^ 0x13579BDF2468ACE0ULL,
      seed ^ 0x2468ACE013579BDFULL,
      seed ^ 0xFEDCBA9876543210ULL,
      seed ^ 0x0123456789ABCDEFULL,
  };
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&child_canaries[0]));
  if (self == nullptr || self == shared->bootstrap_task ||
      (canary_addr & 0xFu) != 0 || !AreInterruptsEnabled()) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    shared->active_detached_children.fetch_sub(1, std::memory_order_acq_rel);
    shared->detached_children_finished.fetch_add(1, std::memory_order_acq_rel);
    return;
  }

  int last_cpu = initial_cpu;
  RecordCpuAndMigration(shared, &last_cpu);

  if (allow_yield) {
    scheduler->Yield();
    shared->cooperative_yields.fetch_add(1, std::memory_order_acq_rel);
    RecordCpuAndMigration(shared, &last_cpu);
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
    }
  }

  if (child_canaries[0] != (seed ^ 0x13579BDF2468ACE0ULL) ||
      child_canaries[1] != (seed ^ 0x2468ACE013579BDFULL) ||
      child_canaries[2] != (seed ^ 0xFEDCBA9876543210ULL) ||
      child_canaries[3] != (seed ^ 0x0123456789ABCDEFULL)) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
  }

  shared->active_detached_children.fetch_sub(1, std::memory_order_acq_rel);
  shared->detached_children_finished.fetch_add(1, std::memory_order_acq_rel);
}

static int64_t SelectRandomChildWeight(Prng64* const prng,
                                       const int child_seq) {
  DCHECK(prng != nullptr);
  // Explicitly exercise exact boundary weights kMinTaskWeight and
  // kMaxTaskWeight as well as random interior weights across
  // [kMinTaskWeight, kMaxTaskWeight].
  if (child_seq == 0) {
    return kMinTaskWeight;
  }
  if (child_seq == 1) {
    return kMaxTaskWeight;
  }
  if (prng->NextBool()) {
    return prng->NextRange(kDefaultTaskWeight / 4, kDefaultTaskWeight * 4);
  }
  return prng->NextRange(kMinTaskWeight, kMaxTaskWeight);
}

static void RunNonYieldingSpinBurst(StressSharedState* const shared,  //
                                    Prng64* const prng,               //
                                    int* const last_cpu) {
  DCHECK(shared != nullptr);
  DCHECK(prng != nullptr);
  DCHECK(last_cpu != nullptr);

  shared->preemptive_spin_bursts.fetch_add(1, std::memory_order_acq_rel);
  const int cpu_before = CurrentCpuId();
  const int64_t ticks_before =
      CurrentCpu()->timer_ticks.load(std::memory_order_relaxed);
  const int spin_iters = prng->NextIntRange(128, 640);

  uint64_t accumulator = prng->NextU64();
  for (int s = 0; s < spin_iters; ++s) {
    accumulator = MixEntropy64(accumulator, static_cast<uint64_t>(s));
    asm volatile("pause" : "+r"(accumulator) : : "memory");
#if __STDC_HOSTED__
    if (s == (spin_iters / 2) && IdtIsInitialized() && AreInterruptsEnabled()) {
      InterruptFrame timer_frame = {};
      timer_frame.vector = kVectorApicTimer;
      timer_frame.rflags = internal::kRflagsInterruptEnableBit;
      IdtDispatch(&timer_frame);
    }
#endif
  }

  RecordCpuAndMigration(shared, last_cpu);
  const int cpu_after = CurrentCpuId();
  const int64_t ticks_after =
      CurrentCpu()->timer_ticks.load(std::memory_order_relaxed);
  if (ticks_after > ticks_before) {
    shared->timer_preemptions_observed.fetch_add(ticks_after - ticks_before,
                                                 std::memory_order_acq_rel);
  } else if (cpu_after != cpu_before) {
    shared->timer_preemptions_observed.fetch_add(1, std::memory_order_acq_rel);
  }
}

static void StressWorkerEntry(void* const raw_arg) {
  StressWorkerContext* const worker =
      static_cast<StressWorkerContext*>(raw_arg);
  if (worker == nullptr || worker->shared == nullptr) {
    return;
  }
  StressSharedState* const shared = worker->shared;
  const std::shared_ptr<TaskScheduler>& scheduler = GetTaskScheduler();
  Task* const self = scheduler->CurrentTask();

  alignas(16) volatile uint64_t stack_canaries[8] = {};
  const uintptr_t canary_addr =
      reinterpret_cast<uintptr_t>(const_cast<uint64_t*>(&stack_canaries[0]));
  if (self == nullptr || self == shared->bootstrap_task ||
      (canary_addr & 0xFu) != 0 || !AreInterruptsEnabled()) {
    shared->stack_canary_corruptions.fetch_add(1, std::memory_order_acq_rel);
    shared->invariant_failed.store(true, std::memory_order_release);
    return;
  }

  Prng64 prng(worker->worker_seed);
  int last_cpu = worker->initial_cpu;
  RecordCpuAndMigration(shared, &last_cpu);

  int alloc_seq = 0;
  int child_seq = 0;
  bool spawned_joined_child = false;
  bool spawned_detached_child = false;

  // Worker 0 executes an explicit non-yielding spinner + probe child check on
  // round 0 to deterministically prove timer-driven preemption (`0x20`) of a
  // CPU-bound non-yielding task.
  if (worker->worker_index == 0) {
    PreemptProbeContext probe_ctx = {};
    probe_ctx.shared = shared;
    probe_ctx.initial_cpu = CurrentCpuId();
    Task* const probe_task =
        scheduler->CreateTaskOnCpu(PreemptProbeChildEntry,  //
                                   &probe_ctx,              //
                                   kDefaultTaskWeight,      //
                                   probe_ctx.initial_cpu);
    if (probe_task == nullptr) {
      shared->invariant_failed.store(true, std::memory_order_release);
      return;
    }
    shared->preemptive_spin_bursts.fetch_add(1, std::memory_order_acq_rel);
    for (int spin = 0; spin < kPreemptProbeMaxSpins &&
                       !probe_ctx.started.load(std::memory_order_acquire);
         ++spin) {
      asm volatile("pause" : : : "memory");
#if __STDC_HOSTED__
      if ((spin & 15) == 7 && IdtIsInitialized() && AreInterruptsEnabled()) {
        InterruptFrame timer_frame = {};
        timer_frame.vector = kVectorApicTimer;
        timer_frame.rflags = internal::kRflagsInterruptEnableBit;
        IdtDispatch(&timer_frame);
      }
#endif
    }
    scheduler->Join(probe_task);
    shared->child_tasks_joined.fetch_add(1, std::memory_order_acq_rel);
    RecordCpuAndMigration(shared, &last_cpu);
    if (!probe_ctx.started.load(std::memory_order_acquire)) {
      shared->invariant_failed.store(true, std::memory_order_release);
      return;
    }
  }

  for (int round = 0; round < shared->rounds_per_worker; ++round) {
    const uint64_t round_tag =
        MixEntropy64(worker->worker_seed, static_cast<uint64_t>(round + 1));
    for (int k = 0; k < 8; ++k) {
      stack_canaries[k] =
          round_tag ^ (static_cast<uint64_t>(k + 1) * 0x1111111111111111ULL);
    }

    RecordCpuAndMigration(shared, &last_cpu);
    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
      break;
    }

    // 1. Randomized PMM & Heap allocation / verification across slots.
    const int initial_allocs =
        (worker->worker_index == 0 && round == 0 && shared->num_workers < 4 &&
         shared->rounds_per_worker < 4)
            ? 4
            : 1;
    for (int a = 0; a < initial_allocs; ++a) {
      const int slot_idx =
          (alloc_seq < 4)
              ? (alloc_seq % shared->max_live_allocs_per_worker)
              : prng.NextIntRange(0, shared->max_live_allocs_per_worker - 1);
      LiveAllocation* const slot = &worker->live_allocs[slot_idx];
      if (slot->kind != StressAllocKind::kNone) {
        VerifyAndReleaseSlot(shared, slot);
      }
      const int kind_idx = (alloc_seq < 4)
                               ? ((worker->worker_index + alloc_seq) & 3)
                               : prng.NextIntRange(0, 3);
      ++alloc_seq;
      AllocateIntoSlot(shared,                //
                       slot,                  //
                       &prng,                 //
                       worker->worker_index,  //
                       round,                 //
                       kind_idx);
    }

    // 2. Cross-CPU wakeup/reschedule IPI (0x21).
    if (shared->cpu_count > 1 && (round == 0 || prng.NextChance(1, 2))) {
      const int cur_cpu = CurrentCpuId();
      const int offset = prng.NextIntRange(1, shared->cpu_count - 1);
      const int target_cpu = (cur_cpu + offset) % shared->cpu_count;
      SmpSendIpi(target_cpu, kVectorWakeupIpi);
      shared->wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
    }

    // 3. Randomly alternate cooperative Yield() and non-yielding CPU-bound
    //    spin bursts while holding live PMM/Heap allocations across switches.
    if (round == 0 || prng.NextBool()) {
      scheduler->Yield();
      shared->cooperative_yields.fetch_add(1, std::memory_order_acq_rel);
      RecordCpuAndMigration(shared, &last_cpu);
    }
    if (round == 0 || prng.NextChance(1, 2)) {
      RunNonYieldingSpinBurst(shared, &prng, &last_cpu);
    }

    if (scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
      shared->invariant_failed.store(true, std::memory_order_release);
      break;
    }

    // 4. Dynamic child task spawning (both joined and detached, using both
    //    CreateTask and CreateTaskOnCpu with randomized weights in
    //    [kMinTaskWeight, kMaxTaskWeight]).
    const bool do_joined = !spawned_joined_child || prng.NextChance(1, 4);
    if (do_joined) {
      JoinedChildContext child_ctx = {};
      child_ctx.shared = shared;
      child_ctx.parent_task = self;
      child_ctx.expected_token = round_tag ^ 0xA5A55A5AA5A55A5AULL;
      const int64_t child_weight = SelectRandomChildWeight(&prng, child_seq++);
      child_ctx.allow_yield = (child_weight >= 128);

      Task* child = nullptr;
      if (prng.NextBool()) {
        const int target_cpu = prng.NextIntRange(0, shared->cpu_count - 1);
        child_ctx.initial_cpu = target_cpu;
        child = scheduler->CreateTaskOnCpu(JoinedChildEntry,  //
                                           &child_ctx,        //
                                           child_weight,      //
                                           target_cpu);
      } else {
        child_ctx.initial_cpu = CurrentCpuId();
        child = scheduler->CreateTask(JoinedChildEntry,  //
                                      &child_ctx,        //
                                      child_weight);
      }
      if (child == nullptr) {
        shared->invariant_failed.store(true, std::memory_order_release);
        break;
      }
      scheduler->Join(child);
      spawned_joined_child = true;
      shared->child_tasks_joined.fetch_add(1, std::memory_order_acq_rel);
      RecordCpuAndMigration(shared, &last_cpu);
      if (child_ctx.observed_token.load(std::memory_order_acquire) !=
              child_ctx.expected_token ||
          scheduler->CurrentTask() != self || !AreInterruptsEnabled()) {
        shared->invariant_failed.store(true, std::memory_order_release);
        break;
      }
    }

    const bool do_detached =
        (!spawned_detached_child || prng.NextChance(1, 4)) &&
        (shared->child_tasks_detached.load(std::memory_order_acquire) <
         kMaxActiveDetachedChildren);
    if (do_detached) {
      const int64_t detached_idx =
          shared->child_tasks_detached.fetch_add(1, std::memory_order_acq_rel);
      if (detached_idx < kMaxActiveDetachedChildren) {
        std::unique_ptr<DetachedChildContext> det_ctx(
            new DetachedChildContext());
        if (det_ctx != nullptr) {
          det_ctx->shared = shared;
          det_ctx->seed = round_tag ^ prng.NextU64();
          const int64_t det_weight =
              SelectRandomChildWeight(&prng, child_seq++);
          det_ctx->allow_yield = (det_weight >= 128);
          shared->active_detached_children.fetch_add(1,
                                                     std::memory_order_acq_rel);

          Task* det_task = nullptr;
          if (prng.NextBool()) {
            const int target_cpu = prng.NextIntRange(0, shared->cpu_count - 1);
            det_ctx->initial_cpu = target_cpu;
            det_task = scheduler->CreateTaskOnCpu(DetachedChildEntry,  //
                                                  det_ctx.get(),       //
                                                  det_weight,          //
                                                  target_cpu);
          } else {
            det_ctx->initial_cpu = CurrentCpuId();
            det_task = scheduler->CreateTask(DetachedChildEntry,  //
                                             det_ctx.get(),       //
                                             det_weight);
          }
          if (det_task == nullptr) {
            shared->active_detached_children.fetch_sub(
                1, std::memory_order_acq_rel);
            shared->child_tasks_detached.fetch_sub(1,
                                                   std::memory_order_acq_rel);
            shared->invariant_failed.store(true, std::memory_order_release);
            break;
          }
          (void)det_ctx.release();
          spawned_detached_child = true;
        } else {
          shared->child_tasks_detached.fetch_sub(1, std::memory_order_acq_rel);
        }
      } else {
        shared->child_tasks_detached.fetch_sub(1, std::memory_order_acq_rel);
      }
    }

    // 5. Verify per-task stack canaries survived all switches and migrations.
    for (int k = 0; k < 8; ++k) {
      const uint64_t expected =
          round_tag ^ (static_cast<uint64_t>(k + 1) * 0x1111111111111111ULL);
      if (stack_canaries[k] != expected) {
        shared->stack_canary_corruptions.fetch_add(1,
                                                   std::memory_order_acq_rel);
        shared->invariant_failed.store(true, std::memory_order_release);
        break;
      }
    }
  }

  // Verify and release all remaining live allocations held by this worker.
  for (int i = 0; i < shared->max_live_allocs_per_worker; ++i) {
    if (worker->live_allocs[i].kind != StressAllocKind::kNone) {
      VerifyAndReleaseSlot(shared, &worker->live_allocs[i]);
    }
  }

  worker->done.store(true, std::memory_order_release);
  shared->workers_completed.fetch_add(1, std::memory_order_acq_rel);
}

}  // namespace

#if __STDC_HOSTED__
void SetRdrandHookForTest(const bool force_supported,
                          const RdrandTestHookFn hook_fn) {
  g_rdrand_force_supported.store(force_supported, std::memory_order_release);
  g_rdrand_hook_fn.store(hook_fn, std::memory_order_release);
  g_rdrand_hook_active.store(true, std::memory_order_release);
}

void ResetRdrandHookForTest() {
  g_rdrand_hook_active.store(false, std::memory_order_release);
  g_rdrand_force_supported.store(false, std::memory_order_release);
  g_rdrand_hook_fn.store(nullptr, std::memory_order_release);
}
#endif

bool CpuSupportsRdrand() {
#if __STDC_HOSTED__
  if (g_rdrand_hook_active.load(std::memory_order_acquire)) {
    return g_rdrand_force_supported.load(std::memory_order_acquire);
  }
#endif
  uint32_t eax = kCpuidLeafFeatures;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
  asm volatile("cpuid" : "+a"(eax), "=b"(ebx), "+c"(ecx), "=d"(edx) : : "cc");
  return (ecx & kCpuidEcxRdrandBit) != 0;
}

bool TryReadRdrand64(uint64_t* const out) {
  DCHECK(out != nullptr);
  if (!CpuSupportsRdrand()) {
    *out = 0;
    return false;
  }
  for (int attempt = 0; attempt < kRdrandMaxRetries; ++attempt) {
#if __STDC_HOSTED__
    if (g_rdrand_hook_active.load(std::memory_order_acquire)) {
      const RdrandTestHookFn hook_fn =
          g_rdrand_hook_fn.load(std::memory_order_acquire);
      if (hook_fn != nullptr) {
        uint64_t hooked_val = 0;
        if (hook_fn(&hooked_val)) {
          *out = hooked_val;
          return true;
        }
        continue;
      }
    }
#endif
    uint64_t val = 0;
    uint8_t ok = 0;
    asm volatile("rdrand %0" : "=r"(val), "=@ccc"(ok) : : "cc");
    if (ok != 0) {
      *out = val;
      return true;
    }
    asm volatile("pause" : : : "memory");
  }
  *out = 0;
  return false;
}

uint64_t ReadTsc64() {
  uint32_t eax = 0;
  uint32_t edx = 0;
  asm volatile("rdtsc" : "=a"(eax), "=d"(edx));
  return (static_cast<uint64_t>(edx) << 32) | static_cast<uint64_t>(eax);
}

uint64_t MixEntropy64(const uint64_t a, const uint64_t b) {
  uint64_t z = a ^ (b + 0x9E3779B97F4A7C15ULL + (a << 6) + (a >> 2));
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

uint64_t HarvestHardwareEntropySeed(const uint64_t extra_entropy) {
  const uint64_t tsc0 = ReadTsc64();
  uint64_t rdrand_val = 0;
  const bool has_rdrand = TryReadRdrand64(&rdrand_val);
  const uint64_t tsc1 = ReadTsc64();

  uint64_t cpu_entropy = 0xD1B54A32D192ED03ULL;
  const CpuLocal* const cpu = CurrentCpuOrNull();
  if (cpu != nullptr) {
    const uint64_t ticks =
        static_cast<uint64_t>(cpu->timer_ticks.load(std::memory_order_relaxed));
    const uint64_t ipis =
        static_cast<uint64_t>(cpu->ipi_count.load(std::memory_order_relaxed));
    cpu_entropy = MixEntropy64((static_cast<uint64_t>(cpu->cpu_id) << 32) |
                                   static_cast<uint64_t>(cpu->apic_id),
                               MixEntropy64(ticks, ipis));
  }

  const uint64_t seq =
      g_entropy_call_seq.fetch_add(1, std::memory_order_relaxed);
  uint64_t mixed = MixEntropy64(tsc0, tsc1 ^ (seq * 0x9E3779B97F4A7C15ULL));
  mixed =
      MixEntropy64(mixed, rdrand_val ^ (has_rdrand ? 0xA5A55A5AA5A55A5AULL
                                                   : 0x5A5AA5A55A5AA5A5ULL));
  mixed = MixEntropy64(mixed, cpu_entropy);
  mixed = MixEntropy64(mixed, extra_entropy);
  return (mixed != 0) ? mixed : 0x9E3779B97F4A7C15ULL;
}

uint64_t Prng64::NextU64() {
  uint64_t x = state_;
  if (x == 0) {
    x = 0x9E3779B97F4A7C15ULL;
  }
  x ^= x >> 12;
  x ^= x << 25;
  x ^= x >> 27;
  state_ = x;
  return x * 0x2545F4914F6CDD1DULL;
}

uint64_t Prng64::NextBounded(const uint64_t bound) {
  DCHECK(bound > 0);
  const uint64_t threshold = (0ULL - bound) % bound;
  for (;;) {
    const uint64_t r = NextU64();
    if (r >= threshold) {
      return r % bound;
    }
  }
}

int64_t Prng64::NextRange(const int64_t min_inclusive,
                          const int64_t max_inclusive) {
  DCHECK(min_inclusive <= max_inclusive);
  const uint64_t span = static_cast<uint64_t>(max_inclusive) -
                        static_cast<uint64_t>(min_inclusive);
  if (span == ~0ULL) {
    return static_cast<int64_t>(NextU64());
  }
  const uint64_t offset = NextBounded(span + 1ULL);
  return static_cast<int64_t>(static_cast<uint64_t>(min_inclusive) + offset);
}

int Prng64::NextIntRange(const int min_inclusive, const int max_inclusive) {
  DCHECK(min_inclusive <= max_inclusive);
  return static_cast<int>(NextRange(static_cast<int64_t>(min_inclusive),
                                    static_cast<int64_t>(max_inclusive)));
}

bool Prng64::NextBool() { return (NextU64() & 1ULL) != 0; }

bool Prng64::NextChance(const int numerator, const int denominator) {
  DCHECK(denominator > 0);
  DCHECK(numerator >= 0);
  DCHECK(numerator <= denominator);
  if (numerator == 0) {
    return false;
  }
  if (numerator == denominator) {
    return true;
  }
  return NextBounded(static_cast<uint64_t>(denominator)) <
         static_cast<uint64_t>(numerator);
}

int64_t Prng64::NextPowerOfTwoAlignment(const int64_t min_align,
                                        const int64_t max_align) {
  DCHECK(min_align > 0);
  DCHECK((min_align & (min_align - 1)) == 0);
  DCHECK(max_align >= min_align);
  DCHECK((max_align & (max_align - 1)) == 0);

  int min_exp = 0;
  while ((1LL << min_exp) < min_align) {
    ++min_exp;
  }
  int max_exp = min_exp;
  while ((1LL << max_exp) < max_align) {
    ++max_exp;
  }
  const int chosen_exp = NextIntRange(min_exp, max_exp);
  return 1LL << chosen_exp;
}

Prng64 Prng64::Fork(const uint64_t stream_id) {
  const uint64_t next_word = NextU64();
  return Prng64(MixEntropy64(next_word, stream_id ^ 0xD1B54A32D192ED03ULL));
}

StressTestStats ExecuteRandomMultiCpuStress(const StressTestConfig& config) {
  DCHECK(config.num_workers >= 0 && config.num_workers <= kMaxStressWorkers);
  DCHECK(config.rounds_per_worker > 0 &&
         config.rounds_per_worker <= kMaxStressRounds);
  DCHECK(config.max_live_allocs_per_worker > 0 &&
         config.max_live_allocs_per_worker <= kMaxLiveAllocsPerWorker);
  DCHECK(config.max_heap_alloc_bytes >= kHeapAlignment &&
         config.max_heap_alloc_bytes <= kMaxHeapAllocBytesLimit);
  DCHECK(config.max_pmm_alloc_frames >= 1 &&
         config.max_pmm_alloc_frames <= kMaxPmmAllocFramesLimit);

  const std::shared_ptr<TaskScheduler>& scheduler = GetTaskScheduler();
  CHECK(scheduler != nullptr);
  const int cpu_count = SmpCpuCount();
  CHECK(cpu_count >= 1 && cpu_count <= 64);
  CHECK(SmpOnlineCpuCount() == cpu_count);

  Task* const bootstrap_task = scheduler->CurrentTask();
  CHECK(bootstrap_task != nullptr);
  CHECK(CurrentCpuId() == 0);
  CHECK(scheduler->IsPreemptEnabled());
  CHECK(AreInterruptsEnabled());

  while (scheduler->ReapZombies() > 0) {
  }

  const int64_t pmm_free_before = PmmFreeFrameCount();
  const int64_t heap_free_before = HeapTotalFreeBytes();
  CHECK(pmm_free_before > 0);
  CHECK(heap_free_before > 0);

  const uint64_t master_seed =
      (config.seed != 0) ? config.seed
                         : HarvestHardwareEntropySeed(
                               static_cast<uint64_t>(pmm_free_before) ^
                               (static_cast<uint64_t>(heap_free_before) << 16));
  CHECK(master_seed != 0);

  if (config.log_to_console) {
    UartWrite("[STRESS] seed=");
    UartWriteHex(master_seed);
    UartWrite("\n");
  }

#if !__STDC_HOSTED__
  const uint32_t calibrated_timer_count = SmpCalibratedTimerInitialCount();
  const uint32_t stress_timer_count = (calibrated_timer_count > 32u)
                                          ? (calibrated_timer_count / 16u)
                                          : calibrated_timer_count;
  if (config.boost_lapic_timer && stress_timer_count > 0) {
    uint32_t boost_count = stress_timer_count;
    SmpRunOnAllCpus(ProgramLapicTimerOnCpuCallback, &boost_count);
  }
#endif

  const int num_workers =
      (config.num_workers > 0)
          ? config.num_workers
          : ((2 * cpu_count <= kMaxStressWorkers) ? (2 * cpu_count)
                                                  : kMaxStressWorkers);
  CHECK(num_workers >= 1 && num_workers <= kMaxStressWorkers);

  // Bound per-allocation heap bytes so peak simultaneous live heap allocations
  // across all workers and tasks remain <= 256 KiB and well below
  // `heap_free_before`, guaranteeing `HeapExpand()` is never triggered.
  const int total_live_slots = num_workers * config.max_live_allocs_per_worker;
  const int safe_slot_budget =
      (total_live_slots > 0) ? (131072 / total_live_slots) : 2048;
  int effective_max_heap_bytes = config.max_heap_alloc_bytes;
  if (effective_max_heap_bytes > safe_slot_budget) {
    effective_max_heap_bytes = safe_slot_budget;
  }
  if (effective_max_heap_bytes < kHeapAlignment) {
    effective_max_heap_bytes = static_cast<int>(kHeapAlignment);
  }

  StressSharedState shared = {};
  shared.bootstrap_task = bootstrap_task;
  shared.cpu_count = cpu_count;
  shared.num_workers = num_workers;
  shared.rounds_per_worker = config.rounds_per_worker;
  shared.max_live_allocs_per_worker = config.max_live_allocs_per_worker;
  shared.effective_max_heap_bytes = effective_max_heap_bytes;
  shared.max_pmm_alloc_frames = config.max_pmm_alloc_frames;
  shared.master_seed = master_seed;
  shared.cpu_participation_mask.store(1ULL, std::memory_order_relaxed);

  std::unique_ptr<StressWorkerContext[]> workers(
      new StressWorkerContext[num_workers]);
  CHECK(workers != nullptr);

  Prng64 master_prng(master_seed);

  // Enqueue the initial half of the workers on CPU 0 while preemption is
  // briefly disabled so remote CPUs (1 .. cpu_count - 1) immediately steal work
  // across CPUs, and place the remaining workers via both CreateTask and
  // CreateTaskOnCpu across all online CPUs.
  scheduler->SetPreemptEnabled(false);
  for (int i = 0; i < num_workers; ++i) {
    workers[i].shared = &shared;
    workers[i].worker_index = i;
    workers[i].worker_seed =
        master_prng.Fork(static_cast<uint64_t>(i + 1)).NextU64();
    const int64_t worker_weight =
        master_prng.NextRange(kDefaultTaskWeight / 2, kDefaultTaskWeight * 2);

    if (cpu_count > 1 && i < (num_workers / 2)) {
      workers[i].initial_cpu = 0;
      workers[i].task = scheduler->CreateTaskOnCpu(StressWorkerEntry,  //
                                                   &workers[i],        //
                                                   worker_weight,      //
                                                   0);
    } else if ((i & 1) == 0) {
      const int target_cpu = i % cpu_count;
      workers[i].initial_cpu = target_cpu;
      workers[i].task = scheduler->CreateTaskOnCpu(StressWorkerEntry,  //
                                                   &workers[i],        //
                                                   worker_weight,      //
                                                   target_cpu);
    } else {
      workers[i].initial_cpu = i % cpu_count;
      workers[i].task = scheduler->CreateTask(StressWorkerEntry,  //
                                              &workers[i],        //
                                              worker_weight);
    }
    CHECK(workers[i].task != nullptr);
  }
  scheduler->SetPreemptEnabled(true);

  for (int c = 1; c < cpu_count; ++c) {
    SmpSendIpi(c, kVectorWakeupIpi);
    shared.wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
  }

  for (int i = 0; i < num_workers; ++i) {
    scheduler->Join(workers[i].task);
  }

  // Wait for all detached child tasks to complete and reap all zombies.
  while (shared.detached_children_finished.load(std::memory_order_acquire) <
         shared.child_tasks_detached.load(std::memory_order_acquire)) {
    const int reaped = scheduler->ReapZombies();
    if (reaped > 0) {
      shared.zombies_reaped.fetch_add(reaped, std::memory_order_acq_rel);
    }
    scheduler->Yield();
  }

  while (shared.zombies_reaped.load(std::memory_order_acquire) <
         shared.child_tasks_detached.load(std::memory_order_acquire)) {
    const int reaped = scheduler->ReapZombies();
    if (reaped > 0) {
      shared.zombies_reaped.fetch_add(reaped, std::memory_order_acq_rel);
    } else {
      scheduler->Yield();
#if __STDC_HOSTED__
      std::this_thread::yield();
#else
      asm volatile("pause" : : : "memory");
#endif
    }
  }
  const int final_reaped = scheduler->ReapZombies();
  if (final_reaped > 0) {
    shared.zombies_reaped.fetch_add(final_reaped, std::memory_order_acq_rel);
  }

  // Ensure every online CPU (0 .. cpu_count - 1) has participated and at least
  // one cross-CPU migration has occurred even if num_workers < cpu_count or
  // host OS thread scheduling delayed an AP thread during the worker window.
  // By the Pigeonhole Principle, spawning `cpu_count - 1` non-yielding,
  // non-exiting barrier tasks across `cpu_count - 1` APs with preemption
  // disabled while CPU 0 spins without yielding/stealing guarantees that each
  // AP holds at most one barrier task and every AP `1 .. cpu_count - 1` must
  // enter a barrier task before the rendezvous completes.
  const uint64_t expected_cpu_mask =
      (cpu_count >= 64) ? ~0ULL
                        : ((1ULL << static_cast<uint32_t>(cpu_count)) - 1ULL);
  if (cpu_count > 1 &&
      ((shared.cpu_participation_mask.load(std::memory_order_acquire) &
        expected_cpu_mask) != expected_cpu_mask ||
       shared.task_migrations_observed.load(std::memory_order_acquire) == 0)) {
    const bool prev_preempt = scheduler->IsPreemptEnabled();
    scheduler->SetPreemptEnabled(false);
    shared.cpu_participation_mask.fetch_or(1ULL, std::memory_order_acq_rel);

    std::atomic<uint64_t> barrier_mask{1ULL};
    CpuParticipationBarrierContext barrier_ctxs[64] = {};
    Task* barrier_tasks[64] = {};
    for (int c = 1; c < cpu_count; ++c) {
      barrier_ctxs[c].shared = &shared;
      barrier_ctxs[c].barrier_mask = &barrier_mask;
      barrier_ctxs[c].expected_cpu_mask = expected_cpu_mask;
      barrier_ctxs[c].initial_cpu = 0;
      barrier_tasks[c] =
          scheduler->CreateTaskOnCpu(CpuParticipationBarrierEntry,  //
                                     &barrier_ctxs[c],              //
                                     kDefaultTaskWeight,            //
                                     c);
      CHECK(barrier_tasks[c] != nullptr);
      SmpSendIpi(c, kVectorWakeupIpi);
      shared.wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
    }

    int wait_spins = 0;
    while ((barrier_mask.load(std::memory_order_acquire) & expected_cpu_mask) !=
           expected_cpu_mask) {
      if ((wait_spins & 255) == 0) {
        const uint64_t mask = barrier_mask.load(std::memory_order_acquire);
        for (int c = 1; c < cpu_count; ++c) {
          if ((mask & (1ULL << static_cast<uint32_t>(c))) == 0) {
            SmpSendIpi(c, kVectorWakeupIpi);
            shared.wakeup_ipis_sent.fetch_add(1, std::memory_order_acq_rel);
          }
        }
      }
      ++wait_spins;
      asm volatile("pause" : : : "memory");
#if __STDC_HOSTED__
      std::this_thread::yield();
#endif
    }

    for (int c = 1; c < cpu_count; ++c) {
      scheduler->Join(barrier_tasks[c]);
      shared.child_tasks_joined.fetch_add(1, std::memory_order_acq_rel);
    }
    scheduler->SetPreemptEnabled(prev_preempt);
  }

  for (int i = 0; i < num_workers; ++i) {
    CHECK(workers[i].done.load(std::memory_order_acquire));
  }
  workers.reset();

#if !__STDC_HOSTED__
  if (config.boost_lapic_timer && calibrated_timer_count > 0) {
    uint32_t restore_count = calibrated_timer_count;
    SmpRunOnAllCpus(ProgramLapicTimerOnCpuCallback, &restore_count);
  }
#endif

  const int64_t pmm_free_after = PmmFreeFrameCount();
  const int64_t heap_free_after = HeapTotalFreeBytes();

  StressTestStats stats = {};
  stats.seed = master_seed;
  stats.cpu_count = cpu_count;
  stats.cpu_participation_mask =
      shared.cpu_participation_mask.load(std::memory_order_acquire);
  stats.cooperative_yields =
      shared.cooperative_yields.load(std::memory_order_acquire);
  stats.preemptive_spin_bursts =
      shared.preemptive_spin_bursts.load(std::memory_order_acquire);
  stats.timer_preemptions_observed =
      shared.timer_preemptions_observed.load(std::memory_order_acquire);
  stats.wakeup_ipis_sent =
      shared.wakeup_ipis_sent.load(std::memory_order_acquire);
  stats.task_migrations_observed =
      shared.task_migrations_observed.load(std::memory_order_acquire);
  stats.child_tasks_joined =
      shared.child_tasks_joined.load(std::memory_order_acquire);
  stats.child_tasks_detached =
      shared.child_tasks_detached.load(std::memory_order_acquire);
  stats.zombies_reaped = shared.zombies_reaped.load(std::memory_order_acquire);
  stats.kmalloc_ops = shared.kmalloc_ops.load(std::memory_order_acquire);
  stats.kmalloc_aligned_ops =
      shared.kmalloc_aligned_ops.load(std::memory_order_acquire);
  stats.cpp_new_delete_ops =
      shared.cpp_new_delete_ops.load(std::memory_order_acquire);
  stats.pmm_alloc_ops = shared.pmm_alloc_ops.load(std::memory_order_acquire);
  stats.payload_verifications =
      shared.payload_verifications.load(std::memory_order_acquire);
  stats.payload_corruptions =
      shared.payload_corruptions.load(std::memory_order_acquire);
  stats.stack_canary_corruptions =
      shared.stack_canary_corruptions.load(std::memory_order_acquire);
  stats.pmm_free_before = pmm_free_before;
  stats.pmm_free_after = pmm_free_after;
  stats.heap_free_before = heap_free_before;
  stats.heap_free_after = heap_free_after;

  CHECK(!shared.invariant_failed.load(std::memory_order_acquire));
  CHECK(shared.workers_completed.load(std::memory_order_acquire) ==
        num_workers);
  CHECK((stats.cpu_participation_mask & expected_cpu_mask) ==
        expected_cpu_mask);
  CHECK(stats.cooperative_yields > 0);
  CHECK(stats.preemptive_spin_bursts > 0);
  CHECK(stats.timer_preemptions_observed > 0);
  if (cpu_count > 1) {
    CHECK(stats.wakeup_ipis_sent > 0);
    CHECK(stats.task_migrations_observed > 0);
  }
  CHECK(stats.child_tasks_joined > 0);
  CHECK(stats.child_tasks_detached > 0);
  CHECK(stats.zombies_reaped == stats.child_tasks_detached);
  CHECK(stats.kmalloc_ops > 0);
  CHECK(stats.kmalloc_aligned_ops > 0);
  CHECK(stats.cpp_new_delete_ops > 0);
  CHECK(stats.pmm_alloc_ops > 0);
  CHECK(stats.payload_verifications > 0);
  CHECK(stats.payload_corruptions == 0);
  CHECK(stats.stack_canary_corruptions == 0);
  CHECK(stats.pmm_free_after == stats.pmm_free_before);
  CHECK(stats.heap_free_after == stats.heap_free_before);
  CHECK(CurrentCpuId() == 0);
  CHECK(scheduler->CurrentTask() == bootstrap_task);
  CHECK(scheduler->IsPreemptEnabled());
  CHECK(AreInterruptsEnabled());

  if (config.log_to_console) {
    UartWrite("[STRESS] cpus=");
    UartWriteDec(static_cast<uint64_t>(stats.cpu_count));
    UartWrite(", yields=");
    UartWriteDec(static_cast<uint64_t>(stats.cooperative_yields));
    UartWrite(", preempts=");
    UartWriteDec(static_cast<uint64_t>(stats.timer_preemptions_observed));
    UartWrite(", migrations=");
    UartWriteDec(static_cast<uint64_t>(stats.task_migrations_observed));
    UartWrite(", joined=");
    UartWriteDec(static_cast<uint64_t>(stats.child_tasks_joined));
    UartWrite(", reaped=");
    UartWriteDec(static_cast<uint64_t>(stats.zombies_reaped));
    UartWrite(", verifications=");
    UartWriteDec(static_cast<uint64_t>(stats.payload_verifications));
    UartWrite("\n");
    UartWrite("[STRESS] random_multicpu_stress: PASS\n");
    constexpr char kSpinnerChars[4] = {'|', '/', '-', '\\'};
    const uint64_t pass_num =
        g_stress_pass_count.fetch_add(1, std::memory_order_relaxed) + 1;
    const char spinner_str[4] = {' ', kSpinnerChars[pass_num & 3ULL], ' ',
                                 '\0'};
    VgaWrite("\r[STRESS] random_multicpu_stress: PASS #");
    VgaWriteDec(pass_num);
    VgaWrite(spinner_str);
    VgaWrite("seed=");
    VgaWriteHex(stats.seed);
    VgaWrite("   \r");
  }

  return stats;
}

void RunRandomMultiCpuStressTest(const uint64_t seed_override) {
  StressTestConfig config = {};
  config.seed = seed_override;
  config.num_workers = 0;
  config.rounds_per_worker = 24;
  config.max_live_allocs_per_worker = 4;
  config.max_heap_alloc_bytes = 2048;
  config.max_pmm_alloc_frames = 3;
  config.boost_lapic_timer = true;
  config.log_to_console = true;
  (void)ExecuteRandomMultiCpuStress(config);
}

}  // namespace protos
