#include "stress_test.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "scheduler_test_support.h"

namespace protos {
namespace {

namespace t = ::testing;

static_assert(Prng64(0).state() != 0);
static_assert(Prng64(1).state() != 0);
static_assert(Prng64(1).state() != Prng64(2).state());

TEST(StressTest, Prng64DeterminismDistinctSeedsAndSeedZeroNonZeroState) {
  Prng64 rng_zero_a(0);
  Prng64 rng_zero_b(0);
  EXPECT_THAT(rng_zero_a.state(), t::Ne(0u));
  EXPECT_THAT(rng_zero_a.state(), t::Eq(rng_zero_b.state()));

  Prng64 rng_one(1);
  Prng64 rng_two(2);
  EXPECT_THAT(rng_one.state(), t::Ne(rng_two.state()));

  for (int i = 0; i < 256; ++i) {
    const uint64_t val_a = rng_zero_a.NextU64();
    const uint64_t val_b = rng_zero_b.NextU64();
    EXPECT_THAT(val_a, t::Eq(val_b));
    EXPECT_THAT(rng_zero_a.state(), t::Ne(0u));
    EXPECT_THAT(rng_one.NextU64(), t::Ne(rng_two.NextU64()));
  }
}

TEST(StressTest, Prng64ForkProducesIndependentDeterministicStreams) {
  Prng64 base_a(0xCAFEBABE12345678ULL);
  Prng64 base_b(0xCAFEBABE12345678ULL);

  Prng64 stream_a1 = base_a.Fork(1);
  Prng64 stream_a2 = base_a.Fork(2);
  Prng64 stream_b1 = base_b.Fork(1);
  Prng64 stream_b2 = base_b.Fork(2);

  for (int i = 0; i < 128; ++i) {
    const uint64_t v_a1 = stream_a1.NextU64();
    const uint64_t v_a2 = stream_a2.NextU64();
    EXPECT_THAT(v_a1, t::Eq(stream_b1.NextU64()));
    EXPECT_THAT(v_a2, t::Eq(stream_b2.NextU64()));
    EXPECT_THAT(v_a1, t::Ne(v_a2));
  }

  // Forking with different stream IDs from identical starting states must also
  // produce distinct child streams.
  Prng64 copy_1(42);
  Prng64 copy_2(42);
  Prng64 forked_1 = copy_1.Fork(100);
  Prng64 forked_2 = copy_2.Fork(101);
  EXPECT_THAT(forked_1.NextU64(), t::Ne(forked_2.NextU64()));
}

TEST(StressTest, Prng64BoundedAndRangeHelpersRespectBoundsAndDistribution) {
  Prng64 rng(0x13579BDF2468ACE0ULL);

  for (int i = 0; i < 64; ++i) {
    EXPECT_THAT(rng.NextBounded(1), t::Eq(0u));
    EXPECT_THAT(rng.NextRange(42, 42), t::Eq(42));
    EXPECT_THAT(rng.NextIntRange(-17, -17), t::Eq(-17));
    EXPECT_FALSE(rng.NextChance(0, 10));
    EXPECT_TRUE(rng.NextChance(10, 10));
  }

  constexpr int kBuckets = 8;
  constexpr int kSamples = 4000;
  int bucket_counts[kBuckets] = {};
  int true_bools = 0;
  int chance_hits = 0;

  for (int i = 0; i < kSamples; ++i) {
    const uint64_t b = rng.NextBounded(kBuckets);
    ASSERT_THAT(b, t::Lt(static_cast<uint64_t>(kBuckets)));
    ++bucket_counts[static_cast<int>(b)];

    const int64_t r64 = rng.NextRange(-500, 500);
    EXPECT_THAT(r64, t::AllOf(t::Ge(-500), t::Le(500)));

    const int r_int = rng.NextIntRange(kMinTaskWeight, kMaxTaskWeight);
    EXPECT_THAT(r_int, t::AllOf(t::Ge(static_cast<int>(kMinTaskWeight)),
                                t::Le(static_cast<int>(kMaxTaskWeight))));

    if (rng.NextBool()) {
      ++true_bools;
    }
    if (rng.NextChance(3, 10)) {
      ++chance_hits;
    }
  }

  for (int b = 0; b < kBuckets; ++b) {
    EXPECT_THAT(bucket_counts[b], t::AllOf(t::Gt(300), t::Lt(700)));
  }
  EXPECT_THAT(true_bools, t::AllOf(t::Gt(1700), t::Lt(2300)));
  EXPECT_THAT(chance_hits, t::AllOf(t::Gt(900), t::Lt(1500)));

  // Full 64-bit signed range [INT64_MIN, INT64_MAX] must not overflow.
  const int64_t full_range_val = rng.NextRange(INT64_MIN, INT64_MAX);
  (void)full_range_val;
}

TEST(StressTest, Prng64NextPowerOfTwoAlignmentCoversAllValidPowersOfTwo) {
  Prng64 rng(0x9E3779B97F4A7C15ULL);

  for (int i = 0; i < 32; ++i) {
    EXPECT_THAT(rng.NextPowerOfTwoAlignment(64, 64), t::Eq(64));
  }

  int exp_hits[13] = {};
  for (int i = 0; i < 2600; ++i) {
    const int64_t align = rng.NextPowerOfTwoAlignment(1, kPageSize);
    EXPECT_THAT(align, t::AllOf(t::Ge(1), t::Le(kPageSize)));
    EXPECT_THAT(align & (align - 1), t::Eq(0));

    int exp = 0;
    while ((1LL << exp) < align) {
      ++exp;
    }
    ASSERT_THAT(exp, t::AllOf(t::Ge(0), t::Le(12)));
    ++exp_hits[exp];
  }

  for (int exp = 0; exp <= 12; ++exp) {
    EXPECT_THAT(exp_hits[exp], t::Gt(80));
  }
}

static std::atomic<int> g_hook_call_count{0};

static bool FailFourTimesThenSucceedHook(uint64_t* const out) {
  const int call =
      g_hook_call_count.fetch_add(1, std::memory_order_acq_rel) + 1;
  if (call < 5) {
    return false;
  }
  *out = 0xA5A55A5AF00DCAFEULL;
  return true;
}

static bool AlwaysFailHook(uint64_t* const /*out*/) {
  g_hook_call_count.fetch_add(1, std::memory_order_acq_rel);
  return false;
}

TEST(StressTest, HardwareEntropyHelpersAndRdrandRetryFallback) {
  SetupEnv(2);

  const uint64_t tsc1 = ReadTsc64();
  const uint64_t tsc2 = ReadTsc64();
  EXPECT_THAT(tsc2, t::Ge(tsc1));

  // Verify MixEntropy64 avalanche behavior across single-bit input changes.
  const uint64_t m0 = MixEntropy64(0x123456789ABCDEF0ULL, 1ULL);
  const uint64_t m1 = MixEntropy64(0x123456789ABCDEF0ULL, 2ULL);
  EXPECT_THAT(m0, t::Ne(m1));
  EXPECT_THAT(__builtin_popcountll(m0 ^ m1), t::AllOf(t::Ge(16), t::Le(48)));

  // Test real hardware RDRAND / RDTSC entropy harvesting path.
  uint64_t hw_val = 0;
  if (CpuSupportsRdrand()) {
    EXPECT_TRUE(TryReadRdrand64(&hw_val));
  }
  const uint64_t seed_a = HarvestHardwareEntropySeed(0x1111ULL);
  const uint64_t seed_b = HarvestHardwareEntropySeed(0x2222ULL);
  EXPECT_THAT(seed_a, t::Ne(0u));
  EXPECT_THAT(seed_b, t::Ne(0u));
  EXPECT_THAT(seed_a, t::Ne(seed_b));

  // 1. Simulate CPU without RDRAND support: TryReadRdrand64 must return false
  //    cleanly and HarvestHardwareEntropySeed must still return non-zero seeds.
  SetRdrandHookForTest(false, nullptr);
  EXPECT_FALSE(CpuSupportsRdrand());
  uint64_t unsupported_out = 0xDEADBEEFULL;
  EXPECT_FALSE(TryReadRdrand64(&unsupported_out));
  EXPECT_THAT(unsupported_out, t::Eq(0u));
  const uint64_t fallback_seed_1 = HarvestHardwareEntropySeed(10);
  const uint64_t fallback_seed_2 = HarvestHardwareEntropySeed(20);
  EXPECT_THAT(fallback_seed_1, t::Ne(0u));
  EXPECT_THAT(fallback_seed_2, t::Ne(0u));
  EXPECT_THAT(fallback_seed_1, t::Ne(fallback_seed_2));

  // 2. Simulate transient RDRAND FIFO exhaustion that succeeds on attempt 5.
  g_hook_call_count.store(0, std::memory_order_relaxed);
  SetRdrandHookForTest(true, FailFourTimesThenSucceedHook);
  EXPECT_TRUE(CpuSupportsRdrand());
  uint64_t retry_out = 0;
  EXPECT_TRUE(TryReadRdrand64(&retry_out));
  EXPECT_THAT(retry_out, t::Eq(0xA5A55A5AF00DCAFEULL));
  EXPECT_THAT(g_hook_call_count.load(), t::Eq(5));

  // 3. Simulate persistent RDRAND failure across all 10 retry attempts.
  g_hook_call_count.store(0, std::memory_order_relaxed);
  SetRdrandHookForTest(true, AlwaysFailHook);
  uint64_t exhausted_out = 0x1234ULL;
  EXPECT_FALSE(TryReadRdrand64(&exhausted_out));
  EXPECT_THAT(exhausted_out, t::Eq(0u));
  EXPECT_THAT(g_hook_call_count.load(), t::Eq(10));

  ResetRdrandHookForTest();

  // 4. Verify HarvestHardwareEntropySeed also works when CurrentCpuOrNull() is
  //    unbound (nullptr).
  ResetCpuLocalForTest();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());
  const uint64_t unbound_seed = HarvestHardwareEntropySeed(0x3333ULL);
  EXPECT_THAT(unbound_seed, t::Ne(0u));
  BindCpuLocal(&g_env.cpu_locals[0]);
}

TEST(StressTest, RandomizedTaskWeightDispatchWithMockScheduler) {
  const std::shared_ptr<MockTaskScheduler> mock_sched =
      std::make_shared<MockTaskScheduler>();
  Prng64 rng(0x42424242ULL);

  std::vector<int64_t> observed_weights;
  std::vector<int> observed_cpus;

  EXPECT_CALL(*mock_sched, CreateTaskOnCpu)
      .Times(16)
      .WillRepeatedly([&](const TaskFn entry,    //
                          void* const /*arg*/,   //
                          const int64_t weight,  //
                          const int target_cpu) -> Task* {
        EXPECT_THAT(entry, t::NotNull());
        observed_weights.push_back(weight);
        observed_cpus.push_back(target_cpu);
        return reinterpret_cast<Task*>(0x1000);
      });
  EXPECT_CALL(*mock_sched, Yield).Times(16);

  for (int i = 0; i < 16; ++i) {
    const int64_t weight = rng.NextRange(kMinTaskWeight, kMaxTaskWeight);
    const int target_cpu = rng.NextIntRange(0, 3);
    Task* const t = mock_sched->CreateTaskOnCpu([](void* const /*arg*/) {},  //
                                                nullptr,                     //
                                                weight,                      //
                                                target_cpu);
    EXPECT_THAT(t, t::NotNull());
    mock_sched->Yield();
  }

  ASSERT_THAT(static_cast<int>(observed_weights.size()), t::Eq(16));
  for (int i = 0; i < 16; ++i) {
    EXPECT_THAT(observed_weights[i],
                t::AllOf(t::Ge(kMinTaskWeight), t::Le(kMaxTaskWeight)));
    EXPECT_THAT(observed_cpus[i], t::AllOf(t::Ge(0), t::Le(3)));
  }
}

TEST(StressTest, SingleCpuRandomizedStressRestoresExactBaselines) {
  SetupEnv(1);
  TaskInit();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  StressTestConfig config = {};
  config.seed = 0xABCDEF0123456789ULL;
  config.num_workers = 4;
  config.rounds_per_worker = 12;
  config.max_live_allocs_per_worker = 4;
  config.max_heap_alloc_bytes = 1024;
  config.max_pmm_alloc_frames = 3;
  config.boost_lapic_timer = false;
  config.log_to_console = false;

  const StressTestStats stats = ExecuteRandomMultiCpuStress(config);

  EXPECT_THAT(stats.seed, t::Eq(config.seed));
  EXPECT_THAT(stats.cpu_count, t::Eq(1));
  EXPECT_THAT(stats.cpu_participation_mask, t::Eq(0x1u));
  EXPECT_THAT(stats.cooperative_yields, t::Gt(0));
  EXPECT_THAT(stats.preemptive_spin_bursts, t::Gt(0));
  EXPECT_THAT(stats.timer_preemptions_observed, t::Gt(0));
  EXPECT_THAT(stats.child_tasks_joined, t::Gt(0));
  EXPECT_THAT(stats.child_tasks_detached, t::Gt(0));
  EXPECT_THAT(stats.zombies_reaped, t::Eq(stats.child_tasks_detached));
  EXPECT_THAT(stats.kmalloc_ops, t::Gt(0));
  EXPECT_THAT(stats.kmalloc_aligned_ops, t::Gt(0));
  EXPECT_THAT(stats.cpp_new_delete_ops, t::Gt(0));
  EXPECT_THAT(stats.pmm_alloc_ops, t::Gt(0));
  EXPECT_THAT(stats.payload_verifications, t::Gt(0));
  EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
  EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
  EXPECT_THAT(stats.pmm_free_before, t::Eq(pmm_before));
  EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
  EXPECT_THAT(stats.heap_free_before, t::Eq(heap_before));
  EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  TaskResetForTest();
}

TEST(StressTest, MultiThreadedFourCpuRandomizedStressAndConsoleEntryPoint) {
  constexpr int kNumCpus = 4;
  SetupEnv(kNumCpus);
  TaskInit();

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  std::atomic<int> ready_aps{0};
  std::atomic<bool> stop_aps{false};
  std::vector<std::thread> ap_threads;
  ap_threads.reserve(kNumCpus - 1);

  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &ready_aps, &stop_aps]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      ready_aps.fetch_add(1, std::memory_order_acq_rel);

      int64_t handled_ipis = 0;
      const std::shared_ptr<TaskScheduler>& ap_sched = GetTaskScheduler();
      while (!stop_aps.load(std::memory_order_acquire)) {
        const int64_t sent_ipis =
            g_env.ipi_sent_count[cpu].load(std::memory_order_acquire);
        if (sent_ipis > handled_ipis && IdtIsInitialized() &&
            AreInterruptsEnabled()) {
          handled_ipis = sent_ipis;
          InterruptFrame ipi_frame = {};
          ipi_frame.vector = kVectorWakeupIpi;
          ipi_frame.rflags = internal::kRflagsInterruptEnableBit;
          IdtDispatch(&ipi_frame);
        }
        if (ap_sched == nullptr || !ap_sched->PollIdleCpu(cpu)) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }

  while (ready_aps.load(std::memory_order_acquire) < (kNumCpus - 1)) {
    std::this_thread::yield();
  }

  StressTestConfig config = {};
  config.seed = 0x55AA55AA12349876ULL;
  config.num_workers = 8;
  config.rounds_per_worker = 16;
  config.max_live_allocs_per_worker = 4;
  config.max_heap_alloc_bytes = 2048;
  config.max_pmm_alloc_frames = 3;
  config.boost_lapic_timer = true;
  config.log_to_console = false;

  const StressTestStats stats = ExecuteRandomMultiCpuStress(config);

  EXPECT_THAT(stats.seed, t::Eq(config.seed));
  EXPECT_THAT(stats.cpu_count, t::Eq(kNumCpus));
  EXPECT_THAT(stats.cpu_participation_mask & 0xFu, t::Eq(0xFu));
  EXPECT_THAT(stats.cooperative_yields, t::Gt(0));
  EXPECT_THAT(stats.preemptive_spin_bursts, t::Gt(0));
  EXPECT_THAT(stats.timer_preemptions_observed, t::Gt(0));
  EXPECT_THAT(stats.wakeup_ipis_sent, t::Gt(0));
  EXPECT_THAT(stats.task_migrations_observed, t::Gt(0));
  EXPECT_THAT(stats.child_tasks_joined, t::Gt(0));
  EXPECT_THAT(stats.child_tasks_detached, t::Gt(0));
  EXPECT_THAT(stats.zombies_reaped, t::Eq(stats.child_tasks_detached));
  EXPECT_THAT(stats.kmalloc_ops, t::Gt(0));
  EXPECT_THAT(stats.kmalloc_aligned_ops, t::Gt(0));
  EXPECT_THAT(stats.cpp_new_delete_ops, t::Gt(0));
  EXPECT_THAT(stats.pmm_alloc_ops, t::Gt(0));
  EXPECT_THAT(stats.payload_verifications, t::Gt(0));
  EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
  EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
  EXPECT_THAT(stats.pmm_free_before, t::Eq(pmm_before));
  EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
  EXPECT_THAT(stats.heap_free_before, t::Eq(heap_before));
  EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));

  // Also test the kernel_main entry point RunRandomMultiCpuStressTest with
  // hardware-seeded entropy (seed_override == 0) and verify UART/VGA markers.
  RunRandomMultiCpuStressTest(0);

  stop_aps.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

  {
    const std::lock_guard<std::mutex> lock(g_env.log_mutex);
    EXPECT_THAT(g_env.uart_log, t::HasSubstr("[STRESS] seed=0x"));
    EXPECT_THAT(g_env.uart_log,
                t::HasSubstr("[STRESS] random_multicpu_stress: PASS\n"));
    EXPECT_THAT(g_env.uart_log, t::Not(t::HasSubstr("FAIL")));
    EXPECT_THAT(g_env.vga_log,
                t::HasSubstr("\r[STRESS] random_multicpu_stress: PASS #"));
    EXPECT_THAT(g_env.vga_log, t::HasSubstr("seed=0x"));
    EXPECT_THAT(g_env.vga_log, t::Not(t::HasSubstr("FAIL")));
  }

  TaskResetForTest();
}

TEST(StressTest, MinimalConfigAndDelayedApFourCpuRendezvousBarrier) {
  // 1. Minimal configuration on 1 CPU (num_workers = 1, rounds_per_worker = 1,
  //    max_live_allocs_per_worker = 1) must exercise all 4 allocation families.
  {
    SetupEnv(1);
    TaskInit();

    const int64_t pmm_before = PmmFreeFrameCount();
    const int64_t heap_before = HeapTotalFreeBytes();

    StressTestConfig min_cfg = {};
    min_cfg.seed = 0x1122334455667788ULL;
    min_cfg.num_workers = 1;
    min_cfg.rounds_per_worker = 1;
    min_cfg.max_live_allocs_per_worker = 1;
    min_cfg.max_heap_alloc_bytes = 64;
    min_cfg.max_pmm_alloc_frames = 1;
    min_cfg.boost_lapic_timer = false;
    min_cfg.log_to_console = false;

    const StressTestStats stats = ExecuteRandomMultiCpuStress(min_cfg);
    EXPECT_THAT(stats.cpu_participation_mask, t::Eq(0x1u));
    EXPECT_THAT(stats.kmalloc_ops, t::Ge(1));
    EXPECT_THAT(stats.kmalloc_aligned_ops, t::Ge(1));
    EXPECT_THAT(stats.cpp_new_delete_ops, t::Ge(1));
    EXPECT_THAT(stats.pmm_alloc_ops, t::Ge(1));
    EXPECT_THAT(stats.payload_verifications, t::Ge(4));
    EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
    EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
    EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
    EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));

    TaskResetForTest();
  }

  // 2. Minimal configuration (num_workers = 1, rounds_per_worker = 1) on 4 CPUs
  //    with a delayed AP thread (CPU 3 sleeping 15 ms before entering
  //    PollIdleCpu) must still achieve full 4-CPU participation (0xF) and
  //    migrations > 0 via the Pigeonhole Principle rendezvous barrier without
  //    having its barrier task stolen and prematurely completed by CPU 0 or
  //    other APs.
  {
    constexpr int kNumCpus = 4;
    SetupEnv(kNumCpus);
    TaskInit();

    const int64_t pmm_before = PmmFreeFrameCount();
    const int64_t heap_before = HeapTotalFreeBytes();

    std::atomic<int> ready_aps{0};
    std::atomic<bool> stop_aps{false};
    std::vector<std::thread> ap_threads;
    ap_threads.reserve(kNumCpus - 1);

    for (int cpu = 1; cpu < kNumCpus; ++cpu) {
      ap_threads.emplace_back([cpu, &ready_aps, &stop_aps]() {
        BindCpuLocal(SmpGetCpuLocal(cpu));
        SetInterruptsEnabledForTest(true);
        ready_aps.fetch_add(1, std::memory_order_acq_rel);

        if (cpu == 3) {
          std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }

        int64_t handled_ipis = 0;
        const std::shared_ptr<TaskScheduler>& ap_sched = GetTaskScheduler();
        while (!stop_aps.load(std::memory_order_acquire)) {
          const int64_t sent_ipis =
              g_env.ipi_sent_count[cpu].load(std::memory_order_acquire);
          if (sent_ipis > handled_ipis && IdtIsInitialized() &&
              AreInterruptsEnabled()) {
            handled_ipis = sent_ipis;
            InterruptFrame ipi_frame = {};
            ipi_frame.vector = kVectorWakeupIpi;
            ipi_frame.rflags = internal::kRflagsInterruptEnableBit;
            IdtDispatch(&ipi_frame);
          }
          if (ap_sched == nullptr || !ap_sched->PollIdleCpu(cpu)) {
            std::this_thread::yield();
          }
        }
        ResetCpuLocalForTest();
      });
    }

    while (ready_aps.load(std::memory_order_acquire) < (kNumCpus - 1)) {
      std::this_thread::yield();
    }

    StressTestConfig delayed_cfg = {};
    delayed_cfg.seed = 0xDEADC0DEBEEF1234ULL;
    delayed_cfg.num_workers = 1;
    delayed_cfg.rounds_per_worker = 1;
    delayed_cfg.max_live_allocs_per_worker = 1;
    delayed_cfg.max_heap_alloc_bytes = 128;
    delayed_cfg.max_pmm_alloc_frames = 1;
    delayed_cfg.boost_lapic_timer = false;
    delayed_cfg.log_to_console = false;

    const StressTestStats stats = ExecuteRandomMultiCpuStress(delayed_cfg);

    stop_aps.store(true, std::memory_order_release);
    for (std::thread& th : ap_threads) {
      th.join();
    }

    EXPECT_THAT(stats.cpu_count, t::Eq(kNumCpus));
    EXPECT_THAT(stats.cpu_participation_mask & 0xFu, t::Eq(0xFu));
    EXPECT_THAT(stats.task_migrations_observed, t::Gt(0));
    EXPECT_THAT(stats.kmalloc_ops, t::Ge(1));
    EXPECT_THAT(stats.kmalloc_aligned_ops, t::Ge(1));
    EXPECT_THAT(stats.cpp_new_delete_ops, t::Ge(1));
    EXPECT_THAT(stats.pmm_alloc_ops, t::Ge(1));
    EXPECT_THAT(stats.payload_verifications, t::Ge(4));
    EXPECT_THAT(stats.payload_corruptions, t::Eq(0));
    EXPECT_THAT(stats.stack_canary_corruptions, t::Eq(0));
    EXPECT_THAT(stats.pmm_free_after, t::Eq(pmm_before));
    EXPECT_THAT(stats.heap_free_after, t::Eq(heap_before));
    EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
    EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));

    TaskResetForTest();
  }
}

TEST(StressDeathTest, PreconditionViolationsTriggerDcheck) {
  SetupEnv(1);
  TaskInit();

  EXPECT_DEATH(TryReadRdrand64(nullptr), "Check failed");
  EXPECT_DEATH(Prng64(1).NextBounded(0), "Check failed");
  EXPECT_DEATH(Prng64(1).NextRange(10, 5), "Check failed");
  EXPECT_DEATH(Prng64(1).NextIntRange(10, -1), "Check failed");
  EXPECT_DEATH(Prng64(1).NextChance(-1, 5), "Check failed");
  EXPECT_DEATH(Prng64(1).NextChance(6, 5), "Check failed");
  EXPECT_DEATH(Prng64(1).NextChance(1, 0), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(0, 16), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(3, 16), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(16, 15), "Check failed");
  EXPECT_DEATH(Prng64(1).NextPowerOfTwoAlignment(32, 16), "Check failed");

  StressTestConfig bad_workers = {};
  bad_workers.num_workers = -1;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_workers), "Check failed");

  StressTestConfig bad_workers_high = {};
  bad_workers_high.num_workers = 65;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_workers_high), "Check failed");

  StressTestConfig bad_rounds = {};
  bad_rounds.rounds_per_worker = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_rounds), "Check failed");

  StressTestConfig bad_rounds_high = {};
  bad_rounds_high.rounds_per_worker = 4097;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_rounds_high), "Check failed");

  StressTestConfig bad_slots = {};
  bad_slots.max_live_allocs_per_worker = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_slots), "Check failed");

  StressTestConfig bad_slots_high = {};
  bad_slots_high.max_live_allocs_per_worker = 9;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_slots_high), "Check failed");

  StressTestConfig bad_heap = {};
  bad_heap.max_heap_alloc_bytes = 8;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_heap), "Check failed");

  StressTestConfig bad_heap_high = {};
  bad_heap_high.max_heap_alloc_bytes = 16385;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_heap_high), "Check failed");

  StressTestConfig bad_pmm = {};
  bad_pmm.max_pmm_alloc_frames = 0;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_pmm), "Check failed");

  StressTestConfig bad_pmm_high = {};
  bad_pmm_high.max_pmm_alloc_frames = 9;
  EXPECT_DEATH(ExecuteRandomMultiCpuStress(bad_pmm_high), "Check failed");

  TaskResetForTest();
}

}  // namespace
}  // namespace protos
