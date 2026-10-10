#include "preemptive_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "scheduler_test_support.h"

namespace protos {
namespace {

namespace t = ::testing;

TEST(PreemptiveSchedulerTest,
     DefaultPreemptStateTogglingAndInitResetsToEnabled) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);
  ASSERT_THAT(sched, t::NotNull());

  // Preemption is enabled by default upon construction.
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // SetPreemptEnabled toggles state locally without calling inner.
  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());

  sched->SetPreemptEnabled(true);
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // Init() resets preempt_enabled_ back to true and delegates to inner->Init().
  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());

  int init_calls = 0;
  EXPECT_CALL(*mock_inner, Init).WillOnce([&]() { ++init_calls; });

  sched->Init();
  EXPECT_THAT(init_calls, t::Eq(1));
  EXPECT_TRUE(sched->IsPreemptEnabled());
}

TEST(PreemptiveSchedulerTest,
     ForwardsTimerInterruptAndWakeupIpiWhenAllSafetyConditionsHold) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  // Case 1: frame == nullptr.
  bool timer_null_called = false;
  bool ipi_null_called = false;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::IsNull());
        timer_null_called = true;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::IsNull());
        ipi_null_called = true;
      });

  sched->OnTimerInterrupt(nullptr);
  sched->OnWakeupIpi(nullptr);
  EXPECT_TRUE(timer_null_called);
  EXPECT_TRUE(ipi_null_called);

  // Case 2: frame != nullptr with frame->rflags == 0.
  InterruptFrame zero_rflags_frame = {};
  zero_rflags_frame.vector = kVectorApicTimer;
  zero_rflags_frame.rflags = 0;

  InterruptFrame* observed_timer_frame = nullptr;
  InterruptFrame* observed_ipi_frame = nullptr;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_timer_frame = frame; });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_ipi_frame = frame; });

  sched->OnTimerInterrupt(&zero_rflags_frame);
  EXPECT_THAT(observed_timer_frame, t::Eq(&zero_rflags_frame));

  zero_rflags_frame.vector = kVectorWakeupIpi;
  sched->OnWakeupIpi(&zero_rflags_frame);
  EXPECT_THAT(observed_ipi_frame, t::Eq(&zero_rflags_frame));

  // Case 3: frame != nullptr with kRflagsInterruptEnableBit set in rflags.
  InterruptFrame enabled_frame = {};
  enabled_frame.vector = kVectorApicTimer;
  enabled_frame.rflags = internal::kRflagsInterruptEnableBit | 0x2ULL;

  observed_timer_frame = nullptr;
  observed_ipi_frame = nullptr;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_timer_frame = frame; });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_ipi_frame = frame; });

  sched->OnTimerInterrupt(&enabled_frame);
  EXPECT_THAT(observed_timer_frame, t::Eq(&enabled_frame));

  enabled_frame.vector = kVectorWakeupIpi;
  sched->OnWakeupIpi(&enabled_frame);
  EXPECT_THAT(observed_ipi_frame, t::Eq(&enabled_frame));
}

TEST(PreemptiveSchedulerTest, SuppressesInterruptsWhenPreemptDisabled) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());

  // StrictMock verifies zero calls to inner when preemption is disabled.
  sched->OnTimerInterrupt(&frame);
  sched->OnTimerInterrupt(nullptr);
  sched->OnWakeupIpi(&frame);
  sched->OnWakeupIpi(nullptr);

  // Re-enabling preemption restores forwarding.
  sched->SetPreemptEnabled(true);
  int timer_calls = 0;
  int ipi_calls = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++timer_calls;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++ipi_calls;
      });

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
  EXPECT_THAT(timer_calls, t::Eq(1));
  EXPECT_THAT(ipi_calls, t::Eq(1));
}

TEST(PreemptiveSchedulerTest, SuppressesInterruptsWhenCpuLocalIsNull) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  // Unbind CpuLocal on the current host thread.
  ResetCpuLocalForTest();
  EXPECT_THAT(CurrentCpuOrNull(), t::IsNull());

  // StrictMock verifies zero calls to inner when CurrentCpuOrNull() == nullptr.
  sched->OnTimerInterrupt(&frame);
  sched->OnTimerInterrupt(nullptr);
  sched->OnWakeupIpi(&frame);
  sched->OnWakeupIpi(nullptr);

  // Re-binding valid online CpuLocal restores forwarding.
  BindCpuLocal(&g_env.cpu_locals[0]);
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest, SuppressesInterruptsWhenCurrentCpuIsOffline) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  g_env.cpu_locals[0].online = false;
  ASSERT_THAT(CurrentCpuOrNull(), t::NotNull());
  EXPECT_FALSE(CurrentCpu()->online);

  // StrictMock verifies zero calls to inner when CurrentCpu()->online == false.
  sched->OnTimerInterrupt(&frame);
  sched->OnTimerInterrupt(nullptr);
  sched->OnWakeupIpi(&frame);
  sched->OnWakeupIpi(nullptr);

  // Marking CPU online restores forwarding.
  g_env.cpu_locals[0].online = true;
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     SuppressesInterruptsWhenIrqSpinLockIsHeldOnCurrentCpu) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  IrqSpinLock outer_lock;
  IrqSpinLock inner_lock;
  {
    const IrqSpinLockGuard outer_guard(outer_lock);
    sched->OnTimerInterrupt(&frame);
    sched->OnWakeupIpi(&frame);

    // Even if the simulated interrupt flag were true while a lock is held on
    // TopHeldLockSlot(), preemption must still be suppressed by the held lock.
    SetInterruptsEnabledForTest(true);
    sched->OnTimerInterrupt(&frame);
    sched->OnWakeupIpi(&frame);
    SetInterruptsEnabledForTest(false);

    {
      const IrqSpinLockGuard inner_guard(inner_lock);
      sched->OnTimerInterrupt(&frame);
      sched->OnWakeupIpi(&frame);
    }

    // Outer lock is still held after inner guard unlocks.
    sched->OnTimerInterrupt(&frame);
    sched->OnWakeupIpi(&frame);
  }

  // Once all locks are released, forwarding succeeds.
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     SuppressesInterruptsWhenHostInterruptFlagIsDisabled) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  SetInterruptsEnabledForTest(false);
  EXPECT_FALSE(AreInterruptsEnabled());

  // StrictMock verifies zero calls when simulated host interrupts are disabled.
  sched->OnTimerInterrupt(&frame);
  sched->OnTimerInterrupt(nullptr);
  sched->OnWakeupIpi(&frame);
  sched->OnWakeupIpi(nullptr);

  SetInterruptsEnabledForTest(true);
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&frame));
        ++forwarded;
      });

  sched->OnTimerInterrupt(&frame);
  sched->OnWakeupIpi(&frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     SuppressesInterruptsWhenFrameRflagsHasInterruptEnableBitCleared) {
  SetupEnv(1);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  // Non-zero rflags with IF (bit 9) == 0 must suppress preemption.
  InterruptFrame cli_frame = {};
  cli_frame.vector = kVectorApicTimer;
  cli_frame.rflags = 0x2ULL;

  sched->OnTimerInterrupt(&cli_frame);
  sched->OnWakeupIpi(&cli_frame);

  cli_frame.rflags = 0x246ULL & ~internal::kRflagsInterruptEnableBit;
  sched->OnTimerInterrupt(&cli_frame);
  sched->OnWakeupIpi(&cli_frame);

  // Setting IF (bit 9) in rflags allows forwarding.
  cli_frame.rflags |= internal::kRflagsInterruptEnableBit;
  int forwarded = 0;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&cli_frame));
        ++forwarded;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const passed_frame) {
        EXPECT_THAT(passed_frame, t::Eq(&cli_frame));
        ++forwarded;
      });

  sched->OnTimerInterrupt(&cli_frame);
  sched->OnWakeupIpi(&cli_frame);
  EXPECT_THAT(forwarded, t::Eq(2));
}

TEST(PreemptiveSchedulerTest,
     ForwardsAllTaskManagementAndSchedulingOperationsDirectlyToInner) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  Task* const fake_task_a = reinterpret_cast<Task*>(0x1001);
  Task* const fake_task_b = reinterpret_cast<Task*>(0x1002);
  Task* const fake_current = reinterpret_cast<Task*>(0x1003);
  int dummy_arg_a = 11;
  int dummy_arg_b = 22;

  // 1. CreateTask
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t weight) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg_a));
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight * 2));
        return fake_task_a;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, &dummy_arg_a, kDefaultTaskWeight * 2),
              t::Eq(fake_task_a));

  // 2. CreateTaskOnCpu
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg_b));
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight / 2));
        EXPECT_THAT(target_cpu, t::Eq(3));
        return fake_task_b;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask,                //
                                     &dummy_arg_b,            //
                                     kDefaultTaskWeight / 2,  //
                                     3),
              t::Eq(fake_task_b));

  // 3. Yield
  int yield_calls = 0;
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() { ++yield_calls; });
  sched->Yield();
  EXPECT_THAT(yield_calls, t::Eq(1));

  // 4. Join
  Task* joined_task = nullptr;
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    joined_task = task;
  });
  sched->Join(fake_task_a);
  EXPECT_THAT(joined_task, t::Eq(fake_task_a));

  // 5. ReapZombies
  EXPECT_CALL(*mock_inner, ReapZombies).WillOnce([&]() { return 5; });
  EXPECT_THAT(sched->ReapZombies(), t::Eq(5));

  // 7. PollIdleCpu
  EXPECT_CALL(*mock_inner, PollIdleCpu)
      .WillOnce([&](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return true;
      })
      .WillOnce([&](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(1));
        return false;
      });
  EXPECT_TRUE(sched->PollIdleCpu(2));
  EXPECT_FALSE(sched->PollIdleCpu(1));

  // 8. RunqueueLoad
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu_index) {
    EXPECT_THAT(cpu_index, t::Eq(3));
    return 7;
  });
  EXPECT_THAT(sched->RunqueueLoad(3), t::Eq(7));

  // 9. StealTask
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        return true;
      })
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(1));
        EXPECT_THAT(src_cpu, t::Eq(3));
        return false;
      });
  EXPECT_TRUE(sched->StealTask(0, 2));
  EXPECT_FALSE(sched->StealTask(1, 3));

  // 10. CurrentTask
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() {
    return fake_current;
  });
  EXPECT_THAT(sched->CurrentTask(), t::Eq(fake_current));
}

TEST(PreemptiveSchedulerTest,
     ConcurrentPreemptToggleAndInterruptDispatchUnderTsan) {
  constexpr int kNumThreads = 4;
  constexpr int kIterationsPerThread = 500;
  SetupEnv(kNumThreads);

  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreatePreemptiveScheduler(mock_inner);

  std::atomic<int64_t> timer_forwarded{0};
  std::atomic<int64_t> ipi_forwarded{0};
  std::atomic<bool> safety_violation{false};

  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillRepeatedly([&](InterruptFrame* const frame) {
        const CpuLocal* const cpu = CurrentCpuOrNull();
        if (cpu == nullptr || !cpu->online || !AreInterruptsEnabled()) {
          safety_violation.store(true, std::memory_order_relaxed);
        }
        IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
        if (top_slot != nullptr && *top_slot != nullptr) {
          safety_violation.store(true, std::memory_order_relaxed);
        }
        if (frame != nullptr && frame->rflags != 0 &&
            (frame->rflags & internal::kRflagsInterruptEnableBit) == 0) {
          safety_violation.store(true, std::memory_order_relaxed);
        }
        timer_forwarded.fetch_add(1, std::memory_order_relaxed);
      });

  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillRepeatedly([&](InterruptFrame* const frame) {
        const CpuLocal* const cpu = CurrentCpuOrNull();
        if (cpu == nullptr || !cpu->online || !AreInterruptsEnabled()) {
          safety_violation.store(true, std::memory_order_relaxed);
        }
        IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
        if (top_slot != nullptr && *top_slot != nullptr) {
          safety_violation.store(true, std::memory_order_relaxed);
        }
        if (frame != nullptr && frame->rflags != 0 &&
            (frame->rflags & internal::kRflagsInterruptEnableBit) == 0) {
          safety_violation.store(true, std::memory_order_relaxed);
        }
        ipi_forwarded.fetch_add(1, std::memory_order_relaxed);
      });

  std::atomic<int> ready_threads{0};
  std::atomic<bool> start_flag{false};
  std::vector<std::thread> threads;
  threads.reserve(kNumThreads);

  for (int cpu = 0; cpu < kNumThreads; ++cpu) {
    threads.emplace_back([cpu, &sched, &ready_threads, &start_flag]() {
      BindCpuLocal(&g_env.cpu_locals[cpu]);
      SetInterruptsEnabledForTest(true);
      ready_threads.fetch_add(1, std::memory_order_acq_rel);

      while (!start_flag.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }

      InterruptFrame safe_frame = {};
      safe_frame.vector = kVectorApicTimer;
      safe_frame.rflags = internal::kRflagsInterruptEnableBit;

      InterruptFrame cli_frame = {};
      cli_frame.vector = kVectorApicTimer;
      cli_frame.rflags = 0x2ULL;

      IrqSpinLock local_lock;

      for (int iter = 0; iter < kIterationsPerThread; ++iter) {
        const bool enable = ((iter + cpu) & 3) != 0;
        sched->SetPreemptEnabled(enable);
        (void)sched->IsPreemptEnabled();

        sched->OnTimerInterrupt(&safe_frame);
        sched->OnWakeupIpi(&safe_frame);
        sched->OnTimerInterrupt(&cli_frame);
        sched->OnWakeupIpi(&cli_frame);

        {
          const IrqSpinLockGuard guard(local_lock);
          sched->OnTimerInterrupt(&safe_frame);
          sched->OnWakeupIpi(&safe_frame);
        }
      }

      ResetCpuLocalForTest();
    });
  }

  while (ready_threads.load(std::memory_order_acquire) < kNumThreads) {
    std::this_thread::yield();
  }
  start_flag.store(true, std::memory_order_release);

  for (std::thread& th : threads) {
    th.join();
  }

  BindCpuLocal(&g_env.cpu_locals[0]);
  SetInterruptsEnabledForTest(true);
  sched->SetPreemptEnabled(true);

  InterruptFrame final_frame = {};
  final_frame.vector = kVectorApicTimer;
  final_frame.rflags = internal::kRflagsInterruptEnableBit;
  sched->OnTimerInterrupt(&final_frame);
  sched->OnWakeupIpi(&final_frame);

  EXPECT_FALSE(safety_violation.load(std::memory_order_acquire));
  EXPECT_THAT(timer_forwarded.load(std::memory_order_acquire), t::Gt(0));
  EXPECT_THAT(ipi_forwarded.load(std::memory_order_acquire), t::Gt(0));
}

TEST(PreemptiveSchedulerDeathTest, NullInnerSchedulerTriggersDcheck) {
  SetupEnv(1);
  EXPECT_DEATH(CreatePreemptiveScheduler(nullptr), "Check failed");
}

}  // namespace
}  // namespace protos
