#include "work_stealing_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core_runqueue_scheduler.h"
#include "scheduler_test_support.h"

namespace protos {
namespace {

namespace t = ::testing;

TEST(WorkStealingSchedulerTest, InitDelegatesToInnerScheduler) {
  SetupEnv(2);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);
  ASSERT_THAT(sched, t::NotNull());

  int init_calls = 0;
  EXPECT_CALL(*mock_inner, Init).WillOnce([&]() { ++init_calls; });

  sched->Init();
  EXPECT_THAT(init_calls, t::Eq(1));
}

TEST(WorkStealingSchedulerTest,
     CreateTaskSelectsMinimumLoadOnlineCpuBreaksTiesAndSendsRemoteWakeupIpi) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  int loads[4] = {4, 2, 0, 3};
  Task* const fake_task = reinterpret_cast<Task*>(0x1001);
  int dummy_arg = 42;
  int captured_cpu = -1;
  int64_t captured_weight = 0;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::AllOf(t::Ge(0), t::Lt(4)));
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg));
        captured_weight = weight;
        captured_cpu = target_cpu;
        return fake_task;
      });

  // 1. CPU 2 has minimum load (0). Calling from CPU 0 places the task on remote
  //    CPU 2 and sends kVectorWakeupIpi (0x21) to CPU 2 only.
  EXPECT_THAT(sched->CreateTask(NoopTask, &dummy_arg, kDefaultTaskWeight * 2),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(2));
  EXPECT_THAT(captured_weight, t::Eq(kDefaultTaskWeight * 2));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[2].load(), t::Eq(kVectorWakeupIpi));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[1].load(), t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[3].load(), t::Eq(0));

  // 2. Tie-breaking between non-zero CPUs (loads {5, 1, 3, 1}): must select
  //    lowest cpu_id (CPU 1) and send wakeup IPI to remote CPU 1.
  loads[0] = 5;
  loads[1] = 1;
  loads[2] = 3;
  loads[3] = 1;
  captured_cpu = -1;
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::IsNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        captured_cpu = target_cpu;
        return fake_task;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask), t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(1));
  EXPECT_THAT(g_env.ipi_sent_count[1].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[1].load(), t::Eq(kVectorWakeupIpi));

  // 3. Tie-breaking including CPU 0 (loads {1, 2, 1, 1}): selects CPU 0 and
  //    does NOT send a self-IPI when called from CPU 0.
  loads[0] = 1;
  loads[1] = 2;
  loads[2] = 1;
  loads[3] = 1;
  captured_cpu = -1;
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn /*entry*/,    //
                    void* const /*arg*/,       //
                    const int64_t /*weight*/,  //
                    const int target_cpu) {
        captured_cpu = target_cpu;
        return fake_task;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(captured_cpu, t::Eq(0));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));
}

TEST(WorkStealingSchedulerTest,
     CreateTaskSkipsOfflineCpusAndAvoidsSelfIpiOnNonZeroLocalCpu) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Mark CPU 1 offline and CPU 2 as having a null CpuLocal descriptor.
  g_env.cpu_locals[1].online = false;
  g_env.null_cpu_local[2] = true;
  // Bind current thread to CPU 3.
  BindCpuLocal(&g_env.cpu_locals[3]);

  int loads[4] = {4, 0, 0, 2};
  std::vector<int> queried_cpus;
  Task* const fake_task = reinterpret_cast<Task*>(0x1002);
  int captured_cpu = -1;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    queried_cpus.push_back(cpu);
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn /*entry*/,    //
                    void* const /*arg*/,       //
                    const int64_t /*weight*/,  //
                    const int target_cpu) {
        captured_cpu = target_cpu;
        return fake_task;
      });

  // CPUs 1 and 2 are offline so only CPUs 0 and 3 are queried; CPU 3 has lower
  // load (2 < 4) and is the current CPU, so no IPI is sent.
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(fake_task));
  EXPECT_THAT(queried_cpus, t::ElementsAre(0, 3));
  EXPECT_THAT(captured_cpu, t::Eq(3));
  for (int i = 0; i < 4; ++i) {
    EXPECT_THAT(g_env.ipi_sent_count[i].load(), t::Eq(0));
  }
}

TEST(WorkStealingSchedulerTest,
     CreateTaskOnCpuDelegatesAndSendsWakeupIpiOnlyWhenRemoteOrUnbound) {
  SetupEnv(3);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  Task* const fake_task = reinterpret_cast<Task*>(0x2001);
  int dummy_arg = 7;

  // 1. Target is local CPU (CPU 0 while bound to CPU 0): no wakeup IPI.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg));
        EXPECT_THAT(weight, t::Eq(kMinTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(0));
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, &dummy_arg, kMinTaskWeight, 0),
              t::Eq(fake_task));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(0));

  // 2. Target is remote CPU (CPU 2 while bound to CPU 0): sends wakeup IPI to
  //    CPU 2.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::Eq(&dummy_arg));
        EXPECT_THAT(weight, t::Eq(kMaxTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(2));
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, &dummy_arg, kMaxTaskWeight, 2),
              t::Eq(fake_task));
  EXPECT_THAT(g_env.ipi_sent_count[2].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[2].load(), t::Eq(kVectorWakeupIpi));

  // 3. When CurrentCpuOrNull() == nullptr (unbound host thread), target_cpu 0
  //    is treated as remote and receives kVectorWakeupIpi (0x21).
  ResetCpuLocalForTest();
  ASSERT_THAT(CurrentCpuOrNull(), t::IsNull());
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::Eq(&NoopTask));
        EXPECT_THAT(arg, t::IsNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(0));
        return fake_task;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 0),
              t::Eq(fake_task));
  EXPECT_THAT(g_env.ipi_sent_count[0].load(), t::Eq(1));
  EXPECT_THAT(g_env.last_ipi_vector[0].load(), t::Eq(kVectorWakeupIpi));
}

TEST(WorkStealingSchedulerTest,
     YieldStealsFromBusiestOnlineDonorWhenLocalRunqueueEmptyAndSkipsWhenBusy) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Case 1: Local runqueue load > 0 -> delegates directly to inner->Yield()
  // without calling StealTask.
  int yield_calls = 0;
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(0));
    return 2;
  });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() { ++yield_calls; });

  sched->Yield();
  EXPECT_THAT(yield_calls, t::Eq(1));

  // Case 2: Local runqueue load == 0 -> attempts to steal from busiest online
  // remote donor before delegating to inner->Yield().
  // Mark CPU 3 offline despite high load so CPU 2 (load 4) is selected.
  g_env.cpu_locals[3].online = false;
  const int loads[4] = {0, 1, 4, 9};
  std::vector<std::string> call_order;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        call_order.push_back("StealTask(0,2)");
        return true;
      });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    call_order.push_back("Yield");
  });

  sched->Yield();
  EXPECT_THAT(call_order, t::ElementsAre("StealTask(0,2)", "Yield"));
}

TEST(
    WorkStealingSchedulerTest,
    PollIdleCpuRejectsOutOfRangeIndexAndDelegatesDirectlyWhenLocalLoadNonZero) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // 1. Out-of-range cpu_index returns false without touching mock_inner.
  EXPECT_FALSE(sched->PollIdleCpu(-1));
  EXPECT_FALSE(sched->PollIdleCpu(4));
  EXPECT_FALSE(sched->PollIdleCpu(99));

  // 2. When inner->RunqueueLoad(cpu_index) > 0, delegates directly to
  //    inner->PollIdleCpu(cpu_index) without calling StealTask.
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(2));
    return 3;
  });
  EXPECT_CALL(*mock_inner, PollIdleCpu).WillOnce([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(2));
    return true;
  });
  EXPECT_TRUE(sched->PollIdleCpu(2));
}

TEST(
    WorkStealingSchedulerTest,
    PollIdleCpuTriesDonorsInDescendingLoadOrderAndPollsOnlyAfterStealSucceeds) {
  SetupEnv(5);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // Mark CPU 4 offline (even with load 10, it must be ignored).
  g_env.cpu_locals[4].online = false;

  // Destination is CPU 2 (load 0). Online remote donors:
  // CPU 1 (load 5, busiest), CPU 3 (load 3, 2nd busiest), CPU 0 (load 1, 3rd).
  int loads[5] = {1, 5, 0, 3, 10};
  std::vector<std::pair<int, int>> steal_attempts;
  int polled_cpu = -1;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        return false;
      })
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        loads[src_cpu] -= 1;
        loads[dst_cpu] += 1;
        return true;
      });
  EXPECT_CALL(*mock_inner, PollIdleCpu).WillOnce([&](const int cpu_index) {
    polled_cpu = cpu_index;
    return true;
  });

  EXPECT_TRUE(sched->PollIdleCpu(2));
  EXPECT_THAT(steal_attempts,
              t::ElementsAre(std::make_pair(2, 1), std::make_pair(2, 3)));
  EXPECT_THAT(polled_cpu, t::Eq(2));

  // When all online remote donors have load > 0 but every StealTask call fails,
  // PollIdleCpu must NOT call inner->PollIdleCpu and must return false.
  loads[0] = 2;
  loads[1] = 4;
  loads[2] = 0;
  loads[3] = 2;
  steal_attempts.clear();

  EXPECT_CALL(*mock_inner, StealTask)
      .Times(3)
      .WillRepeatedly([&](const int dst_cpu, const int src_cpu) {
        steal_attempts.emplace_back(dst_cpu, src_cpu);
        return false;
      });

  EXPECT_FALSE(sched->PollIdleCpu(2));
  // Tie between CPU 0 (load 2) and CPU 3 (load 2) breaks toward lower cpu_id 0.
  EXPECT_THAT(steal_attempts,
              t::ElementsAre(std::make_pair(2, 1), std::make_pair(2, 0),
                             std::make_pair(2, 3)));

  // When all online remote donors have load 0, PollIdleCpu returns false
  // without calling StealTask or inner->PollIdleCpu.
  loads[0] = 0;
  loads[1] = 0;
  loads[2] = 0;
  loads[3] = 0;
  EXPECT_FALSE(sched->PollIdleCpu(2));
}

TEST(WorkStealingSchedulerTest,
     OnTimerInterruptAndOnWakeupIpiStealWhenOnlineAndIdleBeforeForwarding) {
  SetupEnv(3);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;

  // 1. CurrentCpuOrNull() is non-null, online, and local RunqueueLoad == 0:
  //    must attempt work stealing before forwarding frame to inner.
  const int idle_loads[3] = {0, 3, 1};
  std::vector<std::string> sequence;

  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    return idle_loads[cpu];
  });
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(1));
        sequence.push_back("TimerSteal");
        return true;
      });
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
        sequence.push_back("OnTimerInterrupt");
      });

  sched->OnTimerInterrupt(&timer_frame);
  EXPECT_THAT(sequence, t::ElementsAre("TimerSteal", "OnTimerInterrupt"));

  sequence.clear();
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(1));
        sequence.push_back("IpiSteal");
        return true;
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
        sequence.push_back("OnWakeupIpi");
      });

  sched->OnWakeupIpi(&ipi_frame);
  EXPECT_THAT(sequence, t::ElementsAre("IpiSteal", "OnWakeupIpi"));

  // 2. When local RunqueueLoad > 0: forwards directly without stealing.
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillRepeatedly([&](const int cpu) {
    EXPECT_THAT(cpu, t::Eq(0));
    return 1;
  });
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  sched->OnTimerInterrupt(&timer_frame);
  sched->OnWakeupIpi(&ipi_frame);

  // 3. When current CPU is offline: forwards directly without querying load or
  //    stealing.
  g_env.cpu_locals[0].online = false;
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  sched->OnTimerInterrupt(&timer_frame);
  sched->OnWakeupIpi(&ipi_frame);
  g_env.cpu_locals[0].online = true;

  // 4. When CurrentCpuOrNull() == nullptr: forwards directly without querying
  //    load or stealing.
  ResetCpuLocalForTest();
  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&timer_frame));
      });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce([&](InterruptFrame* const frame) {
        EXPECT_THAT(frame, t::Eq(&ipi_frame));
      });

  sched->OnTimerInterrupt(&timer_frame);
  sched->OnWakeupIpi(&ipi_frame);
}

TEST(WorkStealingSchedulerTest, DirectForwardingMethodsDelegateToInner) {
  SetupEnv(4);
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  Task* const fake_task = reinterpret_cast<Task*>(0x3001);

  // Join
  Task* joined_task = nullptr;
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    joined_task = task;
  });
  sched->Join(fake_task);
  EXPECT_THAT(joined_task, t::Eq(fake_task));

  // ReapZombies
  EXPECT_CALL(*mock_inner, ReapZombies).WillOnce([&]() { return 5; });
  EXPECT_THAT(sched->ReapZombies(), t::Eq(5));

  // RunqueueLoad
  EXPECT_CALL(*mock_inner, RunqueueLoad).WillOnce([&](const int cpu_index) {
    EXPECT_THAT(cpu_index, t::Eq(3));
    return 7;
  });
  EXPECT_THAT(sched->RunqueueLoad(3), t::Eq(7));

  // StealTask
  EXPECT_CALL(*mock_inner, StealTask)
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(1));
        EXPECT_THAT(src_cpu, t::Eq(2));
        return true;
      });
  EXPECT_TRUE(sched->StealTask(1, 2));

  // SetPreemptEnabled & IsPreemptEnabled
  std::vector<bool> preempt_states;
  EXPECT_CALL(*mock_inner, SetPreemptEnabled)
      .WillOnce([&](const bool enabled) { preempt_states.push_back(enabled); })
      .WillOnce([&](const bool enabled) { preempt_states.push_back(enabled); });
  sched->SetPreemptEnabled(false);
  sched->SetPreemptEnabled(true);
  EXPECT_THAT(preempt_states, t::ElementsAre(false, true));

  EXPECT_CALL(*mock_inner, IsPreemptEnabled)
      .WillOnce([&]() { return false; })
      .WillOnce([&]() { return true; });
  EXPECT_FALSE(sched->IsPreemptEnabled());
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // CurrentTask
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() { return fake_task; });
  EXPECT_THAT(sched->CurrentTask(), t::Eq(fake_task));
}

TEST(WorkStealingSchedulerDeathTest, PreconditionViolationsTriggerDcheck) {
  SetupEnv(2);

  // 1. Null inner scheduler passed to CreateWorkStealingScheduler.
  EXPECT_DEATH(CreateWorkStealingScheduler(nullptr), "Check failed");

  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateWorkStealingScheduler(mock_inner);

  // 2. Null entry function in CreateTask and CreateTaskOnCpu.
  EXPECT_DEATH(sched->CreateTask(nullptr, nullptr, kDefaultTaskWeight),
               "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(nullptr, nullptr, kDefaultTaskWeight, 0),
               "Check failed");

  // 3. Out-of-bounds weight in CreateTask and CreateTaskOnCpu.
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, 0), "Check failed");
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, -10), "Check failed");
  EXPECT_DEATH(sched->CreateTask(NoopTask, nullptr, kMaxTaskWeight + 1),
               "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, 0, 0), "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, -1, 0),
               "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kMaxTaskWeight + 1, 0),
               "Check failed");

  // 4. Negative or >= SmpCpuCount() target_cpu in CreateTaskOnCpu.
  EXPECT_DEATH(
      sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, -1),
      "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 2),
               "Check failed");

  // 5. Offline target_cpu (either local->online == false or SmpGetCpuLocal ==
  //    nullptr) in CreateTaskOnCpu.
  g_env.cpu_locals[1].online = false;
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
               "Check failed");
  g_env.cpu_locals[1].online = true;
  g_env.null_cpu_local[1] = true;
  EXPECT_DEATH(sched->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
               "Check failed");
}

class ForwardingSpyTaskScheduler final : public TaskScheduler {
 public:
  explicit ForwardingSpyTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {}

  void Init() override { inner_->Init(); }
  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    return inner_->CreateTask(entry, arg, weight);
  }
  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    return inner_->CreateTaskOnCpu(entry, arg, weight, target_cpu);
  }
  void Yield() override { inner_->Yield(); }
  void Join(Task* const task) override { inner_->Join(task); }
  int ReapZombies() override { return inner_->ReapZombies(); }
  bool PollIdleCpu(const int cpu_index) override {
    return inner_->PollIdleCpu(cpu_index);
  }
  int RunqueueLoad(const int cpu_index) const override {
    return inner_->RunqueueLoad(cpu_index);
  }
  bool StealTask(const int dst_cpu, const int src_cpu) override {
    if (src_cpu == 0) {
      steal_from_cpu0_calls_.fetch_add(1, std::memory_order_acq_rel);
    }
    return inner_->StealTask(dst_cpu, src_cpu);
  }
  void SetPreemptEnabled(const bool enabled) override {
    inner_->SetPreemptEnabled(enabled);
  }
  bool IsPreemptEnabled() const override { return inner_->IsPreemptEnabled(); }
  void OnTimerInterrupt(InterruptFrame* const frame) override {
    inner_->OnTimerInterrupt(frame);
  }
  void OnWakeupIpi(InterruptFrame* const frame) override {
    inner_->OnWakeupIpi(frame);
  }
  Task* CurrentTask() const override { return inner_->CurrentTask(); }

  int StealFromCpu0Calls() const {
    return steal_from_cpu0_calls_.load(std::memory_order_acquire);
  }

 private:
  const std::shared_ptr<TaskScheduler> inner_;
  std::atomic<int> steal_from_cpu0_calls_{0};
};

struct BootstrapLoadProbeCtx {
  TaskScheduler* sched = nullptr;
  ForwardingSpyTaskScheduler* spy = nullptr;
  int cpu0_load_with_only_bootstrap_queued = -1;
  int steal_calls_while_only_bootstrap_queued = -1;
  int cpu0_load_with_extra_worker_queued = -1;
  int steal_calls_after_extra_worker_queued = -1;
  bool ap1_poll_after_extra_worker = false;
  int extra_worker_runs = 0;
  Task* extra_task = nullptr;
};

static void ExtraStealableWorker(void* const raw_arg) {
  BootstrapLoadProbeCtx* const ctx =
      static_cast<BootstrapLoadProbeCtx*>(raw_arg);
  ++ctx->extra_worker_runs;
}

static void BootstrapLoadProbeWorker(void* const raw_arg) {
  BootstrapLoadProbeCtx* const ctx =
      static_cast<BootstrapLoadProbeCtx*>(raw_arg);
  // While `BootstrapLoadProbeWorker` is running on CPU 0, `bootstrap_task_` is
  // queued in CPU 0's runqueue tree waiting in `Join`. Because
  // `bootstrap_task_` is unstealable, `RunqueueLoad(0)` must be 0.
  ctx->cpu0_load_with_only_bootstrap_queued = ctx->sched->RunqueueLoad(0);

  // Idle APs 1, 2, 3 polling for work must see RunqueueLoad(0) == 0 and must
  // NOT call StealTask(dst_cpu, 0).
  for (int ap = 1; ap < 4; ++ap) {
    EXPECT_FALSE(ctx->sched->PollIdleCpu(ap));
  }
  ctx->steal_calls_while_only_bootstrap_queued = ctx->spy->StealFromCpu0Calls();

  // Now enqueue a second worker task on CPU 0 alongside `bootstrap_task_`.
  // `RunqueueLoad(0)` must become 1, and an idle AP must steal the second
  // worker task (skipping `bootstrap_task_`).
  ctx->extra_task = ctx->sched->CreateTaskOnCpu(ExtraStealableWorker,  //
                                                ctx,                   //
                                                kDefaultTaskWeight,    //
                                                0);
  ctx->cpu0_load_with_extra_worker_queued = ctx->sched->RunqueueLoad(0);
  BindCpuLocal(SmpGetCpuLocal(1));
  ctx->ap1_poll_after_extra_worker = ctx->sched->PollIdleCpu(1);
  BindCpuLocal(SmpGetCpuLocal(0));
  ctx->steal_calls_after_extra_worker_queued = ctx->spy->StealFromCpu0Calls();
}

TEST(WorkStealingSchedulerTest,
     IdleApsDoNotAttemptStealFromCpu0WhenOnlyBootstrapTaskIsQueued) {
  SetupEnv(4);
  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  const auto spy = std::make_shared<ForwardingSpyTaskScheduler>(core);
  const std::shared_ptr<TaskScheduler> sched = CreateWorkStealingScheduler(spy);
  sched->Init();

  BootstrapLoadProbeCtx ctx;
  ctx.sched = sched.get();
  ctx.spy = spy.get();

  Task* const worker = sched->CreateTaskOnCpu(BootstrapLoadProbeWorker,  //
                                              &ctx,                      //
                                              kDefaultTaskWeight,        //
                                              0);
  ASSERT_THAT(worker, t::NotNull());
  sched->Join(worker);
  ASSERT_THAT(ctx.extra_task, t::NotNull());
  sched->Join(ctx.extra_task);

  EXPECT_THAT(ctx.cpu0_load_with_only_bootstrap_queued, t::Eq(0));
  EXPECT_THAT(ctx.steal_calls_while_only_bootstrap_queued, t::Eq(0));
  EXPECT_THAT(ctx.cpu0_load_with_extra_worker_queued, t::Eq(1));
  EXPECT_TRUE(ctx.ap1_poll_after_extra_worker);
  EXPECT_THAT(ctx.steal_calls_after_extra_worker_queued, t::Eq(1));
  EXPECT_THAT(ctx.extra_worker_runs, t::Eq(1));
}

}  // namespace
}  // namespace protos
