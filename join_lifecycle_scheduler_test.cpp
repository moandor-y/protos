#include "join_lifecycle_scheduler.h"

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

constexpr int kMaxJoinRecordsForTest = 1024;

static Task* FakeTaskPtr(const int id) {
  return reinterpret_cast<Task*>(static_cast<uintptr_t>(0x10000 + id * 0x10));
}

struct CapturedTaskLaunch {
  TaskFn entry = nullptr;
  void* arg = nullptr;

  void RunToCompletion() const {
    ASSERT_THAT(entry, t::NotNull());
    entry(arg);
  }
};

// 1. Init() clears all join records and delegates to inner->Init().
TEST(JoinLifecycleSchedulerTest, InitClearsAllJoinRecordsAndDelegatesToInner) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const stale_zombie = FakeTaskPtr(1);
  Task* const stale_running = FakeTaskPtr(2);
  CapturedTaskLaunch zombie_launch;

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t weight) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        zombie_launch = {entry, arg};
        return stale_zombie;
      })
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t weight) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        return stale_running;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(stale_zombie));
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(stale_running));

  // Complete `stale_zombie` without reaping it.
  zombie_launch.RunToCompletion();

  // Calling Init() must clear all join records and forward to inner->Init().
  int init_calls = 0;
  EXPECT_CALL(*mock_inner, Init).WillOnce([&]() { ++init_calls; });
  sched->Init();
  EXPECT_THAT(init_calls, t::Eq(1));

  // Because Init() cleared the join record table, ReapZombies() must find 0
  // exited tasks and must not call inner->Join().
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));

  // Full capacity of kMaxJoinRecordsForTest slots is available after Init().
  for (int i = 0; i < kMaxJoinRecordsForTest; ++i) {
    Task* const task = FakeTaskPtr(100 + i);
    EXPECT_CALL(*mock_inner, CreateTask)
        .WillOnce([task](const TaskFn /*entry*/,  //
                         void* const /*arg*/,     //
                         const int64_t /*weight*/) { return task; });
    EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
                t::Eq(task));
  }
}

// 2. CreateTask and CreateTaskOnCpu delegate to inner, allocate/track a join
//    record for non-null returned Task* pointers, and release the record when
//    inner returns nullptr.
TEST(JoinLifecycleSchedulerTest,
     CreateTaskAndCreateTaskOnCpuTrackNonNullAndHandleNullptrCleanly) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  int observed_a = 0;
  int observed_b = 0;
  int arg_a = 11;
  int arg_b = 22;
  Task* const self_task = FakeTaskPtr(1);
  Task* const task_a = FakeTaskPtr(2);
  Task* const task_b = FakeTaskPtr(3);
  CapturedTaskLaunch launch_a;
  CapturedTaskLaunch launch_b;

  // Non-null CreateTask return.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t weight) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight * 2));
        launch_a = {entry, arg};
        return task_a;
      });
  EXPECT_THAT(sched->CreateTask(
                  [](void* const raw) { *static_cast<int*>(raw) += 100; },  //
                  &arg_a,                                                   //
                  kDefaultTaskWeight * 2),
              t::Eq(task_a));

  // Nullptr CreateTask return releases its pre-allocated join record slot.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t weight) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        return nullptr;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::IsNull());

  // Non-null CreateTaskOnCpu return.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight / 2));
        EXPECT_THAT(target_cpu, t::Eq(3));
        launch_b = {entry, arg};
        return task_b;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(
                  [](void* const raw) { *static_cast<int*>(raw) += 200; },  //
                  &arg_b,                                                   //
                  kDefaultTaskWeight / 2,                                   //
                  3),
              t::Eq(task_b));

  // Nullptr CreateTaskOnCpu return releases its pre-allocated join record slot.
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,    //
                    void* const arg,       //
                    const int64_t weight,  //
                    const int target_cpu) {
        EXPECT_THAT(entry, t::NotNull());
        EXPECT_THAT(arg, t::NotNull());
        EXPECT_THAT(weight, t::Eq(kDefaultTaskWeight));
        EXPECT_THAT(target_cpu, t::Eq(1));
        return nullptr;
      });
  EXPECT_THAT(sched->CreateTaskOnCpu(NoopTask,            //
                                     nullptr,             //
                                     kDefaultTaskWeight,  //
                                     1),
              t::IsNull());

  // Verify both `task_a` and `task_b` run their user callbacks and mark exited
  // when their wrapped entry returns.
  launch_a.RunToCompletion();
  launch_b.RunToCompletion();
  observed_a = arg_a;
  observed_b = arg_b;
  EXPECT_THAT(observed_a, t::Eq(111));
  EXPECT_THAT(observed_b, t::Eq(222));

  std::vector<Task*> joined_tasks;
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    joined_tasks.push_back(task);
  });

  sched->Join(task_a);
  sched->Join(task_b);
  EXPECT_THAT(joined_tasks, t::ElementsAre(task_a, task_b));
}

// 3. Join(task) when task has already finished before Join(task) is called:
//    immediately delegates to inner->Join(task) without calling inner->Yield().
TEST(JoinLifecycleSchedulerTest,
     JoinWhenTaskAlreadyExitedDelegatesImmediatelyWithoutYielding) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(10);
  Task* const target_task = FakeTaskPtr(11);
  Task* joined_task = nullptr;
  CapturedTaskLaunch target_launch;

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        target_launch = {entry, arg};
        return target_task;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(target_task));

  // Target finishes before Join(target_task) is invoked.
  target_launch.RunToCompletion();

  // StrictMock ensures inner->Yield() is NEVER called during Join().
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() { return self_task; });
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    joined_task = task;
  });

  sched->Join(target_task);
  EXPECT_THAT(joined_task, t::Eq(target_task));

  // Join record slot was freed by Join(), so ReapZombies() finds nothing.
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
}

// 4. Join(task) when task has not yet exited: calls inner->Yield() repeatedly
//    until task finishes its entry callback, then delegates to
//    inner->Join(task) and frees the join record slot.
TEST(JoinLifecycleSchedulerTest,
     JoinWhenTaskRunningYieldsRepeatedlyUntilExitAndFreesSlot) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(20);
  Task* const target_task = FakeTaskPtr(21);
  int yield_count = 0;
  constexpr int kYieldsBeforeExit = 4;
  Task* joined_task = nullptr;
  CapturedTaskLaunch target_launch;

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        target_launch = {entry, arg};
        return target_task;
      });
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(target_task));

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Yield).WillRepeatedly([&]() {
    ++yield_count;
    if (yield_count == kYieldsBeforeExit) {
      target_launch.RunToCompletion();
    }
  });
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(yield_count, t::Eq(kYieldsBeforeExit));
    joined_task = task;
  });

  sched->Join(target_task);
  EXPECT_THAT(yield_count, t::Eq(kYieldsBeforeExit));
  EXPECT_THAT(joined_task, t::Eq(target_task));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
}

// 5. ReapZombies(): reaps all exited, unjoined tasks that have no active joiner
//    by calling inner->Join(zombie) and returning the count of reaped tasks,
//    while leaving still-running tasks and tasks currently being joined
//    untouched; subsequent ReapZombies() returns 0.
TEST(JoinLifecycleSchedulerTest,
     ReapZombiesReapsOnlyUnjoinedExitedTasksAndSkipsRunningOrBeingJoined) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const joiner_task = FakeTaskPtr(30);
  Task* const zombie_1 = FakeTaskPtr(31);
  Task* const running_task = FakeTaskPtr(32);
  Task* const zombie_2 = FakeTaskPtr(33);
  Task* const actively_joined_task = FakeTaskPtr(34);

  CapturedTaskLaunch launch_z1;
  CapturedTaskLaunch launch_running;
  CapturedTaskLaunch launch_z2;
  CapturedTaskLaunch launch_active;

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        launch_z1 = {entry, arg};
        return zombie_1;
      })
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        launch_running = {entry, arg};
        return running_task;
      })
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        launch_z2 = {entry, arg};
        return zombie_2;
      })
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        launch_active = {entry, arg};
        return actively_joined_task;
      });

  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(zombie_1));
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(running_task));
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(zombie_2));
  EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(actively_joined_task));

  // `zombie_1` and `zombie_2` finish without any joiner.
  launch_z1.RunToCompletion();
  launch_z2.RunToCompletion();

  // Start `Join(actively_joined_task)` on `joiner_task`. Inside `Yield()`,
  // `actively_joined_task` finishes AND `ReapZombies()` runs before `Join()`
  // finishes. `ReapZombies()` must reap `zombie_1` and `zombie_2`, skip
  // `running_task` (not exited) and skip `actively_joined_task` (exited, but
  // has an active `joiner`).
  std::vector<Task*> reaped_during_yield;
  int reap_count_during_yield = -1;
  int reap_count_second_call = -1;
  Task* joined_by_joiner = nullptr;

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return joiner_task;
  });
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() {
    launch_active.RunToCompletion();
    reap_count_during_yield = sched->ReapZombies();
    reap_count_second_call = sched->ReapZombies();
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    if (reap_count_during_yield < 0) {
      reaped_during_yield.push_back(task);
    } else {
      joined_by_joiner = task;
    }
  });

  sched->Join(actively_joined_task);

  EXPECT_THAT(reap_count_during_yield, t::Eq(2));
  EXPECT_THAT(reaped_during_yield, t::ElementsAre(zombie_1, zombie_2));
  EXPECT_THAT(reap_count_second_call, t::Eq(0));
  EXPECT_THAT(joined_by_joiner, t::Eq(actively_joined_task));

  // `running_task` was left untouched by ReapZombies(); once it finishes,
  // ReapZombies() reaps it and subsequent ReapZombies() returns 0.
  launch_running.RunToCompletion();

  Task* reaped_running = nullptr;
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    reaped_running = task;
  });
  EXPECT_THAT(sched->ReapZombies(), t::Eq(1));
  EXPECT_THAT(reaped_running, t::Eq(running_task));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
}

// 6. Join record table capacity & slot reuse: creating, finishing, and
//    joining/reaping more than 1024 (kMaxJoinRecords) tasks over time succeeds
//    by reusing freed join record slots.
TEST(JoinLifecycleSchedulerTest,
     JoinRecordTableCapacityAndSlotReuseAcrossMoreThanMaxJoinRecords) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(1);
  int joined_count = 0;

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::NotNull());
    ++joined_count;
  });

  // Fill all 1024 slots simultaneously, then finish and join half (512) and
  // reap the other half (512), repeated over 3 full waves (3072 tasks total).
  constexpr int kWaves = 3;
  for (int wave = 0; wave < kWaves; ++wave) {
    std::vector<Task*> wave_tasks;
    std::vector<CapturedTaskLaunch> wave_launches;
    wave_tasks.reserve(kMaxJoinRecordsForTest);
    wave_launches.reserve(kMaxJoinRecordsForTest);

    for (int i = 0; i < kMaxJoinRecordsForTest; ++i) {
      Task* const task = FakeTaskPtr(1000 + wave * kMaxJoinRecordsForTest + i);
      CapturedTaskLaunch launch;
      EXPECT_CALL(*mock_inner, CreateTask)
          .WillOnce([task, &launch](const TaskFn entry,  //
                                    void* const arg,     //
                                    const int64_t /*weight*/) {
            launch = {entry, arg};
            return task;
          });
      EXPECT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
                  t::Eq(task));
      wave_tasks.push_back(task);
      wave_launches.push_back(launch);
    }

    for (int i = 0; i < kMaxJoinRecordsForTest; ++i) {
      wave_launches[i].RunToCompletion();
    }

    const int half = kMaxJoinRecordsForTest / 2;
    for (int i = 0; i < half; ++i) {
      sched->Join(wave_tasks[i]);
    }
    EXPECT_THAT(sched->ReapZombies(), t::Eq(kMaxJoinRecordsForTest - half));
    EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  }

  EXPECT_THAT(joined_count, t::Eq(kWaves * kMaxJoinRecordsForTest));
}

// 7. Multi-threaded concurrency test: one thread runs Join(task)
//    spinning/yielding while another thread concurrently finishes task's
//    wrapped entry, verifying thread-safe synchronization under TSAN.
TEST(JoinLifecycleSchedulerTest,
     ConcurrentJoinAndTaskCompletionAcrossThreadsSynchronizesCleanlyUnderTsan) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  constexpr int kNumPairs = 32;
  Task* const joiner_task = FakeTaskPtr(1);
  thread_local Task* tls_current_task = nullptr;

  std::atomic<int> total_yields{0};
  std::atomic<int> total_completions{0};
  std::atomic<int> total_joins{0};

  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return tls_current_task;
  });
  EXPECT_CALL(*mock_inner, Yield).WillRepeatedly([&]() {
    total_yields.fetch_add(1, std::memory_order_relaxed);
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::NotNull());
    total_joins.fetch_add(1, std::memory_order_acq_rel);
  });

  for (int round = 0; round < kNumPairs; ++round) {
    Task* const worker_task = FakeTaskPtr(100 + round);
    CapturedTaskLaunch worker_launch;
    EXPECT_CALL(*mock_inner, CreateTask)
        .WillOnce([worker_task, &worker_launch](const TaskFn entry,  //
                                                void* const arg,     //
                                                const int64_t /*weight*/) {
          worker_launch = {entry, arg};
          return worker_task;
        });
    ASSERT_THAT(sched->CreateTask(
                    [](void* const raw) {
                      static_cast<std::atomic<int>*>(raw)->fetch_add(
                          1, std::memory_order_acq_rel);
                    },
                    &total_completions, kDefaultTaskWeight),
                t::Eq(worker_task));

    std::atomic<bool> join_entered_yield{false};
    const int yields_before = total_yields.load(std::memory_order_relaxed);

    std::thread worker_thread([&]() {
      SetInterruptsEnabledForTest(true);
      tls_current_task = worker_task;
      while (total_yields.load(std::memory_order_acquire) <= yields_before &&
             !join_entered_yield.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      worker_launch.RunToCompletion();
    });

    tls_current_task = joiner_task;
    join_entered_yield.store(true, std::memory_order_release);
    sched->Join(worker_task);
    worker_thread.join();
  }

  EXPECT_THAT(total_completions.load(std::memory_order_acquire),
              t::Eq(kNumPairs));
  EXPECT_THAT(total_joins.load(std::memory_order_acquire), t::Eq(kNumPairs));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
}

// 8. Direct forwarding of Yield, PollIdleCpu, RunqueueLoad, StealTask,
//    SetPreemptEnabled, IsPreemptEnabled, OnTimerInterrupt, OnWakeupIpi, and
//    CurrentTask to inner.
TEST(JoinLifecycleSchedulerTest, DirectlyForwardsPassThroughMethodsToInner) {
  ResetTestEnv();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  // Yield
  int yield_calls = 0;
  EXPECT_CALL(*mock_inner, Yield).WillOnce([&]() { ++yield_calls; });
  sched->Yield();
  EXPECT_THAT(yield_calls, t::Eq(1));

  // PollIdleCpu
  EXPECT_CALL(*mock_inner, PollIdleCpu)
      .WillOnce([&](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(2));
        return true;
      })
      .WillOnce([&](const int cpu_index) {
        EXPECT_THAT(cpu_index, t::Eq(0));
        return false;
      });
  EXPECT_TRUE(sched->PollIdleCpu(2));
  EXPECT_FALSE(sched->PollIdleCpu(0));

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
        EXPECT_THAT(src_cpu, t::Eq(3));
        return true;
      })
      .WillOnce([&](const int dst_cpu, const int src_cpu) {
        EXPECT_THAT(dst_cpu, t::Eq(0));
        EXPECT_THAT(src_cpu, t::Eq(2));
        return false;
      });
  EXPECT_TRUE(sched->StealTask(1, 3));
  EXPECT_FALSE(sched->StealTask(0, 2));

  // SetPreemptEnabled & IsPreemptEnabled
  bool preempt_state = true;
  EXPECT_CALL(*mock_inner, SetPreemptEnabled)
      .WillRepeatedly([&](const bool enabled) { preempt_state = enabled; });
  EXPECT_CALL(*mock_inner, IsPreemptEnabled).WillRepeatedly([&]() {
    return preempt_state;
  });
  sched->SetPreemptEnabled(false);
  EXPECT_FALSE(sched->IsPreemptEnabled());
  sched->SetPreemptEnabled(true);
  EXPECT_TRUE(sched->IsPreemptEnabled());

  // OnTimerInterrupt & OnWakeupIpi
  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;
  InterruptFrame* observed_timer_frame = nullptr;
  InterruptFrame* observed_ipi_frame = nullptr;

  EXPECT_CALL(*mock_inner, OnTimerInterrupt)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_timer_frame = frame; });
  EXPECT_CALL(*mock_inner, OnWakeupIpi)
      .WillOnce(
          [&](InterruptFrame* const frame) { observed_ipi_frame = frame; });

  sched->OnTimerInterrupt(&timer_frame);
  EXPECT_THAT(observed_timer_frame, t::Eq(&timer_frame));
  sched->OnWakeupIpi(&ipi_frame);
  EXPECT_THAT(observed_ipi_frame, t::Eq(&ipi_frame));

  // CurrentTask
  Task* const expected_current = FakeTaskPtr(99);
  EXPECT_CALL(*mock_inner, CurrentTask).WillOnce([&]() {
    return expected_current;
  });
  EXPECT_THAT(sched->CurrentTask(), t::Eq(expected_current));
}

// 9. DCHECK death tests (JoinLifecycleSchedulerDeathTest) verifying
//    preconditions: null inner in CreateJoinLifecycleScheduler, null entry in
//    CreateTask/CreateTaskOnCpu, null task in Join, CurrentTask() == nullptr in
//    Join, self-join (Join(self)), joining an unknown/unregistered Task*,
//    double-joining an already-joined Task*, and calling Join or returning from
//    a task while holding an IrqSpinLock.
TEST(JoinLifecycleSchedulerDeathTest, PreconditionViolationsTriggerDcheck) {
  ResetTestEnv();

  // 1. Null inner in CreateJoinLifecycleScheduler.
  EXPECT_DEATH(CreateJoinLifecycleScheduler(nullptr), "Check failed");

  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  EXPECT_DEATH(sched->CreateTask(nullptr, nullptr, kDefaultTaskWeight),
               "Check failed");
  EXPECT_DEATH(sched->CreateTaskOnCpu(nullptr, nullptr, kDefaultTaskWeight, 0),
               "Check failed");

  Task* const self_task = FakeTaskPtr(1);
  Task* const target_task = FakeTaskPtr(2);
  Task* const unknown_task = FakeTaskPtr(3);
  CapturedTaskLaunch target_launch;

  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        target_launch = {entry, arg};
        return target_task;
      });
  ASSERT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
              t::Eq(target_task));

  // Complete `target_task` so valid `Join(target_task)` can complete.
  target_launch.RunToCompletion();

  // 2. Null task in Join(nullptr).
  EXPECT_DEATH(sched->Join(nullptr), "Check failed");

  // 3. CurrentTask() == nullptr in Join.
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([]() {
    return nullptr;
  });
  EXPECT_DEATH(sched->Join(target_task), "Check failed");

  // Restore CurrentTask() == self_task for remaining checks.
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });

  // 4. Self-join: Join(self).
  EXPECT_DEATH(sched->Join(self_task), "Check failed");

  // 5. Joining an unknown/unregistered Task*.
  EXPECT_DEATH(sched->Join(unknown_task), "Check failed");

  // 6. Calling Join while holding an IrqSpinLock.
  EXPECT_DEATH(
      {
        IrqSpinLock held_lock;
        const IrqSpinLockGuard guard(held_lock);
        sched->Join(target_task);
      },
      "Check failed");

  // 7. Double-joining an already-joined Task*.
  EXPECT_CALL(*mock_inner, Join).WillOnce([&](Task* const task) {
    EXPECT_THAT(task, t::Eq(target_task));
  });
  sched->Join(target_task);
  EXPECT_DEATH(sched->Join(target_task), "Check failed");
}

// 10. Same-CPU preemption / synchronous child completion inside CreateTask and
//     CreateTaskOnCpu before inner returns the Task* pointer must not spin-wait
//     or livelock in MarkRecordExited.
TEST(JoinLifecycleSchedulerTest,
     SynchronousTaskCompletionInsideCreateTaskDoesNotLivelock) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  Task* const self_task = FakeTaskPtr(1);
  Task* const sync_task_1 = FakeTaskPtr(2);
  Task* const sync_task_2 = FakeTaskPtr(3);
  int callback_runs = 0;

  // Simulate same-CPU preemption where the newly created task is scheduled and
  // runs to completion inside inner_->CreateTask BEFORE CreateTask returns.
  EXPECT_CALL(*mock_inner, CreateTask)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/) {
        entry(arg);
        return sync_task_1;
      });
  EXPECT_CALL(*mock_inner, CreateTaskOnCpu)
      .WillOnce([&](const TaskFn entry,  //
                    void* const arg,     //
                    const int64_t /*weight*/, const int /*target_cpu*/) {
        entry(arg);
        return sync_task_2;
      });

  EXPECT_THAT(sched->CreateTask(
                  [](void* const raw) { ++(*static_cast<int*>(raw)); },  //
                  &callback_runs,                                        //
                  kDefaultTaskWeight),
              t::Eq(sync_task_1));
  EXPECT_THAT(sched->CreateTaskOnCpu(
                  [](void* const raw) { ++(*static_cast<int*>(raw)); },  //
                  &callback_runs,                                        //
                  kDefaultTaskWeight,                                    //
                  0),
              t::Eq(sync_task_2));
  EXPECT_THAT(callback_runs, t::Eq(2));

  // Join sync_task_1 immediately (must not yield) and reap sync_task_2 via
  // ReapZombies().
  std::vector<Task*> joined_tasks;
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    joined_tasks.push_back(task);
  });

  sched->Join(sync_task_1);
  EXPECT_THAT(sched->ReapZombies(), t::Eq(1));
  EXPECT_THAT(joined_tasks, t::ElementsAre(sync_task_1, sync_task_2));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// 11. Dynamic scaling beyond 1024 concurrent active/unjoined tasks without
//     hitting a fixed table limit or leaking heap memory.
TEST(JoinLifecycleSchedulerTest,
     SupportsMoreThan1024ConcurrentUnjoinedTasksSimultaneously) {
  ResetTestEnv();
  const int64_t heap_before = HeapTotalFreeBytes();
  const auto mock_inner = std::make_shared<t::StrictMock<MockTaskScheduler>>();
  const std::shared_ptr<TaskScheduler> sched =
      CreateJoinLifecycleScheduler(mock_inner);

  constexpr int kNumConcurrentTasks = 1500;
  Task* const self_task = FakeTaskPtr(1);
  std::vector<Task*> tasks;
  std::vector<CapturedTaskLaunch> launches;
  tasks.reserve(kNumConcurrentTasks);
  launches.reserve(kNumConcurrentTasks);

  for (int i = 0; i < kNumConcurrentTasks; ++i) {
    Task* const task = FakeTaskPtr(5000 + i);
    CapturedTaskLaunch launch;
    EXPECT_CALL(*mock_inner, CreateTask)
        .WillOnce([task, &launch](const TaskFn entry,  //
                                  void* const arg,     //
                                  const int64_t /*weight*/) {
          launch = {entry, arg};
          return task;
        });
    ASSERT_THAT(sched->CreateTask(NoopTask, nullptr, kDefaultTaskWeight),
                t::Eq(task));
    tasks.push_back(task);
    launches.push_back(launch);
  }

  for (int i = 0; i < kNumConcurrentTasks; ++i) {
    launches[i].RunToCompletion();
  }

  int inner_join_calls = 0;
  EXPECT_CALL(*mock_inner, CurrentTask).WillRepeatedly([&]() {
    return self_task;
  });
  EXPECT_CALL(*mock_inner, Join).WillRepeatedly([&](Task* const task) {
    EXPECT_THAT(task, t::NotNull());
    ++inner_join_calls;
  });

  const int half = kNumConcurrentTasks / 2;
  for (int i = 0; i < half; ++i) {
    sched->Join(tasks[i]);
  }
  EXPECT_THAT(sched->ReapZombies(), t::Eq(kNumConcurrentTasks - half));
  EXPECT_THAT(sched->ReapZombies(), t::Eq(0));
  EXPECT_THAT(inner_join_calls, t::Eq(kNumConcurrentTasks));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

}  // namespace
}  // namespace protos
