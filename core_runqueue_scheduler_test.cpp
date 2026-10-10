#include "core_runqueue_scheduler.h"

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "scheduler_test_support.h"

namespace protos {
namespace {

namespace t = ::testing;

// ============================================================================
// 1. `Init()` and Topology Initialization Across 1..4 CPUs
// ============================================================================

TEST(CoreRunqueueSchedulerTest,
     InitAdoptsBootstrapTaskAndInitializesPerCpuRunqueuesAcrossTopologies) {
  for (const int num_cpus : {1, 2, 3, 4, 128}) {
    SetupEnv(num_cpus);
    // Unbind CpuLocal before Init() to verify Init() automatically binds BSP
    // CpuLocal(0) when unbound.
    ResetCpuLocalForTest();
    EXPECT_TRUE(IdtIsInitialized());

    const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
    ASSERT_THAT(core, t::NotNull());
    // Before Init(), PollIdleCpu must safely return false.
    EXPECT_FALSE(core->PollIdleCpu(0));

    core->Init();

    EXPECT_THAT(CurrentCpuId(), t::Eq(0));

    // Preemption state on the core layer is always enabled and
    // SetPreemptEnabled is a safe no-op.
    EXPECT_TRUE(core->IsPreemptEnabled());
    core->SetPreemptEnabled(false);
    EXPECT_TRUE(core->IsPreemptEnabled());
    core->SetPreemptEnabled(true);
    EXPECT_TRUE(core->IsPreemptEnabled());

    Task* const bootstrap = core->CurrentTask();
    ASSERT_THAT(bootstrap, t::NotNull());

    for (int c = 0; c < num_cpus; ++c) {
      EXPECT_THAT(core->RunqueueLoad(c), t::Eq(0));
      EXPECT_FALSE(core->PollIdleCpu(c));
    }
    // Out-of-range PollIdleCpu returns false safely.
    EXPECT_FALSE(core->PollIdleCpu(-1));
    EXPECT_FALSE(core->PollIdleCpu(num_cpus));

    // Verify each AP (1 .. num_cpus - 1) has its own distinct idle task bound
    // as current_task.
    std::vector<Task*> initial_tasks;
    initial_tasks.push_back(bootstrap);
    for (int c = 1; c < num_cpus; ++c) {
      BindCpuLocal(SmpGetCpuLocal(c));
      Task* const ap_idle = core->CurrentTask();
      ASSERT_THAT(ap_idle, t::NotNull());
      for (Task* const prev_task : initial_tasks) {
        EXPECT_THAT(ap_idle, t::Ne(prev_task));
      }
      initial_tasks.push_back(ap_idle);
    }
    BindCpuLocal(SmpGetCpuLocal(0));
  }
}

// ============================================================================
// 2. `CreateTask`, `CreateTaskOnCpu`, Stack Alignment, `Yield`, `Exit`
// ============================================================================

struct LifecycleTaskCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> trace;
  bool explicit_exit = false;
  int task_index = 0;
  bool stack_aligned = false;
  Task* observed_task = nullptr;
  int observed_cpu = -1;
};

static void LifecycleWorker(void* const raw_arg) {
  LifecycleTaskCtx* const ctx = static_cast<LifecycleTaskCtx*>(raw_arg);
  ASSERT_THAT(ctx, t::NotNull());
  ASSERT_THAT(ctx->sched, t::NotNull());

  alignas(16) uint64_t probe = 0;
  ctx->stack_aligned = ((reinterpret_cast<uintptr_t>(&probe) & 0xFu) == 0);
  ctx->observed_task = ctx->sched->CurrentTask();
  ctx->observed_cpu = CurrentCpuId();

  for (int step = 0; step < 3; ++step) {
    ctx->trace.push_back(ctx->task_index * 10 + step);
    ctx->sched->Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     CreateTaskAndCreateTaskOnCpuVerifyAlignmentIdentityYieldAndExitModes) {
  SetupEnv(3);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  core->Init();

  LifecycleTaskCtx ctx_implicit;
  ctx_implicit.sched = core.get();
  ctx_implicit.task_index = 1;
  ctx_implicit.explicit_exit = false;

  LifecycleTaskCtx ctx_explicit;
  ctx_explicit.sched = core.get();
  ctx_explicit.task_index = 2;
  ctx_explicit.explicit_exit = true;

  LifecycleTaskCtx ctx_remote;
  ctx_remote.sched = core.get();
  ctx_remote.task_index = 3;
  ctx_remote.explicit_exit = false;

  // CreateTask always places onto CPU 0's runqueue.
  Task* const task_a =
      core->CreateTask(LifecycleWorker, &ctx_implicit, kDefaultTaskWeight);
  Task* const task_b = core->CreateTaskOnCpu(LifecycleWorker,     //
                                             &ctx_explicit,       //
                                             kDefaultTaskWeight,  //
                                             0);
  // CreateTaskOnCpu places onto the explicit target_cpu (CPU 2) without sending
  // any IPIs (IPIs belong to WorkStealingScheduler).
  Task* const task_c = core->CreateTaskOnCpu(LifecycleWorker,     //
                                             &ctx_remote,         //
                                             kDefaultTaskWeight,  //
                                             2);

  ASSERT_THAT(task_a, t::NotNull());
  ASSERT_THAT(task_b, t::NotNull());
  ASSERT_THAT(task_c, t::NotNull());
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(2));
  EXPECT_THAT(core->RunqueueLoad(1), t::Eq(0));
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(1));
  for (int c = 0; c < 3; ++c) {
    EXPECT_THAT(g_env.ipi_sent_count[c].load(), t::Eq(0));
  }

  core->Join(task_a);
  core->Join(task_b);
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(0));

  EXPECT_TRUE(ctx_implicit.stack_aligned);
  EXPECT_THAT(ctx_implicit.observed_task, t::Eq(task_a));
  EXPECT_THAT(ctx_implicit.observed_cpu, t::Eq(0));
  EXPECT_THAT(ctx_implicit.trace, t::ElementsAre(10, 11, 12));

  EXPECT_TRUE(ctx_explicit.stack_aligned);
  EXPECT_THAT(ctx_explicit.observed_task, t::Eq(task_b));
  EXPECT_THAT(ctx_explicit.observed_cpu, t::Eq(0));
  EXPECT_THAT(ctx_explicit.trace, t::ElementsAre(20, 21, 22));

  // Steal `task_c` from CPU 2 onto CPU 0 and run it to completion.
  EXPECT_TRUE(core->StealTask(0, 2));
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(0));
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(1));
  core->Join(task_c);

  EXPECT_TRUE(ctx_remote.stack_aligned);
  EXPECT_THAT(ctx_remote.observed_task, t::Eq(task_c));
  EXPECT_THAT(ctx_remote.observed_cpu, t::Eq(0));
  EXPECT_THAT(ctx_remote.trace, t::ElementsAre(30, 31, 32));

  // Verify `CurrentTask()` and `Yield()` disable interrupts internally before
  // querying `GetCurrentCpuId()` and restore the caller's prior interrupt
  // state (both when called with interrupts enabled and when disabled).
  SetInterruptsEnabledForTest(true);
  Task* const curr_irq_on = core->CurrentTask();
  EXPECT_TRUE(AreInterruptsEnabled());
  core->Yield();
  EXPECT_TRUE(AreInterruptsEnabled());

  SetInterruptsEnabledForTest(false);
  Task* const curr_irq_off = core->CurrentTask();
  EXPECT_FALSE(AreInterruptsEnabled());
  core->Yield();
  EXPECT_FALSE(AreInterruptsEnabled());
  SetInterruptsEnabledForTest(true);
  EXPECT_THAT(curr_irq_on, t::Eq(curr_irq_off));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 3. Weighted `RbTree` `vruntime` Ordering and Inverse-Weight Scaling
// ============================================================================

struct WeightedShareCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> dispatch_order;
  int slices_for_task[3] = {};
  int slices_when_high_finished[3] = {};
};

struct WeightedShareTaskArg {
  WeightedShareCtx* shared = nullptr;
  int index = 0;
};

static void WeightedShareWorker(void* const raw_arg) {
  WeightedShareTaskArg* const arg = static_cast<WeightedShareTaskArg*>(raw_arg);
  WeightedShareCtx* const shared = arg->shared;
  const int idx = arg->index;

  for (int i = 0; i < 8; ++i) {
    shared->dispatch_order.push_back(idx);
    ++shared->slices_for_task[idx];
    if (idx == 0 && i == 7) {
      for (int k = 0; k < 3; ++k) {
        shared->slices_when_high_finished[k] = shared->slices_for_task[k];
      }
    }
    shared->sched->Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     WeightedRbTreeVruntimeOrderingProportionalShareAndFifoTieBreaking) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  core->Init();

  WeightedShareCtx shared;
  shared.sched = core.get();
  WeightedShareTaskArg args[3] = {
      {&shared, 0},
      {&shared, 1},
      {&shared, 2},
  };

  // Create three tasks on CPU 0 with weights 2048 (2x), 1024 (1x), and 512
  // (0.5x). All three start at min_vruntime == 0 and are ordered initially by
  // FIFO enqueue_seq (0, 1, 2).
  Task* const t_high =
      core->CreateTask(WeightedShareWorker, &args[0], kDefaultTaskWeight * 2);
  Task* const t_med =
      core->CreateTask(WeightedShareWorker, &args[1], kDefaultTaskWeight);
  Task* const t_low =
      core->CreateTask(WeightedShareWorker, &args[2], kDefaultTaskWeight / 2);

  core->Join(t_high);
  core->Join(t_med);
  core->Join(t_low);

  // First 3 dispatches must follow initial FIFO enqueue_seq tie-breaking
  // (0, 1, 2) because all 3 started at vruntime == 0.
  ASSERT_THAT(static_cast<int>(shared.dispatch_order.size()), t::Eq(24));
  EXPECT_THAT(shared.dispatch_order[0], t::Eq(0));
  EXPECT_THAT(shared.dispatch_order[1], t::Eq(1));
  EXPECT_THAT(shared.dispatch_order[2], t::Eq(2));

  // When t_high (weight 2048, +512/slice) completes its 8th slice (at
  // vruntime == 4096), t_med (weight 1024, +1024/slice) has completed 4 slices
  // and t_low (weight 512, +2048/slice) has completed 2 slices (4:2:1 ratio).
  EXPECT_THAT(shared.slices_when_high_finished[0], t::Eq(8));
  EXPECT_THAT(shared.slices_when_high_finished[1], t::Eq(4));
  EXPECT_THAT(shared.slices_when_high_finished[2], t::Eq(2));

  int high_finish_pos = -1;
  int med_finish_pos = -1;
  int low_finish_pos = -1;
  for (int i = 0; i < static_cast<int>(shared.dispatch_order.size()); ++i) {
    if (shared.dispatch_order[i] == 0) {
      high_finish_pos = i;
    } else if (shared.dispatch_order[i] == 1) {
      med_finish_pos = i;
    } else if (shared.dispatch_order[i] == 2) {
      low_finish_pos = i;
    }
  }
  EXPECT_THAT(high_finish_pos, t::Lt(med_finish_pos));
  EXPECT_THAT(med_finish_pos, t::Lt(low_finish_pos));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 4. Monotonic `min_vruntime` Floor Clamping on Create and `StealTask`
// ============================================================================

struct MinVruntimeFloorCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> create_trace;
  std::vector<int> migrate_trace;
  Task* newborn = nullptr;
};

static void NewbornFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  for (int i = 0; i < 4; ++i) {
    ctx->create_trace.push_back(2);
    ctx->sched->Yield();
  }
}

static void VeteranFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);

  // Advance CPU 0's min_vruntime by 10 slices before creating `newborn`.
  for (int i = 0; i < 10; ++i) {
    ctx->sched->Yield();
  }

  ctx->newborn = ctx->sched->CreateTaskOnCpu(NewbornFloorWorker,  //
                                             ctx,                 //
                                             kDefaultTaskWeight,  //
                                             0);
  for (int i = 0; i < 4; ++i) {
    ctx->create_trace.push_back(1);
    ctx->sched->Yield();
  }
}

static void MigratedFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(20);
    ctx->sched->Yield();
  }
}

static void LocalPeerFloorWorker(void* const raw_arg) {
  MinVruntimeFloorCtx* const ctx = static_cast<MinVruntimeFloorCtx*>(raw_arg);
  for (int i = 0; i < 4; ++i) {
    ctx->migrate_trace.push_back(10);
    ctx->sched->Yield();
  }
}

TEST(CoreRunqueueSchedulerTest,
     MonotonicMinVruntimeFloorClampingOnCreationAndCrossCpuSteal) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  core->Init();

  MinVruntimeFloorCtx ctx;
  ctx.sched = core.get();

  // (a) Creation floor: `veteran` advances CPU 0's min_vruntime by 10 yields,
  //     then spawns `newborn` on CPU 0. Because `newborn`'s vruntime is
  //     clamped to CPU 0's min_vruntime (instead of 0), `newborn` does not
  //     starve `veteran` for 4 consecutive slices — they interleave.
  Task* const veteran = core->CreateTaskOnCpu(VeteranFloorWorker,  //
                                              &ctx,                //
                                              kDefaultTaskWeight,  //
                                              0);
  core->Join(veteran);
  ASSERT_THAT(ctx.newborn, t::NotNull());
  core->Join(ctx.newborn);

  ASSERT_THAT(static_cast<int>(ctx.create_trace.size()), t::Eq(8));
  int max_consecutive_newborn = 0;
  int current_run = 0;
  for (const int id : ctx.create_trace) {
    if (id == 2) {
      ++current_run;
      if (current_run > max_consecutive_newborn) {
        max_consecutive_newborn = current_run;
      }
    } else {
      current_run = 0;
    }
  }
  EXPECT_THAT(max_consecutive_newborn, t::Le(2));

  // (b) Cross-CPU migration floor: create `remote_task` on CPU 1 (where
  //     min_vruntime == 0), then steal it onto CPU 0 (where min_vruntime > 0)
  //     and run it alongside `local_peer` on CPU 0. Because `StealTask(0, 1)`
  //     clamps `remote_task`'s vruntime to CPU 0's min_vruntime, `remote_task`
  //     (20) interleaves with `local_peer` (10) instead of starving it.
  Task* const remote_task = core->CreateTaskOnCpu(MigratedFloorWorker,  //
                                                  &ctx,                 //
                                                  kDefaultTaskWeight,   //
                                                  1);
  EXPECT_THAT(core->RunqueueLoad(1), t::Eq(1));
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(0));
  EXPECT_TRUE(core->StealTask(0, 1));
  EXPECT_THAT(core->RunqueueLoad(1), t::Eq(0));
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(1));

  Task* const local_peer = core->CreateTaskOnCpu(LocalPeerFloorWorker,  //
                                                 &ctx,                  //
                                                 kDefaultTaskWeight,    //
                                                 0);
  core->Join(remote_task);
  core->Join(local_peer);

  ASSERT_THAT(static_cast<int>(ctx.migrate_trace.size()), t::Eq(8));
  int max_consecutive_migrated = 0;
  current_run = 0;
  for (const int id : ctx.migrate_trace) {
    if (id == 20) {
      ++current_run;
      if (current_run > max_consecutive_migrated) {
        max_consecutive_migrated = current_run;
      }
    } else {
      current_run = 0;
    }
  }
  EXPECT_THAT(max_consecutive_migrated, t::Le(2));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 5. `StealTask(dst_cpu, src_cpu)` in Isolation
// ============================================================================

struct StealOrderCtx {
  TaskScheduler* sched = nullptr;
  std::vector<int> execution_order;
  bool steal_from_cpu0_while_bootstrap_queued_result = true;
  bool steal_second_task_from_cpu0_result = false;
  int cpu0_load_observed_inside_worker = -1;
  Task* extra_task = nullptr;
};

static void RecordIdOneWorker(void* const raw_arg) {
  StealOrderCtx* const ctx = static_cast<StealOrderCtx*>(raw_arg);
  ctx->execution_order.push_back(1);
}

static void RecordIdTwoWorker(void* const raw_arg) {
  StealOrderCtx* const ctx = static_cast<StealOrderCtx*>(raw_arg);
  ctx->execution_order.push_back(2);
}

static void RecordIdThreeWorker(void* const raw_arg) {
  StealOrderCtx* const ctx = static_cast<StealOrderCtx*>(raw_arg);
  ctx->execution_order.push_back(3);
}

static void StealSkipBootstrapProbeWorker(void* const raw_arg) {
  StealOrderCtx* const ctx = static_cast<StealOrderCtx*>(raw_arg);
  // While this worker is running on CPU 0, the bootstrap task is sitting in
  // CPU 0's runqueue tree (with `is_bootstrap == true`).
  ctx->cpu0_load_observed_inside_worker = ctx->sched->RunqueueLoad(0);
  // Attempting to steal from CPU 0 to empty CPU 2 when ONLY the bootstrap task
  // is queued on CPU 0 must skip the bootstrap task and return false!
  ctx->steal_from_cpu0_while_bootstrap_queued_result =
      ctx->sched->StealTask(2, 0);

  // If a non-bootstrap task is also enqueued on CPU 0 alongside the bootstrap
  // task, StealTask(2, 0) must skip the bootstrap task and steal the
  // non-bootstrap task!
  ctx->extra_task = ctx->sched->CreateTaskOnCpu(RecordIdThreeWorker,  //
                                                ctx,                  //
                                                kDefaultTaskWeight,   //
                                                0);
  ASSERT_THAT(ctx->extra_task, t::NotNull());
  ctx->steal_second_task_from_cpu0_result = ctx->sched->StealTask(2, 0);
}

TEST(CoreRunqueueSchedulerTest,
     StealTaskLockOrderingSkipsBootstrapAndRespectsDestinationAndSourceState) {
  SetupEnv(4);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  core->Init();

  // 1. Edge cases: same CPU (`dst_cpu == src_cpu`) or empty `src_cpu` returns
  //    false.
  EXPECT_FALSE(core->StealTask(0, 0));
  EXPECT_FALSE(core->StealTask(0, 1));
  EXPECT_FALSE(core->StealTask(2, 1));

  StealOrderCtx ctx;
  ctx.sched = core.get();

  // 2. Enqueue two tasks on CPU 2 (`t1` first, `t2` second).
  Task* const t1 = core->CreateTaskOnCpu(RecordIdOneWorker,   //
                                         &ctx,                //
                                         kDefaultTaskWeight,  //
                                         2);
  Task* const t2 = core->CreateTaskOnCpu(RecordIdTwoWorker,   //
                                         &ctx,                //
                                         kDefaultTaskWeight,  //
                                         2);
  ASSERT_THAT(t1, t::NotNull());
  ASSERT_THAT(t2, t::NotNull());
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(2));

  // 3. Test `dst_cpu > src_cpu`: steal from CPU 2 to CPU 3 (`3 > 2`).
  //    Must steal the lowest-key task (`t1`, earlier `enqueue_seq`).
  EXPECT_TRUE(core->StealTask(3, 2));
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(1));
  EXPECT_THAT(core->RunqueueLoad(3), t::Eq(1));

  // 4. Calling `StealTask(3, 2)` again while CPU 3 is already non-empty
  //    (`RunqueueLoad(3) == 1`) must return `true` immediately WITHOUT stealing
  //    `t2` from CPU 2.
  EXPECT_TRUE(core->StealTask(3, 2));
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(1));
  EXPECT_THAT(core->RunqueueLoad(3), t::Eq(1));

  // 5. Offline CPU check: if either `dst_cpu` or `src_cpu` is offline,
  //    `StealTask` returns `false` without stealing.
  g_env.cpu_locals[1].online = false;
  EXPECT_FALSE(core->StealTask(1, 2));
  EXPECT_FALSE(core->StealTask(0, 1));
  g_env.cpu_locals[1].online = true;

  // 6. Test `dst_cpu < src_cpu`: steal `t1` from CPU 3 to CPU 0 (`0 < 3`) and
  //    run it, then steal `t2` from CPU 2 to CPU 0 (`0 < 2`) and run it.
  EXPECT_TRUE(core->StealTask(0, 3));
  EXPECT_THAT(core->RunqueueLoad(3), t::Eq(0));
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(1));
  core->Join(t1);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(1));

  EXPECT_TRUE(core->StealTask(0, 2));
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(0));
  EXPECT_THAT(core->RunqueueLoad(0), t::Eq(1));
  core->Join(t2);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(1, 2));

  // 7. Verify `StealTask` never steals the bootstrap task (`is_bootstrap`):
  //    Run `StealSkipBootstrapProbeWorker` on CPU 0. While it executes,
  //    bootstrap is in CPU 0's runqueue tree.
  Task* const probe = core->CreateTaskOnCpu(StealSkipBootstrapProbeWorker,  //
                                            &ctx,                           //
                                            kDefaultTaskWeight,             //
                                            0);
  core->Join(probe);
  EXPECT_THAT(ctx.cpu0_load_observed_inside_worker, t::Eq(0));
  EXPECT_FALSE(ctx.steal_from_cpu0_while_bootstrap_queued_result);
  EXPECT_TRUE(ctx.steal_second_task_from_cpu0_result);
  ASSERT_THAT(ctx.extra_task, t::NotNull());
  EXPECT_THAT(core->RunqueueLoad(2), t::Eq(1));

  // Steal `extra_task` from CPU 2 onto CPU 0 and join it.
  EXPECT_TRUE(core->StealTask(0, 2));
  core->Join(ctx.extra_task);
  EXPECT_THAT(ctx.execution_order, t::ElementsAre(1, 2, 3));

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 6. `OnTimerInterrupt` and `OnWakeupIpi` Preemption & Weighted Tick Charging
// ============================================================================

struct TimerPreemptCtx {
  TaskScheduler* sched = nullptr;
  int task_b_slices = 0;
  int task_b_slices_after_wakeup_ipi = -1;
  int task_b_slices_after_tick_1 = -1;
  int task_b_slices_after_tick_2 = -1;
};

static void TimerPreemptWorkerB(void* const raw_arg) {
  TimerPreemptCtx* const ctx = static_cast<TimerPreemptCtx*>(raw_arg);
  // Slice 1: advance `task_b->vruntime` from 0 to +512 (weight = 2048) and
  // yield to `task_a` (which is still at `vruntime == 0`).
  ++ctx->task_b_slices;
  ctx->sched->Yield();

  // Slice 2: reached when `task_a` is preempted by `OnTimerInterrupt` once
  // `task_a->vruntime` (1024) exceeds `task_b->vruntime` (512).
  ++ctx->task_b_slices;
}

static void TimerPreemptWorkerA(void* const raw_arg) {
  TimerPreemptCtx* const ctx = static_cast<TimerPreemptCtx*>(raw_arg);
  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  // At entry, `task_a->vruntime == 0` and `task_b` is in CPU 0's runqueue at
  // `vruntime == 512`.
  // 1. `OnWakeupIpi` does not charge vruntime and sees `first->vruntime` (512)
  //    >= `curr->vruntime` (0), so it MUST NOT preempt.
  ctx->sched->OnWakeupIpi(&frame);
  ctx->task_b_slices_after_wakeup_ipi = ctx->task_b_slices;

  // 2. First `OnTimerInterrupt` charges +512 (`weight = 2048`) to `task_a`,
  //    bringing `task_a->vruntime` to 512. Because `first->vruntime <
  //    curr->vruntime` (`512 < 512`) is false, it MUST NOT preempt yet.
  ctx->sched->OnTimerInterrupt(&frame);
  ctx->task_b_slices_after_tick_1 = ctx->task_b_slices;

  // 3. Second `OnTimerInterrupt` charges another +512 to `task_a`, bringing
  //    `task_a->vruntime` to 1024. Now `first->vruntime < curr->vruntime`
  //    (`512 < 1024`) is true, so `task_a` IS preempted and `task_b` runs its
  //    second slice!
  ctx->sched->OnTimerInterrupt(&frame);
  ctx->task_b_slices_after_tick_2 = ctx->task_b_slices;
}

static void ApSimpleRunWorker(void* const raw_arg) {
  int* const counter = static_cast<int*>(raw_arg);
  ++(*counter);
}

TEST(
    CoreRunqueueSchedulerTest,
    OnTimerInterruptChargesWeightedVruntimeAndOnWakeupIpiPreemptsIdleAndLagging) {
  SetupEnv(2);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();

  InterruptFrame frame = {};
  frame.vector = kVectorApicTimer;
  frame.rflags = internal::kRflagsInterruptEnableBit;

  // Calling OnTimerInterrupt / OnWakeupIpi before Init() is a safe no-op.
  core->OnTimerInterrupt(&frame);
  core->OnWakeupIpi(&frame);

  core->Init();

  // Calling OnTimerInterrupt / OnWakeupIpi with unbound CpuLocal or offline CPU
  // is a safe no-op.
  ResetCpuLocalForTest();
  core->OnTimerInterrupt(&frame);
  core->OnWakeupIpi(&frame);
  BindCpuLocal(SmpGetCpuLocal(0));
  g_env.cpu_locals[0].online = false;
  core->OnTimerInterrupt(&frame);
  core->OnWakeupIpi(&frame);
  g_env.cpu_locals[0].online = true;

  // Part A: Verify weighted vruntime charging and strict `<` preemption
  // threshold on CPU 0.
  TimerPreemptCtx ctx;
  ctx.sched = core.get();
  Task* const task_b = core->CreateTaskOnCpu(TimerPreemptWorkerB,     //
                                             &ctx,                    //
                                             kDefaultTaskWeight * 2,  //
                                             0);
  Task* const task_a = core->CreateTaskOnCpu(TimerPreemptWorkerA,     //
                                             &ctx,                    //
                                             kDefaultTaskWeight * 2,  //
                                             0);
  core->Join(task_b);
  core->Join(task_a);

  EXPECT_THAT(ctx.task_b_slices_after_wakeup_ipi, t::Eq(1));
  EXPECT_THAT(ctx.task_b_slices_after_tick_1, t::Eq(1));
  EXPECT_THAT(ctx.task_b_slices_after_tick_2, t::Eq(2));

  // Part B: Verify `OnWakeupIpi` and `OnTimerInterrupt` immediately preempt the
  // idle task on an Application Processor (CPU 1).
  int ap_wakeup_runs = 0;
  Task* const ap_ipi_task = core->CreateTaskOnCpu(ApSimpleRunWorker,   //
                                                  &ap_wakeup_runs,     //
                                                  kDefaultTaskWeight,  //
                                                  1);
  BindCpuLocal(SmpGetCpuLocal(1));
  InterruptFrame ipi_frame = {};
  ipi_frame.vector = kVectorWakeupIpi;
  ipi_frame.rflags = internal::kRflagsInterruptEnableBit;
  core->OnWakeupIpi(&ipi_frame);
  EXPECT_THAT(ap_wakeup_runs, t::Eq(1));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(ap_ipi_task);

  int ap_timer_runs = 0;
  Task* const ap_timer_task = core->CreateTaskOnCpu(ApSimpleRunWorker,   //
                                                    &ap_timer_runs,      //
                                                    kDefaultTaskWeight,  //
                                                    1);
  BindCpuLocal(SmpGetCpuLocal(1));
  core->OnTimerInterrupt(&frame);
  EXPECT_THAT(ap_timer_runs, t::Eq(1));
  BindCpuLocal(SmpGetCpuLocal(0));
  core->Join(ap_timer_task);

  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 7. Two-Stage Zombie Stack/TCB Reclamation (`Join` and `ReapZombies`)
// ============================================================================

struct ZombieStageCtx {
  TaskScheduler* sched = nullptr;
  bool zombie_ran = false;
};

static void ExplicitExitZombieWorker(void* const raw_arg) {
  ZombieStageCtx* const ctx = static_cast<ZombieStageCtx*>(raw_arg);
  ctx->zombie_ran = true;
}

TEST(CoreRunqueueSchedulerTest,
     TwoStageZombieReclamationFreesStackOnContextSwitchAndTcbOnJoin) {
  SetupEnv(1);
  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  core->Init();

  ZombieStageCtx ctx;
  ctx.sched = core.get();
  Task* const zombie =
      core->CreateTask(ExplicitExitZombieWorker, &ctx, kDefaultTaskWeight);
  ASSERT_THAT(zombie, t::NotNull());

  // Immediately after creation, both the 4-frame kernel stack and TCB are
  // allocated.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before - kTaskStackFrames));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  while (!ctx.zombie_ran) {
    core->Yield();
  }

  // Stage 1: As soon as `zombie` exited and switched back to the bootstrap
  // task, `FinishContextSwitch` on the bootstrap stack MUST have freed
  // `zombie`'s 16 KiB stack back to the PMM, while `zombie`'s TCB remains
  // allocated on the heap until `Join(zombie)`.
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  // `CoreRunqueueTaskScheduler::ReapZombies()` is the base no-op (returns 0).
  EXPECT_THAT(core->ReapZombies(), t::Eq(0));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Lt(heap_before));

  // Enqueue a runnable probe task on CPU 0; calling `Join(zombie)` when
  // `zombie` has already exited must reap its TCB immediately WITHOUT yielding
  // to `probe_task`.
  int probe_runs = 0;
  Task* const probe_task =
      core->CreateTask(ApSimpleRunWorker, &probe_runs, kDefaultTaskWeight);
  ASSERT_THAT(probe_task, t::NotNull());

  core->Join(zombie);
  EXPECT_THAT(probe_runs, t::Eq(0));

  core->Join(probe_task);
  EXPECT_THAT(probe_runs, t::Eq(1));
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 8. Multi-Threaded SMP Concurrency Test (`Yield`, `StealTask`,
// `OnTimerInterrupt`)
// ============================================================================

struct MtCoreSharedCtx {
  TaskScheduler* sched = nullptr;
  std::atomic<uint32_t> cpu_mask{0};
  std::atomic<int> completed_tasks{0};
  std::atomic<int64_t> total_iterations{0};
  std::atomic<int64_t> remote_executions{0};
};

struct MtCoreTaskArg {
  MtCoreSharedCtx* shared = nullptr;
  int task_index = 0;
  std::atomic<int> iterations{0};
};

static void MtCoreConcurrencyWorker(void* const raw_arg) {
  MtCoreTaskArg* const arg = static_cast<MtCoreTaskArg*>(raw_arg);
  MtCoreSharedCtx* const shared = arg->shared;
  TaskScheduler* const sched = shared->sched;

  InterruptFrame timer_frame = {};
  timer_frame.vector = kVectorApicTimer;
  timer_frame.rflags = internal::kRflagsInterruptEnableBit;

  for (int round = 0; round < 6; ++round) {
    const int cpu = CurrentCpuId();
    shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                              std::memory_order_acq_rel);
    if (cpu != 0) {
      shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
    }
    arg->iterations.fetch_add(1, std::memory_order_acq_rel);
    shared->total_iterations.fetch_add(1, std::memory_order_acq_rel);

    if ((round & 1) == 0) {
      sched->Yield();
    } else {
      sched->OnTimerInterrupt(&timer_frame);
      sched->Yield();
    }
  }

  const int cpu = CurrentCpuId();
  shared->cpu_mask.fetch_or(1u << static_cast<uint32_t>(cpu),
                            std::memory_order_acq_rel);
  if (cpu != 0) {
    shared->remote_executions.fetch_add(1, std::memory_order_acq_rel);
  }
  shared->completed_tasks.fetch_add(1, std::memory_order_acq_rel);
}

TEST(CoreRunqueueSchedulerTest,
     ConcurrentMultiThreadedYieldStealTaskAndTimerPreemptionUnderTsan) {
  constexpr int kNumCpus = 4;
  constexpr int kNumTasks = 24;
  SetupEnv(kNumCpus);

  const int64_t pmm_before = PmmFreeFrameCount();
  const int64_t heap_before = HeapTotalFreeBytes();

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  core->Init();
  Task* const bootstrap = core->CurrentTask();

  MtCoreSharedCtx shared;
  shared.sched = core.get();
  std::unique_ptr<MtCoreTaskArg[]> args(new MtCoreTaskArg[kNumTasks]);
  std::vector<Task*> tasks;
  tasks.reserve(kNumTasks);

  // Enqueue tasks across CPU 0 (majority) and CPUs 1..3 so StealTask exercises
  // both `dst_cpu < src_cpu` and `dst_cpu > src_cpu` lock acquisition orders.
  for (int i = 0; i < kNumTasks; ++i) {
    args[i].shared = &shared;
    args[i].task_index = i;
    const int initial_cpu = (i < 18) ? 0 : (i % kNumCpus);
    Task* const task = core->CreateTaskOnCpu(MtCoreConcurrencyWorker,     //
                                             &args[i],                    //
                                             kDefaultTaskWeight + i * 8,  //
                                             initial_cpu);
    ASSERT_THAT(task, t::NotNull());
    tasks.push_back(task);
  }

  std::atomic<int> ready_workers{0};
  std::atomic<bool> stop_workers{false};
  std::vector<std::thread> ap_threads;
  ap_threads.reserve(kNumCpus - 1);

  for (int cpu = 1; cpu < kNumCpus; ++cpu) {
    ap_threads.emplace_back([cpu, &core, &ready_workers, &stop_workers]() {
      BindCpuLocal(SmpGetCpuLocal(cpu));
      SetInterruptsEnabledForTest(true);
      ready_workers.fetch_add(1, std::memory_order_acq_rel);

      int donor_cursor = 0;
      while (!stop_workers.load(std::memory_order_acquire)) {
        if (core->RunqueueLoad(cpu) == 0) {
          for (int step = 0; step < kNumCpus; ++step) {
            const int donor = (donor_cursor + step) % kNumCpus;
            if (donor != cpu && core->RunqueueLoad(donor) > 0) {
              if (core->StealTask(cpu, donor)) {
                donor_cursor = (donor + 1) % kNumCpus;
                break;
              }
            }
          }
        }
        if (!core->PollIdleCpu(cpu)) {
          std::this_thread::yield();
        }
      }
      ResetCpuLocalForTest();
    });
  }

  while (ready_workers.load(std::memory_order_acquire) < (kNumCpus - 1)) {
    std::this_thread::yield();
  }

  // While waiting for all tasks to finish on CPU 0, also allow CPU 0 to steal
  // back remaining runnable tasks if CPU 0 drains its local queue first.
  while (shared.completed_tasks.load(std::memory_order_acquire) < kNumTasks) {
    if (core->RunqueueLoad(0) == 0) {
      for (int src = 1; src < kNumCpus; ++src) {
        if (core->RunqueueLoad(src) > 0 && core->StealTask(0, src)) {
          break;
        }
      }
    }
    core->Yield();
    std::this_thread::yield();
  }

  for (Task* const task : tasks) {
    core->Join(task);
  }

  stop_workers.store(true, std::memory_order_release);
  for (std::thread& th : ap_threads) {
    th.join();
  }

  EXPECT_THAT(CurrentCpuId(), t::Eq(0));
  EXPECT_THAT(core->CurrentTask(), t::Eq(bootstrap));
  EXPECT_THAT(shared.completed_tasks.load(), t::Eq(kNumTasks));
  EXPECT_THAT(shared.total_iterations.load(), t::Eq(kNumTasks * 6));
  for (int i = 0; i < kNumTasks; ++i) {
    EXPECT_THAT(args[i].iterations.load(), t::Eq(6));
  }

  EXPECT_THAT(shared.remote_executions.load(), t::Gt(0));
  EXPECT_THAT(shared.cpu_mask.load() & ~1u, t::Ne(0u));

  for (int c = 0; c < kNumCpus; ++c) {
    EXPECT_THAT(core->RunqueueLoad(c), t::Eq(0));
  }
  EXPECT_THAT(PmmFreeFrameCount(), t::Eq(pmm_before));
  EXPECT_THAT(HeapTotalFreeBytes(), t::Eq(heap_before));
}

// ============================================================================
// 9. `DCHECK` Precondition and Invariant Death Tests
// ============================================================================

struct DeathLockWorkerCtx {
  TaskScheduler* sched = nullptr;
  Task* bootstrap = nullptr;
  IrqSpinLock leaked_lock;
};

static void ExitWhileHoldingSpinLockWorker(void* const raw_arg) {
  DeathLockWorkerCtx* const ctx = static_cast<DeathLockWorkerCtx*>(raw_arg);
  ctx->leaked_lock.Lock();
}

static void JoinBootstrapWorker(void* const raw_arg) {
  DeathLockWorkerCtx* const ctx = static_cast<DeathLockWorkerCtx*>(raw_arg);
  ctx->sched->Join(ctx->bootstrap);
}

TEST(CoreRunqueueSchedulerDeathTest,
     PreconditionAndInvariantViolationsTriggerDcheck) {
  SetupEnv(2);

  // 1. Calling methods before `Init()` must trigger `DCHECK`.
  const std::shared_ptr<TaskScheduler> uninit = CreateCoreRunqueueScheduler();
  EXPECT_DEATH(uninit->CreateTask(NoopTask), "Check failed");
  EXPECT_DEATH(
      uninit->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 0),
      "Check failed");
  EXPECT_DEATH(uninit->Yield(), "Check failed");
  EXPECT_DEATH(uninit->Join(nullptr), "Check failed");
  EXPECT_DEATH(uninit->ReapZombies(), "Check failed");
  EXPECT_DEATH(uninit->RunqueueLoad(0), "Check failed");
  EXPECT_DEATH(uninit->StealTask(0, 1), "Check failed");
  EXPECT_DEATH(uninit->CurrentTask(), "Check failed");

  const std::shared_ptr<TaskScheduler> core = CreateCoreRunqueueScheduler();
  g_env.idt_initialized = false;
  EXPECT_DEATH(core->Init(), "Check failed");
  g_env.idt_initialized = true;
  core->Init();
  EXPECT_DEATH(core->Init(), "Check failed");

  // 2. Invalid `CreateTask` / `CreateTaskOnCpu` arguments (`entry == nullptr`,
  //    invalid `weight`, out-of-range or offline `target_cpu`).
  EXPECT_DEATH(core->CreateTask(nullptr), "Check failed");
  EXPECT_DEATH(core->CreateTask(NoopTask, nullptr, 0), "Check failed");
  EXPECT_DEATH(core->CreateTask(NoopTask, nullptr, -5), "Check failed");
  EXPECT_DEATH(core->CreateTask(NoopTask, nullptr, kMaxTaskWeight + 1),
               "Check failed");
  EXPECT_DEATH(core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, -1),
               "Check failed");
  EXPECT_DEATH(core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 2),
               "Check failed");

  g_env.cpu_locals[1].online = false;
  EXPECT_DEATH(core->CreateTaskOnCpu(NoopTask, nullptr, kDefaultTaskWeight, 1),
               "Check failed");
  g_env.cpu_locals[1].online = true;

  // 3. Out-of-range CPU indices on `RunqueueLoad` and `StealTask`.
  EXPECT_DEATH(core->RunqueueLoad(-1), "Check failed");
  EXPECT_DEATH(core->RunqueueLoad(2), "Check failed");
  EXPECT_DEATH(core->StealTask(-1, 0), "Check failed");
  EXPECT_DEATH(core->StealTask(2, 0), "Check failed");
  EXPECT_DEATH(core->StealTask(0, -1), "Check failed");
  EXPECT_DEATH(core->StealTask(0, 2), "Check failed");

  // 4. Invalid `Join` calls (null task, self-join, joining an idle task, or
  //    joining the bootstrap task).
  EXPECT_DEATH(core->Join(nullptr), "Check failed");
  EXPECT_DEATH(core->Join(core->CurrentTask()), "Check failed");

  BindCpuLocal(SmpGetCpuLocal(1));
  Task* const ap1_idle = core->CurrentTask();
  BindCpuLocal(SmpGetCpuLocal(0));
  EXPECT_DEATH(core->Join(ap1_idle), "Check failed");

  DeathLockWorkerCtx worker_ctx;
  worker_ctx.sched = core.get();
  worker_ctx.bootstrap = core->CurrentTask();

  EXPECT_DEATH(
      {
        Task* const t = core->CreateTask(JoinBootstrapWorker,  //
                                         &worker_ctx,          //
                                         kDefaultTaskWeight);
        core->Join(t);
      },
      "Check failed");

  // 5. Yielding, Joining, or Exiting while holding an `IrqSpinLock` must
  //    trigger `DCHECK`.
  EXPECT_DEATH(
      {
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        core->Yield();
      },
      "Check failed");

  EXPECT_DEATH(
      {
        Task* const dummy =
            core->CreateTask(NoopTask, nullptr, kDefaultTaskWeight);
        IrqSpinLock lock;
        const IrqSpinLockGuard guard(lock);
        core->Join(dummy);
      },
      "Check failed");

  EXPECT_DEATH(
      {
        Task* const t = core->CreateTask(ExitWhileHoldingSpinLockWorker,  //
                                         &worker_ctx,                     //
                                         kDefaultTaskWeight);
        core->Join(t);
      },
      "Check failed");
}

}  // namespace
}  // namespace protos
