#include "task.h"

#include <memory>
#include <utility>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif

#include "core_runqueue_scheduler.h"
#include "idt.h"
#include "join_lifecycle_scheduler.h"
#include "preemptive_scheduler.h"
#include "work_stealing_scheduler.h"

namespace protos {
namespace {

// Single global variable across all task scheduler implementation files.
std::shared_ptr<TaskScheduler> g_task_scheduler;

static void ApicTimerInterruptHandler(InterruptFrame* const frame) {
  if (g_task_scheduler == nullptr) {
    return;
  }
  g_task_scheduler->OnTimerInterrupt(frame);
}

static void WakeupIpiInterruptHandler(InterruptFrame* const frame) {
  if (g_task_scheduler == nullptr) {
    return;
  }
  g_task_scheduler->OnWakeupIpi(frame);
}

static void ResetSchedulerStateInternal() {
  if (IdtIsInitialized()) {
    IdtUnregisterHandler(kVectorApicTimer);
    IdtUnregisterHandler(kVectorWakeupIpi);
  }
  g_task_scheduler.reset();
}

}  // namespace

void TaskInit() {
  ResetSchedulerStateInternal();
  if (!IdtIsInitialized()) {
    IdtInit();
  }

  std::shared_ptr<TaskScheduler> scheduler =
      CreatePreemptiveScheduler(CreateJoinLifecycleScheduler(
          CreateWorkStealingScheduler(CreateCoreRunqueueScheduler())));
  scheduler->Init();
  g_task_scheduler = std::move(scheduler);

  IdtRegisterHandler(kVectorApicTimer, ApicTimerInterruptHandler);
  IdtRegisterHandler(kVectorWakeupIpi, WakeupIpiInterruptHandler);
}

const std::shared_ptr<TaskScheduler>& GetTaskScheduler() {
  return g_task_scheduler;
}

#if __STDC_HOSTED__
void TaskResetForTest() { ResetSchedulerStateInternal(); }
#endif

}  // namespace protos
