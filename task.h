#ifndef PROTOS_TASK_H_
#define PROTOS_TASK_H_

#include <cstdint>
#include <memory>

namespace protos {

struct InterruptFrame;
struct Task;

// Default, minimum, and maximum task weights for proportional-share vruntime
// scheduling (matching Linux CFS NICE_0_LOAD = 1024).
constexpr int64_t kDefaultTaskWeight = 1024;
constexpr int64_t kMinTaskWeight = 1;
constexpr int64_t kMaxTaskWeight = 1024 * 1024;

// Base virtual-runtime quantum charged per timer tick or cooperative yield to
// a task with weight == kDefaultTaskWeight.
constexpr int64_t kBaseVruntimeDelta = 1024;

// Number of 4 KiB physical frames allocated per task kernel stack (16 KiB).
constexpr int64_t kTaskStackFrames = 4;
constexpr int64_t kTaskStackSize = kTaskStackFrames * 4096;

// Entry function signature for kernel tasks.
using TaskFn = void (*)(void* arg);
using TaskEntry = TaskFn;

// Abstract composable task scheduler interface.
//
// Implementations follow the decorator/layered composition pattern: each class
// implements `TaskScheduler` with a single scheduling responsibility and wraps
// an inner `std::shared_ptr<TaskScheduler>` (with the innermost base layer
// managing per-CPU runqueues and low-level context switching).
class TaskScheduler {
 public:
  virtual ~TaskScheduler() = default;

  // Initializes the scheduler layer and any wrapped inner layers. Must be
  // called on the BSP (CPU 0) after SMP bring-up before creating or scheduling
  // tasks.
  virtual void Init() = 0;

  // Allocates a new task control block and 16-byte-aligned 16 KiB kernel stack
  // configured to execute `entry(arg)` with proportional-share `weight`
  // (`kMinTaskWeight <= weight <= kMaxTaskWeight`), places it on a CPU selected
  // by the scheduler (e.g. the least-loaded online CPU), and returns an opaque
  // handle to the scheduled task.
  virtual Task* CreateTask(TaskFn entry,         //
                           void* arg = nullptr,  //
                           int64_t weight = kDefaultTaskWeight) = 0;

  // Creates a new task configured to execute `entry(arg)` with `weight` and
  // explicitly enqueues it onto online CPU `target_cpu`
  // (`0 <= target_cpu < SmpCpuCount()`), waking `target_cpu` via IPI if remote.
  virtual Task* CreateTaskOnCpu(TaskFn entry,    //
                                void* arg,       //
                                int64_t weight,  //
                                int target_cpu) = 0;

  // Voluntarily yields the calling CPU to the next eligible runnable task,
  // charging virtual runtime to the calling task inversely proportional to its
  // weight. Must not be called while holding an `IrqSpinLock`.
  virtual void Yield() = 0;

  // Waits for `task` to finish executing and reclaims its TCB and stack
  // resources. If `task` has already exited, reclaims its resources immediately
  // without yielding. Must not be called on `CurrentTask()`, an idle/bootstrap
  // task, or while holding an `IrqSpinLock`.
  virtual void Join(Task* task) = 0;

  // Reclaims TCB and stack resources of all unjoined tasks that have already
  // exited and have no active `Join()` waiter. Returns the number of reaped
  // tasks.
  virtual int ReapZombies() = 0;

  // Polls and dispatches runnable (or stolen) work on idle CPU `cpu_index`.
  // Returns `true` if a runnable task was switched in, or `false` if no work
  // was available.
  virtual bool PollIdleCpu(int cpu_index) = 0;

  // Returns the number of runnable (non-running, non-idle) tasks currently
  // queued on CPU `cpu_index` (`0 <= cpu_index < SmpCpuCount()`).
  virtual int RunqueueLoad(int cpu_index) const = 0;

  // Attempts to migrate one stealable runnable task from `src_cpu` to `dst_cpu`
  // using deadlock-free ascending `cpu_id` runqueue lock ordering and clamping
  // the stolen task's `vruntime` to `dst_cpu`'s `min_vruntime` floor. Returns
  // `true` if `dst_cpu` already had runnable work or a task was stolen.
  virtual bool StealTask(int dst_cpu, int src_cpu) = 0;

  // Enables or disables interrupt-driven preemption on timer and wakeup IPIs.
  virtual void SetPreemptEnabled(bool enabled) = 0;

  // Returns `true` if interrupt-driven preemption is currently enabled.
  virtual bool IsPreemptEnabled() const = 0;

  // Handles a per-CPU Local APIC timer interrupt (`kVectorApicTimer = 0x20`),
  // charging virtual runtime to the interrupted task and preempting it if
  // preemption is safe and a runnable task has lower `vruntime`.
  virtual void OnTimerInterrupt(InterruptFrame* frame) = 0;

  // Handles a cross-CPU wakeup/reschedule IPI (`kVectorWakeupIpi = 0x21`),
  // preempting the current task (or waking an idle CPU) if preemption is safe
  // and a runnable task is eligible to run.
  virtual void OnWakeupIpi(InterruptFrame* frame) = 0;

  // Returns the opaque `Task*` currently executing on the calling CPU in O(1).
  virtual Task* CurrentTask() const = 0;
};

// Initializes the composed 4-layer task scheduler and registers IDT handlers.
void TaskInit();

// Returns the active composed `TaskScheduler` instance (`nullptr` before
// `TaskInit()`).
const std::shared_ptr<TaskScheduler>& GetTaskScheduler();

#if __STDC_HOSTED__
void TaskResetForTest();
#endif

}  // namespace protos

#endif  // PROTOS_TASK_H_
