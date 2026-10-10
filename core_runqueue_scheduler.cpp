#include "core_runqueue_scheduler.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif
#if __STDC_HOSTED__
#include <pthread.h>

#include <thread>
#if defined(__SANITIZE_ADDRESS__)
#include <sanitizer/asan_interface.h>
#include <sanitizer/common_interface_defs.h>
#endif
#if defined(__SANITIZE_THREAD__)
#include <sanitizer/tsan_interface.h>
#endif
#endif

#include "check.h"
#include "heap.h"
#include "idt.h"
#include "pmm.h"
#include "rbtree.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {
struct Task;
extern "C" void TaskStartupEntry(Task* curr);
}  // namespace protos

extern "C" {
void TaskContextSwitch(uintptr_t* prev_rsp, uintptr_t next_rsp);
void TaskEntryTrampoline();
}

#if __STDC_HOSTED__
asm(".text\n"
    ".intel_syntax noprefix\n"
    ".align 16\n"
    ".global TaskContextSwitch\n"
    ".type TaskContextSwitch, @function\n"
    "TaskContextSwitch:\n"
    "  push rbp\n"
    "  push rbx\n"
    "  push r12\n"
    "  push r13\n"
    "  push r14\n"
    "  push r15\n"
    "  mov qword ptr [rdi], rsp\n"
    "  mov rsp, rsi\n"
    "  pop r15\n"
    "  pop r14\n"
    "  pop r13\n"
    "  pop r12\n"
    "  pop rbx\n"
    "  pop rbp\n"
    "  ret\n"
    "\n"
    ".align 16\n"
    ".global TaskEntryTrampoline\n"
    ".type TaskEntryTrampoline, @function\n"
    "TaskEntryTrampoline:\n"
    "  xor rbp, rbp\n"
    "  and rsp, -16\n"
    "  mov rdi, r12\n"
    "  call TaskStartupEntry\n"
    ".Lhost_task_exit_spin:\n"
    "  pause\n"
    "  jmp .Lhost_task_exit_spin\n"
    ".att_syntax prefix\n");
#endif

namespace protos {

namespace {

enum class TaskState : uint8_t {
  kReady = 0,
  kRunning = 1,
  kExited = 2,
};

struct TaskRunqueueKey {
  int64_t vruntime = 0;
  int64_t enqueue_seq = 0;
  int64_t task_id = 0;

  constexpr bool operator<(const TaskRunqueueKey& other) const {
    if (vruntime != other.vruntime) {
      return vruntime < other.vruntime;
    }
    if (enqueue_seq != other.enqueue_seq) {
      return enqueue_seq < other.enqueue_seq;
    }
    return task_id < other.task_id;
  }

  constexpr bool operator==(const TaskRunqueueKey& other) const {
    return vruntime == other.vruntime && enqueue_seq == other.enqueue_seq &&
           task_id == other.task_id;
  }
};

class CoreRunqueueTaskScheduler;

}  // namespace

// Task Control Block (TCB), hidden from `task.h` as an opaque type and defined
// privately inside `core_runqueue_scheduler.cpp`.
struct Task {
  CoreRunqueueTaskScheduler* owner = nullptr;
  int64_t id = 0;
  uintptr_t rsp = 0;
  uintptr_t stack_base = 0;
  uintptr_t stack_top = 0;
  int64_t stack_frames = 0;

  TaskFn entry = nullptr;
  void* arg = nullptr;

  int64_t weight = kDefaultTaskWeight;
  int64_t vruntime = 0;
  int64_t enqueue_seq = 0;
  RbNode runqueue_node;

  std::atomic<TaskState> state{TaskState::kReady};
  std::atomic<int> running_cpu{-1};
  std::atomic<int> last_cpu{0};
  bool is_idle = false;
  bool is_bootstrap = false;
  bool owns_stack = false;
  bool has_custom_stack = false;
  std::atomic<bool> stack_reaped{false};

  std::atomic<int64_t> runtime_ticks{0};
  std::atomic<int64_t> switches{0};
  std::atomic<int> migrations{0};

#if __STDC_HOSTED__
#if defined(__SANITIZE_ADDRESS__)
  void* asan_fake_stack = nullptr;
  const void* asan_stack_bottom = nullptr;
  size_t asan_stack_size = 0;
#endif
#if defined(__SANITIZE_THREAD__)
  void* tsan_fiber = nullptr;
  bool owns_tsan_fiber = false;
#endif
#endif
};

namespace {

struct TaskRunqueueRbTraits {
  static TaskRunqueueKey GetKey(const Task& task) {
    return TaskRunqueueKey{task.vruntime, task.enqueue_seq, task.id};
  }

  static bool Less(const TaskRunqueueKey& a, const TaskRunqueueKey& b) {
    return a < b;
  }
};

using TaskRunqueueTree =
    RbTree<Task, &Task::runqueue_node, TaskRunqueueRbTraits>;

// Initial stack frame layout popped by `TaskContextSwitch` when dispatching a
// newly created task into `TaskEntryTrampoline`. `r12` holds `Task*` so
// `TaskEntryTrampoline` passes it to `TaskStartupEntry(Task* curr)` in `rdi`.
struct InitialSwitchFrame {
  uint64_t r15;
  uint64_t r14;
  uint64_t r13;
  uint64_t r12;
  uint64_t rbx;
  uint64_t rbp;
  uint64_t rip;
  uint64_t pad_zero;
};
static_assert(sizeof(InitialSwitchFrame) == 64);

struct alignas(64) CpuRunqueue {
  IrqSpinLock lock;
  TaskRunqueueTree tree;
  int cpu_id = 0;
  int64_t min_vruntime = 0;
  int64_t next_enqueue_seq = 0;
  std::atomic<int> runnable_count{0};

  Task* current_task = nullptr;
  Task* idle_task = nullptr;

  Task* pending_handoff_task = nullptr;
  TaskState pending_handoff_state = TaskState::kReady;
  bool in_schedule = false;
};

[[gnu::noinline]] static int GetCurrentCpuId() {
  DCHECK(!AreInterruptsEnabled());
  return CurrentCpuId();
}

[[gnu::noinline]] static CpuLocal* GetCurrentCpuOrNull() {
  DCHECK(!AreInterruptsEnabled());
  return CurrentCpuOrNull();
}

class ScopedIrqDisable {
 public:
  ScopedIrqDisable() : saved_irq_(internal::LocalSaveAndDisableInterrupts()) {}
  ~ScopedIrqDisable() { internal::LocalRestoreInterrupts(saved_irq_); }

  ScopedIrqDisable(const ScopedIrqDisable&) = delete;
  ScopedIrqDisable& operator=(const ScopedIrqDisable&) = delete;

 private:
  const bool saved_irq_;
};

static bool IsCpuOnline(const int cpu_index, const int cpu_count) {
  if (cpu_index < 0 || cpu_index >= cpu_count) {
    return false;
  }
  const CpuLocal* const local = SmpGetCpuLocal(cpu_index);
  return local != nullptr && local->online;
}

static int64_t ComputeWeightedVruntimeDelta(const int64_t delta_exec,
                                            const int64_t weight) {
  DCHECK(delta_exec > 0);
  DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
  const int64_t scaled = delta_exec * kBaseVruntimeDelta * kDefaultTaskWeight;
  const int64_t delta = scaled / weight;
  return (delta > 0) ? delta : 1;
}

static void EnsureNativeThreadSanitizerState(Task* const task) {
  DCHECK(task != nullptr);
#if __STDC_HOSTED__
  if (task->has_custom_stack) {
    return;
  }
#if defined(__SANITIZE_ADDRESS__)
  if (task->asan_stack_bottom == nullptr) {
    pthread_attr_t attr;
    if (pthread_getattr_np(pthread_self(), &attr) == 0) {
      void* saddr = nullptr;
      size_t ssize = 0;
      if (pthread_attr_getstack(&attr, &saddr, &ssize) == 0) {
        task->asan_stack_bottom = saddr;
        task->asan_stack_size = ssize;
      }
      pthread_attr_destroy(&attr);
    }
  }
#endif
#if defined(__SANITIZE_THREAD__)
  if (task->tsan_fiber == nullptr) {
    task->tsan_fiber = __tsan_get_current_fiber();
    task->owns_tsan_fiber = false;
  }
#endif
#else
  (void)task;
#endif
}

static void DestroyTaskSanitizerState(Task* const task) {
  DCHECK(task != nullptr);
#if __STDC_HOSTED__ && defined(__SANITIZE_THREAD__)
  if (task->owns_tsan_fiber && task->tsan_fiber != nullptr) {
    __tsan_destroy_fiber(task->tsan_fiber);
    task->tsan_fiber = nullptr;
    task->owns_tsan_fiber = false;
  }
#else
  (void)task;
#endif
}

static void InitializeTaskStackFrame(Task* const task) {
  DCHECK(task != nullptr);
  DCHECK(task->stack_base != 0);
  DCHECK(task->stack_top > task->stack_base);
  DCHECK((task->stack_top & 0xFu) == 0);

  const uintptr_t frame_addr = task->stack_top - sizeof(InitialSwitchFrame);
  DCHECK((frame_addr & 0xFu) == 0);
  InitialSwitchFrame* const frame =
      reinterpret_cast<InitialSwitchFrame*>(frame_addr);
  frame->r15 = 0;
  frame->r14 = 0;
  frame->r13 = 0;
  frame->r12 = reinterpret_cast<uint64_t>(task);
  frame->rbx = 0;
  frame->rbp = 0;
  frame->rip = reinterpret_cast<uintptr_t>(&TaskEntryTrampoline);
  frame->pad_zero = 0;
  task->rsp = frame_addr;

#if __STDC_HOSTED__
#if defined(__SANITIZE_ADDRESS__)
  task->asan_fake_stack = nullptr;
  task->asan_stack_bottom = reinterpret_cast<const void*>(task->stack_base);
  task->asan_stack_size =
      static_cast<size_t>(task->stack_top - task->stack_base);
  __asan_unpoison_memory_region(task->asan_stack_bottom, task->asan_stack_size);
#endif
#if defined(__SANITIZE_THREAD__)
  task->tsan_fiber = __tsan_create_fiber(0);
  task->owns_tsan_fiber = true;
#endif
#endif
}

static void UpdateRunqueueMinVruntime(CpuRunqueue* const rq) {
  DCHECK(rq != nullptr);
  int64_t min_candidate = INT64_MAX;
  const Task* const curr = rq->current_task;
  if (curr != nullptr && !curr->is_idle &&
      curr->state.load(std::memory_order_relaxed) == TaskState::kRunning) {
    min_candidate = curr->vruntime;
  }
  const Task* const first_ready = rq->tree.First();
  if (first_ready != nullptr && first_ready->vruntime < min_candidate) {
    min_candidate = first_ready->vruntime;
  }
  if (min_candidate != INT64_MAX && min_candidate > rq->min_vruntime) {
    rq->min_vruntime = min_candidate;
  }
}

static void EnqueueTaskLocked(CpuRunqueue* const rq,  //
                              Task* const task,       //
                              const bool clamp_to_min_vruntime) {
  DCHECK(rq != nullptr);
  DCHECK(task != nullptr);
  DCHECK(!task->is_idle);

  if (clamp_to_min_vruntime && task->vruntime < rq->min_vruntime) {
    task->vruntime = rq->min_vruntime;
  }
  task->enqueue_seq = ++rq->next_enqueue_seq;
  task->running_cpu.store(-1, std::memory_order_release);
  task->last_cpu.store(rq->cpu_id, std::memory_order_relaxed);
  task->state.store(TaskState::kReady, std::memory_order_release);

  const bool inserted = rq->tree.Insert(task);
  DCHECK(inserted);
  if (!task->is_bootstrap) {
    rq->runnable_count.fetch_add(1, std::memory_order_release);
  }
  UpdateRunqueueMinVruntime(rq);
}

static void IdleTaskLoop(void* const arg) {
  TaskScheduler* const fallback_sched = static_cast<TaskScheduler*>(arg);
  for (;;) {
    int cpu_id = 0;
    {
      const ScopedIrqDisable irq_guard;
      cpu_id = GetCurrentCpuId();
    }
    const std::shared_ptr<TaskScheduler>& active_sched = GetTaskScheduler();
    TaskScheduler* const sched =
        (active_sched != nullptr) ? active_sched.get() : fallback_sched;
    if (sched != nullptr && sched->PollIdleCpu(cpu_id)) {
      continue;
    }
#if !__STDC_HOSTED__
    asm volatile("sti; hlt" : : : "memory", "cc");
#else
    SetInterruptsEnabledForTest(true);
    std::this_thread::yield();
#endif
  }
}

class CoreRunqueueTaskScheduler final : public TaskScheduler {
 public:
  CoreRunqueueTaskScheduler() = default;

  ~CoreRunqueueTaskScheduler() override {
    if (idle_tasks_ != nullptr) {
      for (int i = 0; i < cpu_count_; ++i) {
        DestroyTaskSanitizerState(&idle_tasks_[i]);
      }
    }
    DestroyTaskSanitizerState(&bootstrap_task_);
  }

  void Init() override {
    DCHECK(!initialized_.load(std::memory_order_acquire));
    DCHECK(IdtIsInitialized());
    const ScopedIrqDisable irq_guard;

    const int cpu_count = SmpCpuCount();
    CHECK(cpu_count >= 1);
    CHECK(SmpOnlineCpuCount() == cpu_count);
    cpu_count_ = cpu_count;

    runqueues_.reset(new CpuRunqueue[cpu_count]);
    CHECK(runqueues_ != nullptr);
    idle_tasks_.reset(new Task[cpu_count]);
    CHECK(idle_tasks_ != nullptr);

    if (GetCurrentCpuOrNull() == nullptr) {
      CpuLocal* const bsp_local = SmpGetCpuLocal(0);
      CHECK(bsp_local != nullptr);
      BindCpuLocal(bsp_local);
    }
    CHECK(GetCurrentCpuId() == 0);

    for (int i = 0; i < cpu_count; ++i) {
      runqueues_[i].cpu_id = i;
    }

    CpuLocal* const bsp_local = SmpGetCpuLocal(0);
    CHECK(bsp_local != nullptr);
    bootstrap_task_.owner = this;
    bootstrap_task_.id = 0;
    bootstrap_task_.stack_base = bsp_local->stack_base;
    bootstrap_task_.stack_top = bsp_local->stack_top;
    bootstrap_task_.stack_frames = kTaskStackFrames;
    bootstrap_task_.weight = kDefaultTaskWeight;
    bootstrap_task_.vruntime = 0;
    bootstrap_task_.enqueue_seq = 0;
    bootstrap_task_.state.store(TaskState::kRunning, std::memory_order_relaxed);
    bootstrap_task_.running_cpu.store(0, std::memory_order_relaxed);
    bootstrap_task_.last_cpu.store(0, std::memory_order_relaxed);
    bootstrap_task_.is_idle = false;
    bootstrap_task_.is_bootstrap = true;
    bootstrap_task_.owns_stack = false;
    bootstrap_task_.has_custom_stack = false;
    EnsureNativeThreadSanitizerState(&bootstrap_task_);

    for (int i = 0; i < cpu_count; ++i) {
      CpuLocal* const cpu_local = SmpGetCpuLocal(i);
      CHECK(cpu_local != nullptr);
      CHECK(cpu_local->online);

      Task& idle = idle_tasks_[i];
      idle.owner = this;
      idle.id = i + 1;
      idle.weight = kDefaultTaskWeight;
      idle.vruntime = 0;
      idle.enqueue_seq = 0;
      idle.last_cpu.store(i, std::memory_order_relaxed);
      idle.is_idle = true;
      idle.is_bootstrap = false;
      idle.owns_stack = false;

      if (i == 0) {
        idle.stack_base = reinterpret_cast<uintptr_t>(&bsp_idle_stack_[0]);
        idle.stack_top = idle.stack_base + sizeof(bsp_idle_stack_);
        idle.stack_frames = kTaskStackFrames;
        idle.entry = IdleTaskLoop;
        idle.arg = this;
        idle.has_custom_stack = true;
        idle.state.store(TaskState::kReady, std::memory_order_relaxed);
        idle.running_cpu.store(-1, std::memory_order_relaxed);
        InitializeTaskStackFrame(&idle);

        runqueues_[0].idle_task = &idle;
        runqueues_[0].current_task = &bootstrap_task_;
      } else {
        idle.stack_base = cpu_local->stack_base;
        idle.stack_top = cpu_local->stack_top;
        idle.stack_frames = kApStackFrames;
        idle.entry = IdleTaskLoop;
        idle.arg = this;
        idle.has_custom_stack = false;
        idle.state.store(TaskState::kRunning, std::memory_order_relaxed);
        idle.running_cpu.store(i, std::memory_order_relaxed);

        runqueues_[i].idle_task = &idle;
        runqueues_[i].current_task = &idle;
      }
    }

    next_task_id_.store(cpu_count + 1, std::memory_order_relaxed);
    initialized_.store(true, std::memory_order_release);
  }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    return CreateTaskOnCpu(entry, arg, weight, 0);
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(entry != nullptr);
    DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
    DCHECK(target_cpu >= 0 && target_cpu < cpu_count_);
    DCHECK(IsCpuOnline(target_cpu, cpu_count_));

    void* const raw_tcb = Kmalloc(sizeof(Task));
    CHECK(raw_tcb != nullptr);
    const uintptr_t stack_phys = PmmAllocFrames(kTaskStackFrames);
    if (stack_phys == 0) {
      Kfree(raw_tcb);
      CHECK(false);
    }
    DCHECK((stack_phys & 0xFu) == 0);

    Task* const task = new (raw_tcb) Task();
    task->owner = this;
    task->id = next_task_id_.fetch_add(1, std::memory_order_relaxed);
    task->stack_base = stack_phys;
    task->stack_top = stack_phys + kTaskStackSize;
    task->stack_frames = kTaskStackFrames;
    task->entry = entry;
    task->arg = arg;
    task->weight = weight;
    task->vruntime = 0;
    task->enqueue_seq = 0;
    task->state.store(TaskState::kReady, std::memory_order_relaxed);
    task->running_cpu.store(-1, std::memory_order_relaxed);
    task->last_cpu.store(target_cpu, std::memory_order_relaxed);
    task->is_idle = false;
    task->is_bootstrap = false;
    task->owns_stack = true;
    task->has_custom_stack = true;
    task->stack_reaped.store(false, std::memory_order_relaxed);

    InitializeTaskStackFrame(task);

    CpuRunqueue& rq = runqueues_[target_cpu];
    {
      const IrqSpinLockGuard guard(rq.lock);
      task->vruntime = rq.min_vruntime;
      EnqueueTaskLocked(&rq, task, true);
    }
    return task;
  }

  [[gnu::noinline]] void Yield() override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    {
      const ScopedIrqDisable irq_guard;
      const int cpu_id = GetCurrentCpuId();
      DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
      CpuRunqueue& rq = runqueues_[cpu_id];
      if (rq.in_schedule) {
        return;
      }
      const IrqSpinLockGuard guard(rq.lock);
      Task* const curr = rq.current_task;
      DCHECK(curr != nullptr);
      if (!curr->is_idle) {
        curr->runtime_ticks.fetch_add(1, std::memory_order_relaxed);
        curr->vruntime += ComputeWeightedVruntimeDelta(1, curr->weight);
        UpdateRunqueueMinVruntime(&rq);
      }
    }
    (void)Schedule(TaskState::kReady);
  }

  void Join(Task* const task) override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(task != nullptr);
    Task* const self = CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(task != self);
    DCHECK(!task->is_idle);
    DCHECK(!task->is_bootstrap);
    DCHECK(!self->is_idle);
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    while (task->state.load(std::memory_order_acquire) != TaskState::kExited) {
      const std::shared_ptr<TaskScheduler>& sched = GetTaskScheduler();
      if (sched != nullptr) {
        sched->Yield();
      } else {
        Yield();
      }
#if __STDC_HOSTED__
      std::this_thread::yield();
#else
      asm volatile("pause" : : : "memory");
#endif
    }

    DCHECK(task->stack_reaped.load(std::memory_order_acquire));
    DestroyTaskSanitizerState(task);
    task->~Task();
    Kfree(task);
  }

  int ReapZombies() override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    return 0;
  }

  bool PollIdleCpu(const int cpu_index) override {
    if (!initialized_.load(std::memory_order_acquire) || cpu_index < 0 ||
        cpu_index >= cpu_count_) {
      return false;
    }
    if (runqueues_[cpu_index].runnable_count.load(std::memory_order_acquire) ==
            0 &&
        (cpu_index != 0 ||
         bootstrap_task_.state.load(std::memory_order_acquire) !=
             TaskState::kReady)) {
      return false;
    }
    return Schedule(TaskState::kReady);
  }

  int RunqueueLoad(const int cpu_index) const override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(cpu_index >= 0 && cpu_index < cpu_count_);
    return runqueues_[cpu_index].runnable_count.load(std::memory_order_acquire);
  }

  bool StealTask(const int dst_cpu, const int src_cpu) override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    DCHECK(dst_cpu >= 0 && dst_cpu < cpu_count_);
    DCHECK(src_cpu >= 0 && src_cpu < cpu_count_);
    if (dst_cpu == src_cpu || !IsCpuOnline(dst_cpu, cpu_count_) ||
        !IsCpuOnline(src_cpu, cpu_count_)) {
      return false;
    }

    const int first_cpu = (dst_cpu < src_cpu) ? dst_cpu : src_cpu;
    const int second_cpu = (dst_cpu < src_cpu) ? src_cpu : dst_cpu;
    const IrqSpinLockGuard first_guard(runqueues_[first_cpu].lock);
    const IrqSpinLockGuard second_guard(runqueues_[second_cpu].lock);

    CpuRunqueue& dst_rq = runqueues_[dst_cpu];
    CpuRunqueue& src_rq = runqueues_[src_cpu];

    if (dst_rq.runnable_count.load(std::memory_order_relaxed) > 0) {
      return true;
    }
    if (src_rq.runnable_count.load(std::memory_order_relaxed) == 0) {
      return false;
    }

    Task* stolen = src_rq.tree.First();
    while (stolen != nullptr && (stolen->is_bootstrap || stolen->is_idle)) {
      stolen = TaskRunqueueTree::Next(stolen);
    }
    if (stolen == nullptr) {
      return false;
    }

    src_rq.tree.Erase(stolen);
    src_rq.runnable_count.fetch_sub(1, std::memory_order_release);
    UpdateRunqueueMinVruntime(&src_rq);

    if (stolen->vruntime < dst_rq.min_vruntime) {
      stolen->vruntime = dst_rq.min_vruntime;
    }
    stolen->migrations.fetch_add(1, std::memory_order_relaxed);
    EnqueueTaskLocked(&dst_rq, stolen, false);
    return true;
  }

  void SetPreemptEnabled(const bool /*enabled*/) override {}
  bool IsPreemptEnabled() const override { return true; }

  void OnTimerInterrupt(InterruptFrame* const /*frame*/) override {
    if (!initialized_.load(std::memory_order_acquire)) {
      return;
    }
    const ScopedIrqDisable irq_guard;
    const CpuLocal* const cpu = GetCurrentCpuOrNull();
    if (cpu == nullptr || !cpu->online) {
      return;
    }
    const int cpu_id = cpu->cpu_id;
    if (cpu_id < 0 || cpu_id >= cpu_count_) {
      return;
    }
    CpuRunqueue& rq = runqueues_[cpu_id];
    if (rq.in_schedule || rq.lock.IsLockedByCurrentCpu()) {
      return;
    }

    bool should_preempt = false;
    {
      const IrqSpinLockGuard guard(rq.lock);
      Task* const curr = rq.current_task;
      if (curr != nullptr && !curr->is_idle &&
          curr->state.load(std::memory_order_relaxed) == TaskState::kRunning) {
        curr->runtime_ticks.fetch_add(1, std::memory_order_relaxed);
        curr->vruntime += ComputeWeightedVruntimeDelta(1, curr->weight);
        UpdateRunqueueMinVruntime(&rq);
      }
      const Task* const first = rq.tree.First();
      if (!rq.in_schedule && curr != nullptr && first != nullptr) {
        should_preempt = curr->is_idle || (first->vruntime < curr->vruntime);
      }
    }
    if (should_preempt) {
      (void)Schedule(TaskState::kReady);
    }
  }

  void OnWakeupIpi(InterruptFrame* const /*frame*/) override {
    if (!initialized_.load(std::memory_order_acquire)) {
      return;
    }
    const ScopedIrqDisable irq_guard;
    const CpuLocal* const cpu = GetCurrentCpuOrNull();
    if (cpu == nullptr || !cpu->online) {
      return;
    }
    const int cpu_id = cpu->cpu_id;
    if (cpu_id < 0 || cpu_id >= cpu_count_) {
      return;
    }
    CpuRunqueue& rq = runqueues_[cpu_id];
    if (rq.in_schedule || rq.lock.IsLockedByCurrentCpu()) {
      return;
    }

    bool should_preempt = false;
    {
      const IrqSpinLockGuard guard(rq.lock);
      const Task* const curr = rq.current_task;
      const Task* const first = rq.tree.First();
      if (!rq.in_schedule && curr != nullptr && first != nullptr) {
        should_preempt = curr->is_idle || (first->vruntime < curr->vruntime);
      }
    }
    if (should_preempt) {
      (void)Schedule(TaskState::kReady);
    }
  }

  [[gnu::noinline]] Task* CurrentTask() const override {
    DCHECK(initialized_.load(std::memory_order_acquire));
    const ScopedIrqDisable irq_guard;
    const int cpu_id = GetCurrentCpuId();
    DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
    CpuRunqueue& rq = runqueues_[cpu_id];
    const IrqSpinLockGuard lock_guard(rq.lock);
    Task* const curr = rq.current_task;
    DCHECK(curr != nullptr);
    return curr;
  }

  static void FinishStartupSwitch(CoreRunqueueTaskScheduler* const owner) {
    owner->FinishContextSwitch();
  }

 private:
  friend void protos::TaskStartupEntry(Task* curr);

  [[gnu::noinline]] void ExitCurrentTask() {
    DCHECK(initialized_.load(std::memory_order_acquire));
    Task* const self = CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(!self->is_idle);
    DCHECK(!self->is_bootstrap);
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    (void)Schedule(TaskState::kExited);
    CHECK(false);
  }

  // Completes the second half of a context switch after `TaskContextSwitch`
  // has switched `%rsp` onto the incoming task's stack (`next`). Only once the
  // CPU is no longer executing on the outgoing task's stack (`prev`) is it
  // safe on SMP to either re-enqueue `prev` into `rq.tree` (where a remote CPU
  // could immediately steal and resume it) or free `prev`'s stack frames back
  // to the PMM on task exit.
  [[gnu::noinline]] void FinishContextSwitch() {
    DCHECK(!AreInterruptsEnabled());
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    const int cpu_id = GetCurrentCpuId();
    DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
    CpuRunqueue& rq = runqueues_[cpu_id];
    DCHECK(rq.in_schedule);

#if __STDC_HOSTED__ && defined(__SANITIZE_ADDRESS__)
    Task* const curr = rq.current_task;
    const void* old_bottom = nullptr;
    size_t old_size = 0;
    __sanitizer_finish_switch_fiber(
        (curr != nullptr) ? curr->asan_fake_stack : nullptr,  //
        &old_bottom,                                          //
        &old_size);
    if (rq.pending_handoff_task != nullptr &&
        !rq.pending_handoff_task->has_custom_stack && old_bottom != nullptr &&
        old_size > 0) {
      rq.pending_handoff_task->asan_stack_bottom = old_bottom;
      rq.pending_handoff_task->asan_stack_size = old_size;
    }
#endif

    Task* const prev = rq.pending_handoff_task;
    const TaskState target_state = rq.pending_handoff_state;
    rq.pending_handoff_task = nullptr;
    rq.in_schedule = false;

    if (prev == nullptr) {
      return;
    }

    if (target_state == TaskState::kReady) {
      const IrqSpinLockGuard guard(rq.lock);
      if (prev->is_idle) {
        prev->running_cpu.store(-1, std::memory_order_release);
        prev->state.store(TaskState::kReady, std::memory_order_release);
      } else {
        EnqueueTaskLocked(&rq, prev, false);
      }
      return;
    }

    DCHECK(target_state == TaskState::kExited);
    prev->running_cpu.store(-1, std::memory_order_release);
    if (prev->owns_stack && prev->stack_base != 0) {
      const uintptr_t base = prev->stack_base;
      const int64_t frames = prev->stack_frames;
      prev->stack_base = 0;
      prev->stack_top = 0;
      prev->stack_frames = 0;
#if __STDC_HOSTED__ && defined(__SANITIZE_ADDRESS__)
      __asan_unpoison_memory_region(reinterpret_cast<const void*>(base),
                                    static_cast<size_t>(frames * kPageSize));
#endif
      PmmFreeFrames(base, frames);
      prev->stack_reaped.store(true, std::memory_order_release);
    } else {
      prev->stack_reaped.store(true, std::memory_order_release);
    }
    prev->state.store(TaskState::kExited, std::memory_order_release);
  }

  // Selects the next runnable task on the calling CPU and context-switches to
  // it if a switch is warranted.
  //
  // Parameters:
  // - `outgoing_state`: Target post-switch state for the currently running task
  //   (`prev`):
  //   * `TaskState::kReady` when `prev` remains runnable (cooperative `Yield`,
  //     `PollIdleCpu`, or interrupt preemption). If `prev` is not the idle
  //     task, it switches only if `rq.tree.First()` has strictly smaller
  //     `vruntime` (`candidate->vruntime < prev->vruntime`).
  //   * `TaskState::kExited` when `prev` is terminating (`ExitCurrentTask()`),
  //     in which case `prev` must be switched out to `rq.tree.First()` (or
  //     `rq.idle_task` when `rq.tree` is empty).
  //
  // Returns `true` if a context switch occurred (after `prev` is eventually
  // scheduled back onto a CPU), or `false` if `prev` remained the active task
  // (or if `rq.in_schedule` guarded against nested re-entry).
  [[gnu::noinline]] bool Schedule(const TaskState outgoing_state) {
    // Keep local interrupts disabled across the entire `Schedule()` call—not
    // just while `rq.lock` is held—for three reasons:
    // 1. Before `rq.lock` is acquired: `GetCurrentCpuId()` is read before
    //    `lock_guard` locks `rq.lock`. If an interrupt fired in between, the
    //    ISR could preempt this task and another CPU could steal it, leaving
    //    `cpu_id` and `rq` pointing to the old CPU's runqueue.
    // 2. Across the unlocked handoff window (`TaskContextSwitch` through
    //    `FinishContextSwitch()`): once `lock_guard` releases `rq.lock`,
    //    `rq.current_task` is already `next` while still on `prev`'s stack, and
    //    `FinishContextSwitch()` clears `rq.in_schedule = false` before
    //    re-acquiring `rq.lock` to re-enqueue `prev` (or freeing `prev`'s
    //    stack). An interrupt preempting and migrating `next` there would leave
    //    `FinishContextSwitch()` holding a stale `rq` reference mid-handoff.
    // 3. Preserving per-task `RFLAGS.IF` across cooperative (`IF=1`) <->
    //    ISR-preempted (`IF=0`) switches: if `lock_guard` restored `IF=1`
    //    before `TaskContextSwitch`, a task resuming inside `OnTimerInterrupt`
    //    would run with interrupts enabled before `iretq`; conversely, a task
    //    resuming into `Yield()` from an ISR-preempted task does not execute
    //    `iretq` and relies on `irq_guard` to restore its saved `RFLAGS.IF`.
    const ScopedIrqDisable irq_guard;

    Task* prev = nullptr;
    Task* next = nullptr;
    uintptr_t* prev_rsp_ptr = nullptr;
    uintptr_t next_rsp_val = 0;
    {
      const int cpu_id = GetCurrentCpuId();
      DCHECK(cpu_id >= 0 && cpu_id < cpu_count_);
      CpuRunqueue& rq = runqueues_[cpu_id];
      const IrqSpinLockGuard lock_guard(rq.lock);
      if (rq.in_schedule) {
        return false;
      }
      rq.in_schedule = true;

      prev = rq.current_task;
      DCHECK(prev != nullptr);

      if (outgoing_state == TaskState::kReady) {
        if (prev->is_idle) {
          next = rq.tree.Empty() ? prev : rq.tree.First();
        } else {
          Task* const candidate = rq.tree.First();
          next = (candidate != nullptr && candidate->vruntime < prev->vruntime)
                     ? candidate
                     : prev;
        }
      } else {
        DCHECK(!prev->is_idle);
        next = rq.tree.Empty() ? rq.idle_task : rq.tree.First();
      }

      DCHECK(next != nullptr);
      if (next == prev) {
        // We are on the same task. No need to switch.
        DCHECK(outgoing_state == TaskState::kReady);
        UpdateRunqueueMinVruntime(&rq);
        rq.in_schedule = false;
        return false;
      }
      // We need to do a context switch.

      if (!next->is_idle) {
        rq.tree.Erase(next);
        if (!next->is_bootstrap) {
          rq.runnable_count.fetch_sub(1, std::memory_order_release);
        }
      }
      next->state.store(TaskState::kRunning, std::memory_order_release);
      next->running_cpu.store(cpu_id, std::memory_order_release);
      next->last_cpu.store(cpu_id, std::memory_order_relaxed);
      next->switches.fetch_add(1, std::memory_order_relaxed);
      rq.current_task = next;
      UpdateRunqueueMinVruntime(&rq);

      // SMP stack-safety caveat: Do NOT re-insert `prev` into `rq.tree` (or
      // free its stack on `kExited`) before `TaskContextSwitch` switches `%rsp`
      // off `prev`'s stack. Because `rq.lock` must be released before calling
      // `TaskContextSwitch`, re-enqueuing `prev` here would allow another CPU
      // to steal `prev` and begin executing on `prev`'s stack while this CPU is
      // still saving registers and switching away from it. Instead, stash
      // `prev` in `rq.pending_handoff_task` and finalize its state transition
      // in `FinishContextSwitch()` after landing on `next`'s stack.
      rq.pending_handoff_task = prev;
      rq.pending_handoff_state = outgoing_state;
      EnsureNativeThreadSanitizerState(prev);

      prev_rsp_ptr = &prev->rsp;
      next_rsp_val = next->rsp;
    }

#if __STDC_HOSTED__
#if defined(__SANITIZE_ADDRESS__)
    void** const fake_stack_save = (outgoing_state == TaskState::kExited)
                                       ? nullptr
                                       : &prev->asan_fake_stack;
    __sanitizer_start_switch_fiber(fake_stack_save,          //
                                   next->asan_stack_bottom,  //
                                   next->asan_stack_size);
#endif
#if defined(__SANITIZE_THREAD__)
    __tsan_switch_to_fiber(next->tsan_fiber, 0);
#endif
#endif

    // Note:
    // 1. No locks (including `rq.lock`) may be held across `TaskContextSwitch`,
    //    because it is uncertain when (or if, on `kExited`) `prev` will be
    //    scheduled again, and holding a lock while suspended would block or
    //    deadlock other CPUs and tasks.
    // 2. When `TaskContextSwitch` returns, `prev` may have been stolen and
    //    resumed on a different CPU. Any pre-switch per-CPU state (such as
    //    `cpu_id` or `rq`) is invalid after this point; `FinishContextSwitch()`
    //    must re-query `GetCurrentCpuId()` on the CPU that resumed `prev`.

    // Make sure no locks are held and interrupts are disabled.
    DCHECK(!AreInterruptsEnabled());
    {
      IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
      DCHECK(top_slot == nullptr || *top_slot == nullptr);
    }

    // Do the context switch.
    TaskContextSwitch(prev_rsp_ptr, next_rsp_val);

    // If the task keeps running here, it should not be exiting.
    DCHECK(outgoing_state != TaskState::kExited);

    // Check the preconditions again.
    DCHECK(!AreInterruptsEnabled());
    {
      IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
      DCHECK(top_slot == nullptr || *top_slot == nullptr);
      const int resumed_cpu_id = GetCurrentCpuId();
      DCHECK(resumed_cpu_id >= 0 && resumed_cpu_id < cpu_count_);
      DCHECK(prev->running_cpu.load(std::memory_order_relaxed) ==
             resumed_cpu_id);
      DCHECK(runqueues_[resumed_cpu_id].current_task == prev);
    }

    FinishContextSwitch();
    return true;
  }

  alignas(kPageSize) uint8_t bsp_idle_stack_[kTaskStackSize] = {};
  std::unique_ptr<CpuRunqueue[]> runqueues_;
  Task bootstrap_task_;
  std::unique_ptr<Task[]> idle_tasks_;
  int cpu_count_ = 0;
  std::atomic<int64_t> next_task_id_{1};
  std::atomic<bool> initialized_{false};
};

}  // namespace

extern "C" void TaskStartupEntry(Task* const curr) {
  DCHECK(curr != nullptr);
  DCHECK(curr->owner != nullptr);
  CoreRunqueueTaskScheduler::FinishStartupSwitch(curr->owner);
  internal::LocalRestoreInterrupts(true);

  DCHECK(curr->entry != nullptr);
  curr->entry(curr->arg);

  curr->owner->ExitCurrentTask();
  CHECK(false);
}

std::shared_ptr<TaskScheduler> CreateCoreRunqueueScheduler() {
  return std::make_shared<CoreRunqueueTaskScheduler>();
}

}  // namespace protos
