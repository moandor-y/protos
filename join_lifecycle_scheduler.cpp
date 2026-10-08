#include "join_lifecycle_scheduler.h"

#include <cstdint>
#include <memory>
#include <utility>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"
#include "spinlock.h"

namespace protos {
namespace {

constexpr int kMaxJoinRecords = 1024;

class JoinLifecycleTaskScheduler;

struct TaskJoinRecord {
  JoinLifecycleTaskScheduler* owner = nullptr;
  TaskFn entry = nullptr;
  void* arg = nullptr;
  Task* task = nullptr;
  Task* joiner = nullptr;
  bool ready = false;
  bool exited = false;
  bool joined = false;
  bool in_use = false;
};

class JoinLifecycleTaskScheduler final : public TaskScheduler {
 public:
  explicit JoinLifecycleTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    DCHECK(inner_ != nullptr);
  }

  void Init() override {
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      for (int i = 0; i < kMaxJoinRecords; ++i) {
        join_records_[i] = {};
      }
      next_join_slot_ = 0;
    }
    inner_->Init();
  }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    DCHECK(entry != nullptr);
    int slot = -1;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      slot = AllocateJoinRecordLocked(entry, arg);
    }
    Task* const task =
        inner_->CreateTask(&JoinLifecycleTaskScheduler::TaskEntryWrapper,  //
                           &join_records_[slot],                           //
                           weight);
    const IrqSpinLockGuard join_guard(join_lock_);
    if (task == nullptr) {
      join_records_[slot] = {};
      return nullptr;
    }
    join_records_[slot].task = task;
    join_records_[slot].ready = true;
    return task;
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    DCHECK(entry != nullptr);
    int slot = -1;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      slot = AllocateJoinRecordLocked(entry, arg);
    }
    Task* const task = inner_->CreateTaskOnCpu(
        &JoinLifecycleTaskScheduler::TaskEntryWrapper,  //
        &join_records_[slot],                           //
        weight,                                         //
        target_cpu);
    const IrqSpinLockGuard join_guard(join_lock_);
    if (task == nullptr) {
      join_records_[slot] = {};
      return nullptr;
    }
    join_records_[slot].task = task;
    join_records_[slot].ready = true;
    return task;
  }

  void Yield() override { inner_->Yield(); }

  [[gnu::noinline]] void Join(Task* const task) override {
    DCHECK(task != nullptr);
    Task* const self = inner_->CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(task != self);
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    int task_slot = -1;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      task_slot = FindJoinRecordLocked(task);
      DCHECK(task_slot >= 0);
      TaskJoinRecord& rec = join_records_[task_slot];
      DCHECK(rec.in_use && rec.task == task);
      DCHECK(!rec.joined);
      DCHECK(rec.joiner == nullptr);
      rec.joiner = self;
    }

    while (!IsTaskExited(task_slot, task)) {
      inner_->Yield();
#if __STDC_HOSTED__
      std::this_thread::yield();
#else
      asm volatile("pause" : : : "memory");
#endif
    }

    {
      const IrqSpinLockGuard join_guard(join_lock_);
      TaskJoinRecord& rec = join_records_[task_slot];
      DCHECK(rec.in_use && rec.task == task);
      DCHECK(rec.exited);
      DCHECK(!rec.joined);
      DCHECK(rec.joiner == self);
      rec.joined = true;
      rec.joiner = nullptr;
      rec.task = nullptr;
      rec.in_use = false;
    }

    inner_->Join(task);
  }

  int ReapZombies() override {
    Task* to_free[kMaxJoinRecords];
    int count = 0;

    {
      const IrqSpinLockGuard join_guard(join_lock_);
      for (int i = 0; i < kMaxJoinRecords; ++i) {
        TaskJoinRecord& rec = join_records_[i];
        if (rec.in_use && rec.exited && !rec.joined && rec.joiner == nullptr &&
            rec.task != nullptr) {
          to_free[count++] = rec.task;
          rec.joined = true;
          rec.task = nullptr;
          rec.in_use = false;
        }
      }
    }

    for (int i = 0; i < count; ++i) {
      inner_->Join(to_free[i]);
    }
    return count;
  }

  bool PollIdleCpu(const int cpu_index) override {
    return inner_->PollIdleCpu(cpu_index);
  }

  int RunqueueLoad(const int cpu_index) const override {
    return inner_->RunqueueLoad(cpu_index);
  }

  bool StealTask(const int dst_cpu, const int src_cpu) override {
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

 private:
  static void TaskEntryWrapper(void* const raw_arg) {
    TaskJoinRecord* const rec = static_cast<TaskJoinRecord*>(raw_arg);
    DCHECK(rec != nullptr);
    DCHECK(rec->owner != nullptr);
    DCHECK(rec->entry != nullptr);
    rec->entry(rec->arg);

    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    rec->owner->MarkRecordExited(rec);
  }

  void MarkRecordExited(TaskJoinRecord* const rec) {
    for (;;) {
      {
        const IrqSpinLockGuard join_guard(join_lock_);
        if (rec->ready) {
          DCHECK(rec->in_use && rec->task != nullptr);
          rec->exited = true;
          return;
        }
      }
#if __STDC_HOSTED__
      std::this_thread::yield();
#else
      asm volatile("pause" : : : "memory");
#endif
    }
  }

  int FindJoinRecordLocked(const Task* const task) const {
    for (int i = 0; i < kMaxJoinRecords; ++i) {
      if (join_records_[i].in_use && join_records_[i].ready &&
          join_records_[i].task == task) {
        return i;
      }
    }
    return -1;
  }

  int AllocateJoinRecordLocked(const TaskFn entry, void* const arg) {
    int free_slot = -1;
    for (int i = 0; i < kMaxJoinRecords; ++i) {
      const int idx = (next_join_slot_ + i) % kMaxJoinRecords;
      if (!join_records_[idx].in_use) {
        free_slot = idx;
        break;
      }
    }
    CHECK(free_slot >= 0);
    next_join_slot_ = (free_slot + 1) % kMaxJoinRecords;
    TaskJoinRecord& rec = join_records_[free_slot];
    rec.owner = this;
    rec.entry = entry;
    rec.arg = arg;
    rec.task = nullptr;
    rec.joiner = nullptr;
    rec.ready = false;
    rec.exited = false;
    rec.joined = false;
    rec.in_use = true;
    return free_slot;
  }

  bool IsTaskExited(const int slot, const Task* const task) {
    const IrqSpinLockGuard join_guard(join_lock_);
    const TaskJoinRecord& rec = join_records_[slot];
    DCHECK(rec.in_use && rec.task == task);
    return rec.exited;
  }

  const std::shared_ptr<TaskScheduler> inner_;
  IrqSpinLock join_lock_;
  TaskJoinRecord join_records_[kMaxJoinRecords] = {};
  int next_join_slot_ = 0;
};

}  // namespace

std::shared_ptr<TaskScheduler> CreateJoinLifecycleScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<JoinLifecycleTaskScheduler>(std::move(inner));
}

}  // namespace protos
