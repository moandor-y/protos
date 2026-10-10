#include "join_lifecycle_scheduler.h"

#include <cstdint>
#include <memory>
#include <new>
#include <utility>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"
#include "heap.h"
#include "rbtree.h"
#include "spinlock.h"

namespace protos {
namespace {

class JoinLifecycleTaskScheduler;

struct TaskJoinRecord {
  JoinLifecycleTaskScheduler* owner = nullptr;
  TaskFn entry = nullptr;
  void* arg = nullptr;
  Task* task = nullptr;
  Task* joiner = nullptr;
  bool exited = false;
  bool joined = false;
  bool in_list = false;
  bool in_tree = false;
  RbNode tree_node{};
  TaskJoinRecord* prev = nullptr;
  TaskJoinRecord* next = nullptr;
};

struct TaskJoinRecordRbTraits {
  static uintptr_t GetKey(const TaskJoinRecord& rec) {
    return reinterpret_cast<uintptr_t>(rec.task);
  }

  static bool Less(const uintptr_t a, const uintptr_t b) { return a < b; }
};

using TaskJoinRecordTree =
    RbTree<TaskJoinRecord, &TaskJoinRecord::tree_node, TaskJoinRecordRbTraits>;

class JoinLifecycleTaskScheduler final : public TaskScheduler {
 public:
  explicit JoinLifecycleTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    DCHECK(inner_ != nullptr);
  }

  ~JoinLifecycleTaskScheduler() override { ClearAllRecords(); }

  void Init() override {
    ClearAllRecords();
    inner_->Init();
  }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    DCHECK(entry != nullptr);
    TaskJoinRecord* const rec = AllocateJoinRecord(entry, arg);
    Task* const task =
        inner_->CreateTask(&JoinLifecycleTaskScheduler::TaskEntryWrapper,  //
                           rec,                                            //
                           weight);
    return FinalizeJoinRecord(rec, task);
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    DCHECK(entry != nullptr);
    TaskJoinRecord* const rec = AllocateJoinRecord(entry, arg);
    Task* const task = inner_->CreateTaskOnCpu(
        &JoinLifecycleTaskScheduler::TaskEntryWrapper,  //
        rec,                                            //
        weight,                                         //
        target_cpu);
    return FinalizeJoinRecord(rec, task);
  }

  void Yield() override { inner_->Yield(); }

  [[gnu::noinline]] void Join(Task* const task) override {
    DCHECK(task != nullptr);
    Task* const self = inner_->CurrentTask();
    DCHECK(self != nullptr);
    DCHECK(task != self);
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(top_slot == nullptr || *top_slot == nullptr);

    TaskJoinRecord* rec = nullptr;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      rec = task_tree_.Find(reinterpret_cast<uintptr_t>(task));
      DCHECK(rec != nullptr);
      DCHECK(rec->in_list && rec->in_tree && rec->task == task);
      DCHECK(!rec->joined);
      DCHECK(rec->joiner == nullptr);
      rec->joiner = self;
    }

    while (!IsRecordExited(rec, task)) {
      inner_->Yield();
#if __STDC_HOSTED__
      std::this_thread::yield();
#else
      asm volatile("pause" : : : "memory");
#endif
    }

    {
      const IrqSpinLockGuard join_guard(join_lock_);
      DCHECK(rec->in_list && rec->in_tree && rec->task == task);
      DCHECK(rec->exited);
      DCHECK(!rec->joined);
      DCHECK(rec->joiner == self);
      rec->joined = true;
      rec->joiner = nullptr;
      EraseFromTreeLocked(rec);
      UnlinkRecordLocked(rec);
    }
    FreeJoinRecord(rec);

    inner_->Join(task);
  }

  int ReapZombies() override {
    TaskJoinRecord* reap_head = nullptr;
    TaskJoinRecord* reap_tail = nullptr;
    int count = 0;

    {
      const IrqSpinLockGuard join_guard(join_lock_);
      TaskJoinRecord* curr = records_head_;
      while (curr != nullptr) {
        TaskJoinRecord* const next = curr->next;
        if (curr->exited && !curr->joined && curr->joiner == nullptr &&
            curr->task != nullptr) {
          curr->joined = true;
          EraseFromTreeLocked(curr);
          UnlinkRecordLocked(curr);
          if (reap_tail != nullptr) {
            reap_tail->next = curr;
          } else {
            reap_head = curr;
          }
          reap_tail = curr;
          ++count;
        }
        curr = next;
      }
    }

    TaskJoinRecord* curr = reap_head;
    while (curr != nullptr) {
      TaskJoinRecord* const next = curr->next;
      Task* const task_to_join = curr->task;
      FreeJoinRecord(curr);
      inner_->Join(task_to_join);
      curr = next;
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
    DCHECK(rec != nullptr);
    const IrqSpinLockGuard join_guard(join_lock_);
    DCHECK(rec->in_list);
    DCHECK(!rec->exited);
    rec->exited = true;
  }

  TaskJoinRecord* AllocateJoinRecord(const TaskFn entry, void* const arg) {
    void* const raw = Kmalloc(sizeof(TaskJoinRecord));
    CHECK(raw != nullptr);
    TaskJoinRecord* const rec = new (raw) TaskJoinRecord();
    rec->owner = this;
    rec->entry = entry;
    rec->arg = arg;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      LinkRecordLocked(rec);
    }
    return rec;
  }

  Task* FinalizeJoinRecord(TaskJoinRecord* const rec, Task* const task) {
    DCHECK(rec != nullptr);
    if (task == nullptr) {
      {
        const IrqSpinLockGuard join_guard(join_lock_);
        UnlinkRecordLocked(rec);
      }
      FreeJoinRecord(rec);
      return nullptr;
    }
    const IrqSpinLockGuard join_guard(join_lock_);
    DCHECK(rec->in_list);
    rec->task = task;
    const bool inserted = task_tree_.Insert(rec);
    DCHECK(inserted);
    rec->in_tree = true;
    return task;
  }

  static void FreeJoinRecord(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    rec->~TaskJoinRecord();
    Kfree(rec);
  }

  void LinkRecordLocked(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    DCHECK(!rec->in_list);
    rec->prev = records_tail_;
    rec->next = nullptr;
    if (records_tail_ != nullptr) {
      records_tail_->next = rec;
    } else {
      records_head_ = rec;
    }
    records_tail_ = rec;
    rec->in_list = true;
  }

  void UnlinkRecordLocked(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    DCHECK(rec->in_list);
    if (rec->prev != nullptr) {
      rec->prev->next = rec->next;
    } else {
      records_head_ = rec->next;
    }
    if (rec->next != nullptr) {
      rec->next->prev = rec->prev;
    } else {
      records_tail_ = rec->prev;
    }
    rec->prev = nullptr;
    rec->next = nullptr;
    rec->in_list = false;
  }

  void EraseFromTreeLocked(TaskJoinRecord* const rec) {
    DCHECK(rec != nullptr);
    if (rec->in_tree) {
      task_tree_.Erase(rec);
      rec->in_tree = false;
    }
  }

  void ClearAllRecords() {
    TaskJoinRecord* head = nullptr;
    {
      const IrqSpinLockGuard join_guard(join_lock_);
      head = records_head_;
      records_head_ = nullptr;
      records_tail_ = nullptr;
      task_tree_.Clear();
    }
    while (head != nullptr) {
      TaskJoinRecord* const next = head->next;
      FreeJoinRecord(head);
      head = next;
    }
  }

  bool IsRecordExited(const TaskJoinRecord* const rec, const Task* const task) {
    const IrqSpinLockGuard join_guard(join_lock_);
    DCHECK(rec != nullptr);
    DCHECK(rec->in_list && rec->task == task);
    return rec->exited;
  }

  const std::shared_ptr<TaskScheduler> inner_;
  IrqSpinLock join_lock_;
  TaskJoinRecordTree task_tree_;
  TaskJoinRecord* records_head_ = nullptr;
  TaskJoinRecord* records_tail_ = nullptr;
};

}  // namespace

std::shared_ptr<TaskScheduler> CreateJoinLifecycleScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<JoinLifecycleTaskScheduler>(std::move(inner));
}

}  // namespace protos
