#include "preemptive_scheduler.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif

#include "check.h"
#include "idt.h"
#include "smp.h"
#include "spinlock.h"

namespace protos {
namespace {

[[gnu::noinline]] static CpuLocal* GetCurrentCpuOrNull() {
  return CurrentCpuOrNull();
}

class PreemptiveTaskScheduler final : public TaskScheduler {
 public:
  explicit PreemptiveTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    DCHECK(inner_ != nullptr);
  }

  void Init() override {
    preempt_enabled_.store(true, std::memory_order_relaxed);
    inner_->Init();
  }

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
    return inner_->StealTask(dst_cpu, src_cpu);
  }

  void SetPreemptEnabled(const bool enabled) override {
    preempt_enabled_.store(enabled, std::memory_order_release);
  }

  bool IsPreemptEnabled() const override {
    return preempt_enabled_.load(std::memory_order_acquire);
  }

  void OnTimerInterrupt(InterruptFrame* const frame) override {
    if (!CanPreemptFromInterrupt(frame)) {
      return;
    }
    inner_->OnTimerInterrupt(frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    if (!CanPreemptFromInterrupt(frame)) {
      return;
    }
    inner_->OnWakeupIpi(frame);
  }

  Task* CurrentTask() const override { return inner_->CurrentTask(); }

 private:
  bool CanPreemptFromInterrupt(const InterruptFrame* const frame) const {
    const CpuLocal* const cpu = GetCurrentCpuOrNull();
    if (cpu == nullptr || !cpu->online) {
      return false;
    }
    if (!preempt_enabled_.load(std::memory_order_acquire)) {
      return false;
    }
    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    if (top_slot != nullptr && *top_slot != nullptr) {
      return false;
    }
#if !__STDC_HOSTED__
    if (frame == nullptr ||
        (frame->rflags & internal::kRflagsInterruptEnableBit) == 0) {
      return false;
    }
    return true;
#else
    if (!internal::LocalAreInterruptsEnabled()) {
      return false;
    }
    if (frame != nullptr && frame->rflags != 0 &&
        (frame->rflags & internal::kRflagsInterruptEnableBit) == 0) {
      return false;
    }
    return true;
#endif
  }

  const std::shared_ptr<TaskScheduler> inner_;
  std::atomic<bool> preempt_enabled_{true};
};

}  // namespace

std::shared_ptr<TaskScheduler> CreatePreemptiveScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<PreemptiveTaskScheduler>(std::move(inner));
}

}  // namespace protos
