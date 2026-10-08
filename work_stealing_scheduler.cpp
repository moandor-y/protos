#include "work_stealing_scheduler.h"

#include <climits>
#include <cstdint>
#include <memory>
#include <utility>
#if !__STDC_HOSTED__
#pragma GCC optimize("no-tree-loop-distribute-patterns")
#endif

#include "check.h"
#include "idt.h"
#include "smp.h"

namespace protos {
namespace {

[[gnu::noinline]] static int GetCurrentCpuId() { return CurrentCpuId(); }

[[gnu::noinline]] static CpuLocal* GetCurrentCpuOrNull() {
  return CurrentCpuOrNull();
}

static bool IsCpuOnline(const int cpu_index, const int cpu_count) {
  if (cpu_index < 0 || cpu_index >= cpu_count) {
    return false;
  }
  const CpuLocal* const local = SmpGetCpuLocal(cpu_index);
  return local != nullptr && local->online;
}

class WorkStealingTaskScheduler final : public TaskScheduler {
 public:
  explicit WorkStealingTaskScheduler(std::shared_ptr<TaskScheduler> inner)
      : inner_(std::move(inner)) {
    DCHECK(inner_ != nullptr);
  }

  void Init() override { inner_->Init(); }

  Task* CreateTask(const TaskFn entry,  //
                   void* const arg,     //
                   const int64_t weight) override {
    DCHECK(entry != nullptr);
    DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
    const int target_cpu = SelectLeastLoadedCpu();
    return CreateTaskOnCpu(entry, arg, weight, target_cpu);
  }

  Task* CreateTaskOnCpu(const TaskFn entry,    //
                        void* const arg,       //
                        const int64_t weight,  //
                        const int target_cpu) override {
    DCHECK(entry != nullptr);
    DCHECK(weight >= kMinTaskWeight && weight <= kMaxTaskWeight);
    const int cpu_count = SmpCpuCount();
    DCHECK(target_cpu >= 0 && target_cpu < cpu_count);
    DCHECK(IsCpuOnline(target_cpu, cpu_count));

    Task* const task = inner_->CreateTaskOnCpu(entry, arg, weight, target_cpu);
    SendWakeupIpiIfRemote(target_cpu);
    return task;
  }

  void Yield() override {
    const int cpu_id = GetCurrentCpuId();
    if (inner_->RunqueueLoad(cpu_id) == 0) {
      (void)TryStealForCpu(cpu_id);
    }
    inner_->Yield();
  }

  void Join(Task* const task) override { inner_->Join(task); }

  int ReapZombies() override { return inner_->ReapZombies(); }

  bool PollIdleCpu(const int cpu_index) override {
    const int cpu_count = SmpCpuCount();
    if (cpu_index < 0 || cpu_index >= cpu_count) {
      return false;
    }
    if (inner_->RunqueueLoad(cpu_index) == 0) {
      if (!TryStealForCpu(cpu_index)) {
        return false;
      }
    }
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
    const CpuLocal* const cpu = GetCurrentCpuOrNull();
    if (cpu != nullptr && cpu->online &&
        inner_->RunqueueLoad(cpu->cpu_id) == 0) {
      (void)TryStealForCpu(cpu->cpu_id);
    }
    inner_->OnTimerInterrupt(frame);
  }

  void OnWakeupIpi(InterruptFrame* const frame) override {
    const CpuLocal* const cpu = GetCurrentCpuOrNull();
    if (cpu != nullptr && cpu->online &&
        inner_->RunqueueLoad(cpu->cpu_id) == 0) {
      (void)TryStealForCpu(cpu->cpu_id);
    }
    inner_->OnWakeupIpi(frame);
  }

  Task* CurrentTask() const override { return inner_->CurrentTask(); }

 private:
  int SelectLeastLoadedCpu() const {
    const int cpu_count = SmpCpuCount();
    DCHECK(cpu_count >= 1);
    int best_cpu = 0;
    int best_load = inner_->RunqueueLoad(0);
    for (int cpu = 1; cpu < cpu_count; ++cpu) {
      if (!IsCpuOnline(cpu, cpu_count)) {
        continue;
      }
      const int load = inner_->RunqueueLoad(cpu);
      if (load < best_load) {
        best_load = load;
        best_cpu = cpu;
      }
    }
    return best_cpu;
  }

  static void SendWakeupIpiIfRemote(const int target_cpu) {
    const CpuLocal* const local = GetCurrentCpuOrNull();
    const int current_cpu = (local != nullptr) ? local->cpu_id : -1;
    if (target_cpu != current_cpu) {
      SmpSendIpi(target_cpu, kVectorWakeupIpi);
    }
  }

  bool TryStealForCpu(const int dst_cpu) {
    const int cpu_count = SmpCpuCount();
    if (dst_cpu < 0 || dst_cpu >= cpu_count || cpu_count <= 1) {
      return false;
    }

    int prev_load = INT_MAX;
    int prev_src = -1;

    for (int attempt = 0; attempt < cpu_count - 1; ++attempt) {
      int best_src = -1;
      int best_load = 0;
      for (int cpu = 0; cpu < cpu_count; ++cpu) {
        if (cpu == dst_cpu || !IsCpuOnline(cpu, cpu_count)) {
          continue;
        }
        const int load = inner_->RunqueueLoad(cpu);
        const bool already_tried =
            (load > prev_load) || (load == prev_load && cpu <= prev_src);
        if (already_tried) {
          continue;
        }
        if (load > best_load) {
          best_load = load;
          best_src = cpu;
        }
      }
      if (best_src < 0) {
        break;
      }
      prev_load = best_load;
      prev_src = best_src;
      if (inner_->StealTask(dst_cpu, best_src)) {
        return true;
      }
    }
    return false;
  }

  const std::shared_ptr<TaskScheduler> inner_;
};

}  // namespace

std::shared_ptr<TaskScheduler> CreateWorkStealingScheduler(
    std::shared_ptr<TaskScheduler> inner) {
  return std::make_shared<WorkStealingTaskScheduler>(std::move(inner));
}

}  // namespace protos
