#ifndef PROTOS_SPINLOCK_H_
#define PROTOS_SPINLOCK_H_

#include <atomic>
#include <cstdint>
#if __STDC_HOSTED__
#include <thread>
#endif

#include "check.h"

namespace protos {

class IrqSpinLock;

namespace internal {

constexpr uint64_t kRflagsInterruptEnableBit = 1ULL << 9;

#if !__STDC_HOSTED__
inline IrqSpinLock* g_per_cpu_top_lock[256] = {};

inline uint8_t CurrentCpuApicId() {
  uint32_t eax = 1;
  uint32_t ebx = 0;
  uint32_t ecx = 0;
  uint32_t edx = 0;
  asm volatile("cpuid" : "+a"(eax), "=b"(ebx), "=c"(ecx), "=d"(edx) : : "cc");
  return static_cast<uint8_t>((ebx >> 24) & 0xFFu);
}

inline bool LocalAreInterruptsEnabled() {
  uint64_t rflags = 0;
  asm volatile(
      "pushfq\n\t"
      "pop %0"
      : "=r"(rflags)
      :
      : "memory");
  return (rflags & kRflagsInterruptEnableBit) != 0;
}

inline bool LocalSaveAndDisableInterrupts() {
  uint64_t rflags = 0;
  asm volatile(
      "pushfq\n\t"
      "pop %0\n\t"
      "cli"
      : "=r"(rflags)
      :
      : "memory", "cc");
  return (rflags & kRflagsInterruptEnableBit) != 0;
}

inline void LocalRestoreInterrupts(const bool previously_enabled) {
  if (previously_enabled) {
    asm volatile("sti" : : : "memory", "cc");
  }
}

inline uintptr_t CurrentOwnerId() {
  return static_cast<uintptr_t>(CurrentCpuApicId()) + 1u;
}

inline IrqSpinLock** TopHeldLockSlot() {
  return &g_per_cpu_top_lock[CurrentCpuApicId()];
}
#else
[[gnu::noinline]] inline bool* HostInterruptEnabledSlot() {
  thread_local bool interrupts_enabled = true;
  asm volatile("" : "+m"(interrupts_enabled));
  return &interrupts_enabled;
}

inline bool LocalAreInterruptsEnabled() { return *HostInterruptEnabledSlot(); }

inline void LocalSetInterruptsEnabledForTest(const bool enabled) {
  *HostInterruptEnabledSlot() = enabled;
}

inline bool LocalSaveAndDisableInterrupts() {
  bool* const slot = HostInterruptEnabledSlot();
  const bool previously_enabled = *slot;
  *slot = false;
  return previously_enabled;
}

inline void LocalRestoreInterrupts(const bool previously_enabled) {
  if (previously_enabled) {
    *HostInterruptEnabledSlot() = true;
  }
}

[[gnu::noinline]] inline uintptr_t CurrentOwnerId() {
  thread_local uint8_t thread_token = 0;
  asm volatile("" : "+m"(thread_token));
  return reinterpret_cast<uintptr_t>(&thread_token);
}

[[gnu::noinline]] inline IrqSpinLock** TopHeldLockSlot() {
  thread_local IrqSpinLock* top_lock = nullptr;
  asm volatile("" : "+m"(top_lock));
  return &top_lock;
}
#endif

}  // namespace internal

// Returns true if local interrupts are currently enabled on the calling CPU
// (or simulated as enabled on the calling host thread).
inline bool AreInterruptsEnabled() {
  return internal::LocalAreInterruptsEnabled();
}

#if __STDC_HOSTED__
// Sets the simulated per-thread interrupt-enable flag in host unit tests.
inline void SetInterruptsEnabledForTest(const bool enabled) {
  internal::LocalSetInterruptsEnabledForTest(enabled);
}
#endif

// Interrupt-safe FIFO ticket spinlock.
//
// Saves the caller's interrupt state (`RFLAGS.IF`) and disables local
// interrupts (`cli`) prior to acquiring the ticket lock, and restores the
// previous interrupt state upon release. All fields are zero-initialized so
// global `IrqSpinLock` instances reside in `.bss` without runtime constructors.
class IrqSpinLock {
 public:
  constexpr IrqSpinLock() = default;

  IrqSpinLock(const IrqSpinLock&) = delete;
  IrqSpinLock& operator=(const IrqSpinLock&) = delete;
  IrqSpinLock(IrqSpinLock&&) = delete;
  IrqSpinLock& operator=(IrqSpinLock&&) = delete;

  void Lock() {
    const bool irq_was_enabled = internal::LocalSaveAndDisableInterrupts();
    const uintptr_t current_owner = internal::CurrentOwnerId();
    DCHECK(current_owner != 0);
    DCHECK(owner_.load(std::memory_order_relaxed) != current_owner);

    const int64_t ticket = next_ticket_.fetch_add(1, std::memory_order_relaxed);
#if !__STDC_HOSTED__
    while (serving_ticket_.load(std::memory_order_acquire) != ticket) {
      asm volatile("pause" : : : "memory");
    }
#else
    int spin_iters = 0;
    while (serving_ticket_.load(std::memory_order_acquire) != ticket) {
      asm volatile("pause" : : : "memory");
      if ((++spin_iters & 63) == 0) {
        std::this_thread::yield();
      }
    }
#endif

    DCHECK(!locked_);
    DCHECK(owner_.load(std::memory_order_relaxed) == 0);
    DCHECK(!internal::LocalAreInterruptsEnabled());

    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    prev_held_lock_ = *top_slot;
    *top_slot = this;
    saved_irq_enabled_ = irq_was_enabled;
    locked_ = true;
    owner_.store(current_owner, std::memory_order_relaxed);
  }

  void Unlock() {
    const uintptr_t current_owner = internal::CurrentOwnerId();
    DCHECK(current_owner != 0);
    DCHECK(owner_.load(std::memory_order_relaxed) == current_owner);
    DCHECK(locked_);
    DCHECK(!internal::LocalAreInterruptsEnabled());

    IrqSpinLock** const top_slot = internal::TopHeldLockSlot();
    DCHECK(*top_slot == this);
    *top_slot = prev_held_lock_;
    prev_held_lock_ = nullptr;

    const bool irq_was_enabled = saved_irq_enabled_;
    saved_irq_enabled_ = false;
    locked_ = false;
    owner_.store(0, std::memory_order_relaxed);

    const int64_t next_serving =
        serving_ticket_.load(std::memory_order_relaxed) + 1;
    serving_ticket_.store(next_serving, std::memory_order_release);

    internal::LocalRestoreInterrupts(irq_was_enabled);
  }

  bool IsLocked() const { return owner_.load(std::memory_order_acquire) != 0; }

  bool IsLockedByCurrentCpu() const {
    return owner_.load(std::memory_order_relaxed) == internal::CurrentOwnerId();
  }

  int64_t NextTicket() const {
    return next_ticket_.load(std::memory_order_relaxed);
  }

  int64_t ServingTicket() const {
    return serving_ticket_.load(std::memory_order_acquire);
  }

  static bool AreInterruptsEnabled() {
    return internal::LocalAreInterruptsEnabled();
  }

#if __STDC_HOSTED__
  static void SetInterruptsEnabledForTest(const bool enabled) {
    internal::LocalSetInterruptsEnabledForTest(enabled);
  }
#endif

 private:
  std::atomic<int64_t> next_ticket_{0};
  std::atomic<int64_t> serving_ticket_{0};
  std::atomic<uintptr_t> owner_{0};
  IrqSpinLock* prev_held_lock_ = nullptr;
  bool saved_irq_enabled_ = false;
  bool locked_ = false;
};

// RAII scoped guard for `IrqSpinLock`.
class IrqSpinLockGuard {
 public:
  explicit IrqSpinLockGuard(IrqSpinLock& lock) : lock_(&lock) { lock_->Lock(); }

  explicit IrqSpinLockGuard(IrqSpinLock* const lock) : lock_(lock) {
    DCHECK(lock_ != nullptr);
    lock_->Lock();
  }

  ~IrqSpinLockGuard() { lock_->Unlock(); }

  IrqSpinLockGuard(const IrqSpinLockGuard&) = delete;
  IrqSpinLockGuard& operator=(const IrqSpinLockGuard&) = delete;
  IrqSpinLockGuard(IrqSpinLockGuard&&) = delete;
  IrqSpinLockGuard& operator=(IrqSpinLockGuard&&) = delete;

 private:
  IrqSpinLock* const lock_;
};

}  // namespace protos

#endif  // PROTOS_SPINLOCK_H_
