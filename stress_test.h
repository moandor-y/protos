#ifndef PROTOS_STRESS_TEST_H_
#define PROTOS_STRESS_TEST_H_

#include <cstdint>

namespace protos {

// Returns true if the current x86-64 processor advertises `RDRAND` support via
// `CPUID.01H:ECX[30]`.
bool CpuSupportsRdrand();

// Attempts to read a 64-bit hardware random word via `rdrand` (retrying up to
// 10 times if the hardware RNG FIFO is transiently empty). Writes the result to
// non-null `*out` and returns true on success, or zeroes `*out` and returns
// false cleanly if `RDRAND` is unsupported or retries are exhausted. Validates
// `out != nullptr` with `DCHECK`.
bool TryReadRdrand64(uint64_t* out);

// Reads the 64-bit x86-64 Time-Stamp Counter (`rdtsc`).
uint64_t ReadTsc64();

// Bijective SplitMix64 avalanche mixer combining two 64-bit words `a` and `b`.
uint64_t MixEntropy64(uint64_t a, uint64_t b);

// Harvests a non-zero 64-bit master seed by combining `RDRAND` (when supported
// by the CPU), multiple `RDTSC` readings, per-CPU runtime state (`cpu_id`,
// `timer_ticks`, `ipi_count` when `CurrentCpuOrNull()` is bound), and caller
// `extra_entropy`. Guaranteed to return a non-zero value.
uint64_t HarvestHardwareEntropySeed(uint64_t extra_entropy = 0);

// Fast, deterministic, lock-free 64-bit pseudorandom number generator suitable
// for per-CPU and per-task state without lock contention.
class Prng64 {
 public:
  explicit constexpr Prng64(const uint64_t seed = 1)
      : state_(MixInitialSeed(seed)) {}

  // Generates the next 64-bit pseudorandom word using XorShift64*.
  uint64_t NextU64();

  // Returns an unbiased pseudorandom integer in `[0, bound - 1]`. Validates
  // `bound > 0` with `DCHECK`.
  uint64_t NextBounded(uint64_t bound);

  // Returns an unbiased pseudorandom signed 64-bit integer in
  // `[min_inclusive, max_inclusive]`. Validates `min_inclusive <=
  // max_inclusive` with `DCHECK`.
  int64_t NextRange(int64_t min_inclusive, int64_t max_inclusive);

  // Returns an unbiased pseudorandom signed `int` in
  // `[min_inclusive, max_inclusive]`. Validates `min_inclusive <=
  // max_inclusive` with `DCHECK`.
  int NextIntRange(int min_inclusive, int max_inclusive);

  // Returns a pseudorandom boolean with probability 1/2.
  bool NextBool();

  // Returns true with probability `numerator / denominator`. Validates
  // `denominator > 0` and `0 <= numerator <= denominator` with `DCHECK`.
  bool NextChance(int numerator, int denominator);

  // Returns a pseudorandom power-of-two alignment in `[min_align, max_align]`.
  // Validates with `DCHECK` that `min_align` and `max_align` are positive
  // powers of two with `min_align <= max_align`.
  int64_t NextPowerOfTwoAlignment(int64_t min_align, int64_t max_align);

  // Derives an independent child `Prng64` stream mixed with `stream_id` while
  // advancing this generator's state.
  Prng64 Fork(uint64_t stream_id);

  // Returns the current non-zero internal 64-bit state.
  constexpr uint64_t state() const { return state_; }

 private:
  static constexpr uint64_t MixInitialSeed(const uint64_t seed) {
    uint64_t z = seed + 0x9E3779B97F4A7C15ULL;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    return (z != 0) ? z : 0x9E3779B97F4A7C15ULL;
  }

  uint64_t state_;
};

// Configuration parameters for `ExecuteRandomMultiCpuStress`.
struct StressTestConfig {
  uint64_t seed = 0;    // 0 => harvest via HarvestHardwareEntropySeed()
  int num_workers = 0;  // 0 => 2 * SmpCpuCount()
  int rounds_per_worker = 24;
  int max_live_allocs_per_worker = 4;
  int max_heap_alloc_bytes = 2048;
  int max_pmm_alloc_frames = 3;
  bool boost_lapic_timer = true;
  bool log_to_console = false;
};

// Execution and verification telemetry returned by
// `ExecuteRandomMultiCpuStress`.
struct StressTestStats {
  uint64_t seed = 0;
  int cpu_count = 0;
  uint64_t cpu_participation_mask = 0;
  int64_t cooperative_yields = 0;
  int64_t preemptive_spin_bursts = 0;
  int64_t timer_preemptions_observed = 0;
  int64_t wakeup_ipis_sent = 0;
  int64_t task_migrations_observed = 0;
  int64_t child_tasks_joined = 0;
  int64_t child_tasks_detached = 0;
  int64_t zombies_reaped = 0;
  int64_t kmalloc_ops = 0;
  int64_t kmalloc_aligned_ops = 0;
  int64_t cpp_new_delete_ops = 0;
  int64_t pmm_alloc_ops = 0;
  int64_t payload_verifications = 0;
  int64_t payload_corruptions = 0;
  int64_t stack_canary_corruptions = 0;
  int64_t pmm_free_before = 0;
  int64_t pmm_free_after = 0;
  int64_t heap_free_before = 0;
  int64_t heap_free_after = 0;
};

// Runs the randomized multi-CPU kernel stress workload across all online CPUs
// (`0 .. SmpCpuCount() - 1`), exercising concurrent cooperative and preemptive
// context switches, cross-CPU work stealing, dynamic task creation/joining/
// zombie reaping with randomized weights in `[kMinTaskWeight, kMaxTaskWeight]`,
// and pattern-verified `PMM` and `Heap` allocations. Restores all system
// resources and Local APIC timer settings upon completion and verifies all
// invariants via `CHECK`.
StressTestStats ExecuteRandomMultiCpuStress(const StressTestConfig& config);

// Entry point invoked from `kernel_main` after `RunBootVerificationSuite()`.
// Runs `ExecuteRandomMultiCpuStress` with console logging enabled (using
// `seed_override` if non-zero or hardware entropy when 0) and emits the
// `[STRESS] random_multicpu_stress: PASS` marker to UART and VGA.
void RunRandomMultiCpuStressTest(uint64_t seed_override = 0);

#if __STDC_HOSTED__
// Host unit-test hooks for overriding `CpuSupportsRdrand` and `TryReadRdrand64`
// to deterministically test hardware RNG failure/retry/fallback paths.
using RdrandTestHookFn = bool (*)(uint64_t* out);
void SetRdrandHookForTest(bool force_supported, RdrandTestHookFn hook_fn);
void ResetRdrandHookForTest();
#endif

}  // namespace protos

#endif  // PROTOS_STRESS_TEST_H_
