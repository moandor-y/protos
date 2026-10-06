#ifndef PROTOS_MEMORY_TESTS_H_
#define PROTOS_MEMORY_TESTS_H_

#include <cstdint>

namespace protos {

// Runs non-destructive self-tests for the physical memory manager, kernel heap,
// and multiprocessor (SMP) state after kernel subsystems have been initialized,
// logging pass/fail markers over UART COM1 and the display console.
void RunBootVerificationSuite();

}  // namespace protos

#endif  // PROTOS_MEMORY_TESTS_H_
