#ifndef PROTOS_MEMORY_TESTS_H_
#define PROTOS_MEMORY_TESTS_H_

#include <cstdint>

namespace protos {

// Initializes the physical memory manager and kernel heap from the Multiboot
// info at `multiboot_info_addr`, runs all PMM and heap self-tests, and logs
// pass/fail markers over UART COM1.
void RunBootVerificationSuite(uint32_t multiboot_magic,
                              uint64_t multiboot_info_addr);

}  // namespace protos

#endif  // PROTOS_MEMORY_TESTS_H_
