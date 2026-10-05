#include <cstdint>

#include "memory_tests.h"
#include "uart.h"
#include "vga.h"

namespace protos {

extern "C" [[noreturn]] void kernel_main(const uint32_t multiboot_magic,
                                         const uint64_t multiboot_info_addr) {
  constexpr const char* kGreeting = "Hello, x86-64 Kernel World!\n";

  VgaClear();
  VgaWrite(kGreeting);

  UartInit();
  UartWrite(kGreeting);

  RunBootVerificationSuite(multiboot_magic, multiboot_info_addr);

  for (;;) {
    asm volatile("cli; hlt");
  }
}

}  // namespace protos
