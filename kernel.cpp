#include <cstdint>

#include "check.h"
#include "heap.h"
#include "idt.h"
#include "memory_tests.h"
#include "pmm.h"
#include "smp.h"
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

  IdtInit();
  PmmInit(multiboot_magic, multiboot_info_addr);
  HeapInit();
  SmpInit(multiboot_magic, multiboot_info_addr);

  RunBootVerificationSuite();

  for (;;) {
    asm volatile("cli; hlt");
  }
}

}  // namespace protos
