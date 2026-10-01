#include <cstddef>
#include <cstdint>

namespace protos {

namespace {

constexpr uint16_t kCom1Port = 0x3F8;
constexpr uintptr_t kVgaBufferAddress = 0xB8000;
constexpr size_t kVgaWidth = 80;
constexpr size_t kVgaHeight = 25;
constexpr uint8_t kVgaColorWhiteOnBlack = 0x0F;

static inline void outb(
    const uint16_t port,  //
    const uint8_t value   //
) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

static inline uint8_t inb(const uint16_t port) {
  uint8_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

static void uart_init() {
  outb(kCom1Port + 1, 0x00);  // Disable all UART interrupts (IER = 0)
  outb(kCom1Port + 3, 0x80);  // Enable DLAB (bit 7 of LCR) to set baud divisor
  outb(kCom1Port + 0, 0x03);  // Divisor low byte: 3 (38400 baud)
  outb(kCom1Port + 1, 0x00);  // Divisor high byte: 0
  outb(kCom1Port + 3, 0x03);  // Clear DLAB, set 8N1 (8 bits, no parity, 1 stop)
  outb(kCom1Port + 2, 0xC7);  // Enable & clear FIFOs, 14-byte threshold
  outb(kCom1Port + 4, 0x0B);  // Assert RTS/DSR and OUT2 in Modem Control Reg
}

static void uart_putc(const char c) {
  // Poll Line Status Register (0x3FD) until bit 5 (THRE) is set
  while ((inb(kCom1Port + 5) & 0x20) == 0) {
  }
  outb(kCom1Port, static_cast<uint8_t>(c));
}

static void uart_write(const char* const str) {
  for (size_t i = 0; str[i] != '\0'; ++i) {
    if (str[i] == '\n') {
      uart_putc('\r');
    }
    uart_putc(str[i]);
  }
}

static void vga_clear() {
  volatile uint16_t* const vga =
      reinterpret_cast<volatile uint16_t*>(kVgaBufferAddress);
  const uint16_t blank =
      (static_cast<uint16_t>(kVgaColorWhiteOnBlack) << 8) |
      static_cast<uint8_t>(' ');
  for (size_t i = 0; i < kVgaWidth * kVgaHeight; ++i) {
    vga[i] = blank;
  }
}

static void vga_write(const char* const str) {
  volatile uint16_t* const vga =
      reinterpret_cast<volatile uint16_t*>(kVgaBufferAddress);
  size_t index = 0;
  for (size_t i = 0; str[i] != '\0'; ++i) {
    if (str[i] == '\n') {
      index = ((index / kVgaWidth) + 1) * kVgaWidth;
      continue;
    }
    if (index < kVgaWidth * kVgaHeight) {
      vga[index] =
          (static_cast<uint16_t>(kVgaColorWhiteOnBlack) << 8) |
          static_cast<uint8_t>(str[i]);
      ++index;
    }
  }
}

}  // namespace

extern "C" [[noreturn]] void kernel_main() {
  constexpr const char* kGreeting = "Hello, x86-64 Kernel World!\n";

  vga_clear();
  vga_write(kGreeting);

  uart_init();
  uart_write(kGreeting);

  for (;;) {
    asm volatile("cli; hlt");
  }
}

}  // namespace protos
