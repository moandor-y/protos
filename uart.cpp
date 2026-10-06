#include "uart.h"

#include <cstdint>

namespace protos {

namespace {

constexpr uint16_t kCom1Port = 0x3F8;
constexpr int kMaxTransmitPollIterations = 100000;

bool g_uart_present = false;

static inline void Outb(const uint16_t port, const uint8_t value) {
  asm volatile("out %1, %0" : : "a"(value), "Nd"(port) : "memory");
}

static inline uint8_t Inb(const uint16_t port) {
  uint8_t value = 0;
  asm volatile("in %0, %1" : "=a"(value) : "Nd"(port) : "memory");
  return value;
}

}  // namespace

void UartInit() {
  Outb(kCom1Port + 1, 0x00);
  Outb(kCom1Port + 3, 0x80);
  Outb(kCom1Port + 0, 0x03);
  Outb(kCom1Port + 1, 0x00);
  Outb(kCom1Port + 3, 0x03);
  Outb(kCom1Port + 2, 0xC7);
  Outb(kCom1Port + 4, 0x0B);

  // If the Line Status Register floats high (0xFF), no UART is decoding 0x3F8.
  g_uart_present = (Inb(kCom1Port + 5) != 0xFF);
}

void UartPutc(const char c) {
  if (!g_uart_present) {
    return;
  }
  for (int spin = 0; spin < kMaxTransmitPollIterations; ++spin) {
    if ((Inb(kCom1Port + 5) & 0x20) != 0) {
      Outb(kCom1Port, c);
      return;
    }
    asm volatile("pause");
  }
  // Transmit holding register never became ready (e.g., unclocked Super I/O
  // UART); disable subsequent UART polling to avoid stalling boot.
  g_uart_present = false;
}

void UartWrite(const char* const str) {
  for (int i = 0; str[i] != '\0'; ++i) {
    if (str[i] == '\n') {
      UartPutc('\r');
    }
    UartPutc(str[i]);
  }
}

void UartWriteHex(const uint64_t value) {
  constexpr const char* kHexDigits = "0123456789ABCDEF";
  UartWrite("0x");
  if (value == 0) {
    UartPutc('0');
    return;
  }
  char buffer[16];
  int count = 0;
  uint64_t remaining = value;
  while (remaining > 0) {
    buffer[count] = kHexDigits[remaining & 0xF];
    remaining >>= 4;
    ++count;
  }
  while (count > 0) {
    --count;
    UartPutc(buffer[count]);
  }
}

void UartWriteDec(const uint64_t value) {
  if (value == 0) {
    UartPutc('0');
    return;
  }
  char buffer[20];
  int count = 0;
  uint64_t remaining = value;
  while (remaining > 0) {
    buffer[count] = '0' + (remaining % 10);
    remaining /= 10;
    ++count;
  }
  while (count > 0) {
    --count;
    UartPutc(buffer[count]);
  }
}

}  // namespace protos
