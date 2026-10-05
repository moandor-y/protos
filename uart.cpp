#include "uart.h"

#include <cstddef>
#include <cstdint>

namespace protos {

namespace {

constexpr uint16_t kCom1Port = 0x3F8;

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
}

void UartPutc(const char c) {
  while ((Inb(kCom1Port + 5) & 0x20) == 0) {
  }
  Outb(kCom1Port, static_cast<uint8_t>(c));
}

void UartWrite(const char* const str) {
  for (size_t i = 0; str[i] != '\0'; ++i) {
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
  size_t count = 0;
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
  size_t count = 0;
  uint64_t remaining = value;
  while (remaining > 0) {
    buffer[count] = static_cast<char>('0' + (remaining % 10));
    remaining /= 10;
    ++count;
  }
  while (count > 0) {
    --count;
    UartPutc(buffer[count]);
  }
}

}  // namespace protos
