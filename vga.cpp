#include "vga.h"

#include <cstddef>
#include <cstdint>

namespace protos {

namespace {

constexpr uintptr_t kVgaBufferAddress = 0xB8000;
constexpr size_t kVgaWidth = 80;
constexpr size_t kVgaHeight = 25;
constexpr uint8_t kVgaColorWhiteOnBlack = 0x0F;

}  // namespace

void VgaClear() {
  volatile uint16_t* const vga =
      reinterpret_cast<volatile uint16_t*>(kVgaBufferAddress);
  const uint16_t blank = (static_cast<uint16_t>(kVgaColorWhiteOnBlack) << 8) |
                         static_cast<uint8_t>(' ');
  for (size_t i = 0; i < kVgaWidth * kVgaHeight; ++i) {
    vga[i] = blank;
  }
}

void VgaWrite(const char* const str) {
  volatile uint16_t* const vga =
      reinterpret_cast<volatile uint16_t*>(kVgaBufferAddress);
  size_t index = 0;
  for (size_t i = 0; str[i] != '\0'; ++i) {
    if (str[i] == '\n') {
      index = ((index / kVgaWidth) + 1) * kVgaWidth;
      continue;
    }
    if (index < kVgaWidth * kVgaHeight) {
      vga[index] = (static_cast<uint16_t>(kVgaColorWhiteOnBlack) << 8) |
                   static_cast<uint8_t>(str[i]);
      ++index;
    }
  }
}

}  // namespace protos
