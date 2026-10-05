#ifndef PROTOS_VGA_H_
#define PROTOS_VGA_H_

#include <cstdint>

namespace protos {

// Clears the 80x25 console (both the legacy VGA text buffer at 0xB8000 and any
// attached linear pixel framebuffer) and resets the cursor to (0, 0).
void VgaClear();

// Writes a null-terminated string to the console at the current cursor
// position, advancing the cursor and scrolling as needed.
void VgaWrite(const char* str);

// Attaches a Multiboot2 direct-color linear pixel framebuffer (such as UEFI
// GOP on pure-UEFI machines without legacy 0xB8000 VGA text mode) and renders
// the current 80x25 text buffer onto it.
void VgaAttachFramebuffer(uintptr_t fb_phys_addr,  //
                          uint32_t pitch,          //
                          uint32_t width,          //
                          uint32_t height,         //
                          uint8_t bpp);

}  // namespace protos

#endif  // PROTOS_VGA_H_
