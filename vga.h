#ifndef PROTOS_VGA_H_
#define PROTOS_VGA_H_

#include <cstdint>

namespace protos {

// Clears the 80x25 console (both the legacy VGA text buffer at 0xB8000 and any
// attached linear pixel framebuffer) and resets the cursor to (0, 0).
void VgaClear();

// Writes a single character to the console at the current cursor position,
// advancing the cursor and scrolling as needed.
void VgaPutc(char ch);

// Writes a null-terminated string to the console at the current cursor
// position, advancing the cursor and scrolling as needed.
void VgaWrite(const char* str);

// Writes a 64-bit unsigned integer in hexadecimal ("0x...") to the console.
void VgaWriteHex(uint64_t value);

// Writes a 64-bit unsigned integer in base-10 decimal to the console.
void VgaWriteDec(uint64_t value);

// Lockless emergency panic writer for the VGA console; must only be called with
// local interrupts disabled in unrecoverable panic paths (e.g., CheckFailure).
void VgaPanicWrite(const char* str);

// Lockless emergency base-10 decimal writer for the VGA console; must only be
// called with local interrupts disabled in unrecoverable panic paths.
void VgaPanicWriteDec(uint64_t value);

// Initializes and activates a Multiboot2 direct-color linear pixel framebuffer
// (such as UEFI GOP on pure-UEFI machines without legacy 0xB8000 VGA text mode)
// and renders the current 80x25 text buffer onto it. Once active, legacy VGA
// text memory (0xB8000) and CRT cursor I/O ports (0x3D4/0x3D5) are not touched.
void VgaInitFramebuffer(uintptr_t fb_phys_addr,  //
                        int pitch,               //
                        int width,               //
                        int height,              //
                        int bpp);

// Alias for VgaInitFramebuffer used during Multiboot2 framebuffer attachment.
void VgaAttachFramebuffer(uintptr_t fb_phys_addr,  //
                          int pitch,               //
                          int width,               //
                          int height,              //
                          int bpp);

}  // namespace protos

#endif  // PROTOS_VGA_H_
