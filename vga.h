#ifndef PROTOS_VGA_H_
#define PROTOS_VGA_H_

namespace protos {

// Clears the 80x25 VGA text buffer at physical address 0xB8000 using white-on-black spaces.
void VgaClear();
// Writes a null-terminated string to the VGA text buffer starting at the top-left cell.
void VgaWrite(const char* str);

}  // namespace protos

#endif  // PROTOS_VGA_H_
