#include "vga.h"

#include <cstddef>
#include <cstdint>

#include "paging.h"

namespace protos {

namespace {

constexpr uintptr_t kVgaBufferAddress = 0xB8000;
constexpr size_t kVgaWidth = 80;
constexpr size_t kVgaHeight = 25;
constexpr uint8_t kVgaColorWhiteOnBlack = 0x0F;
constexpr uint32_t kGlyphWidth = 8;
constexpr uint32_t kGlyphHeight = 16;

size_t g_cursor_row = 0;
size_t g_cursor_col = 0;
char g_text_shadow[kVgaHeight][kVgaWidth];

uintptr_t g_fb_addr = 0;
uint32_t g_fb_pitch = 0;
uint32_t g_fb_width = 0;
uint32_t g_fb_height = 0;
uint8_t g_fb_bpp = 0;

// Compact 8x8 bitmap font for ASCII 0x20..0x7E (each uint64_t packs 8 rows of
// 8 horizontal pixels, byte 0 on top, bit 0 on the left; rendered at 2x
// vertical scale -> 8x16).
constexpr uint64_t kAsciiFont8x8[95] = {
    0x0000000000000000ULL,  // ' '
    0x180018183C3C3C18ULL,  // '!'
    0x0000000000666666ULL,  // '"'
    0x36367F367F363600ULL,  // '#'
    0x187C063C603E1800ULL,  // '$'
    0x006333180C666300ULL,  // '%'
    0x6E333B6E1C361C00ULL,  // '&'
    0x0000000000181818ULL,  // '\''
    0x30180C0C0C183000ULL,  // '('
    0x0C18303030180C00ULL,  // ')'
    0x00663CFF3C660000ULL,  // '*'
    0x0018187E18180000ULL,  // '+'
    0x0C18180000000000ULL,  // ','
    0x0000007E00000000ULL,  // '-'
    0x0018180000000000ULL,  // '.'
    0x02060C1830604000ULL,  // '/'
    0x3E63737B6F673E00ULL,  // '0'
    0x7E181818181C1800ULL,  // '1'
    0x7F060C3060663C00ULL,  // '2'
    0x3C66603860663C00ULL,  // '3'
    0x30307F33363C3800ULL,  // '4'
    0x3C6660603E067E00ULL,  // '5'
    0x3C66663E06663C00ULL,  // '6'
    0x1818183060667E00ULL,  // '7'
    0x3C66663C66663C00ULL,  // '8'
    0x3C66607C66663C00ULL,  // '9'
    0x0018180000181800ULL,  // ':'
    0x0C18180000181800ULL,  // ';'
    0x30180C060C183000ULL,  // '<'
    0x00007E007E000000ULL,  // '='
    0x060C1830180C0600ULL,  // '>'
    0x1800183060663C00ULL,  // '?'
    0x3E037B7B7B633E00ULL,  // '@'
    0x66667E66663C1800ULL,  // 'A'
    0x3E66663E66663E00ULL,  // 'B'
    0x3C66060606663C00ULL,  // 'C'
    0x3E66666666663E00ULL,  // 'D'
    0x7E06063E06067E00ULL,  // 'E'
    0x0606063E06067E00ULL,  // 'F'
    0x7C66760606663C00ULL,  // 'G'
    0x6666667E66666600ULL,  // 'H'
    0x3C18181818183C00ULL,  // 'I'
    0x1E33333030307800ULL,  // 'J'
    0x66361E0E1E366600ULL,  // 'K'
    0x7E06060606060600ULL,  // 'L'
    0x6363636B7F776300ULL,  // 'M'
    0x6363737B6F676300ULL,  // 'N'
    0x3C66666666663C00ULL,  // 'O'
    0x06063E6666663E00ULL,  // 'P'
    0x603C766666663C00ULL,  // 'Q'
    0x66361E3E66663E00ULL,  // 'R'
    0x3C66603C06663C00ULL,  // 'S'
    0x1818181818187E00ULL,  // 'T'
    0x3C66666666666600ULL,  // 'U'
    0x183C666666666600ULL,  // 'V'
    0x63777F6B63636300ULL,  // 'W'
    0x66663C183C666600ULL,  // 'X'
    0x1818183C66666600ULL,  // 'Y'
    0x7E060C1830607E00ULL,  // 'Z'
    0x3C0C0C0C0C0C3C00ULL,  // '['
    0x406030180C060200ULL,  // '\\'
    0x3C30303030303C00ULL,  // ']'
    0x0000000063361C00ULL,  // '^'
    0x007F000000000000ULL,  // '_'
    0x0000000000180C0CULL,  // '`'
    0x7C667C603C000000ULL,  // 'a'
    0x3E6666663E060600ULL,  // 'b'
    0x3C6606663C000000ULL,  // 'c'
    0x7C6666667C606000ULL,  // 'd'
    0x3C067E663C000000ULL,  // 'e'
    0x1C18187C18381800ULL,  // 'f'
    0x3C607C66667C0000ULL,  // 'g'
    0x666666663E060600ULL,  // 'h'
    0x3C1818181C001800ULL,  // 'i'
    0x1C36303030380030ULL,  // 'j'
    0x66361E3666060600ULL,  // 'k'
    0x3C18181818181C00ULL,  // 'l'
    0x6B6B7F7763000000ULL,  // 'm'
    0x666666663E000000ULL,  // 'n'
    0x3C6666663C000000ULL,  // 'o'
    0x06063E66663E0000ULL,  // 'p'
    0x60607C66667C0000ULL,  // 'q'
    0x060606663B000000ULL,  // 'r'
    0x3E603C067C000000ULL,  // 's'
    0x381818187C181800ULL,  // 't'
    0x7C66666666000000ULL,  // 'u'
    0x183C666666000000ULL,  // 'v'
    0x367F6B6363000000ULL,  // 'w'
    0x663C183C66000000ULL,  // 'x'
    0x3C607C6666000000ULL,  // 'y'
    0x7E0C18307E000000ULL,  // 'z'
    0x3818180C18183800ULL,  // '{'
    0x1818180018181800ULL,  // '|'
    0x0E18183018180E00ULL,  // '}'
    0x0000003B6E000000ULL,  // '~'
};

static void RenderCellToFramebuffer(const size_t row,  //
                                    const size_t col,  //
                                    const char ch) {
  if (g_fb_addr == 0 || (g_fb_bpp != 32 && g_fb_bpp != 24)) {
    return;
  }
  const uint32_t base_x = static_cast<uint32_t>(col) * kGlyphWidth;
  const uint32_t base_y = static_cast<uint32_t>(row) * kGlyphHeight;
  if (base_x + kGlyphWidth > g_fb_width ||
      base_y + kGlyphHeight > g_fb_height) {
    return;
  }

  const uint8_t u = static_cast<uint8_t>(ch);
  const uint64_t glyph =
      (u >= 0x20 && u <= 0x7E) ? kAsciiFont8x8[u - 0x20] : 0ULL;

  volatile uint8_t* const fb_base =
      reinterpret_cast<volatile uint8_t*>(g_fb_addr);
  const uint32_t bytes_per_pixel = g_fb_bpp / 8;

  for (uint32_t py = 0; py < kGlyphHeight; ++py) {
    const uint8_t row_bits =
        static_cast<uint8_t>((glyph >> ((py / 2) * 8)) & 0xFF);
    volatile uint8_t* const line = fb_base + (base_y + py) * g_fb_pitch;
    for (uint32_t px = 0; px < kGlyphWidth; ++px) {
      const bool lit = (row_bits & (1U << px)) != 0;
      const uint8_t intensity = lit ? 0xFF : 0x00;
      volatile uint8_t* const pixel = line + (base_x + px) * bytes_per_pixel;
      pixel[0] = intensity;
      pixel[1] = intensity;
      pixel[2] = intensity;
      if (bytes_per_pixel == 4) {
        pixel[3] = 0x00;
      }
    }
  }
}

static void WriteCell(const size_t row, const size_t col, const char ch) {
  g_text_shadow[row][col] = ch;
  volatile uint16_t* const vga =
      reinterpret_cast<volatile uint16_t*>(kVgaBufferAddress);
  vga[row * kVgaWidth + col] =
      (static_cast<uint16_t>(kVgaColorWhiteOnBlack) << 8) |
      static_cast<uint8_t>(ch);
  RenderCellToFramebuffer(row, col, ch);
}

static void ScrollUpOneRow() {
  for (size_t r = 0; r + 1 < kVgaHeight; ++r) {
    for (size_t c = 0; c < kVgaWidth; ++c) {
      WriteCell(r, c, g_text_shadow[r + 1][c]);
    }
  }
  for (size_t c = 0; c < kVgaWidth; ++c) {
    WriteCell(kVgaHeight - 1, c, ' ');
  }
}

}  // namespace

void VgaClear() {
  g_cursor_row = 0;
  g_cursor_col = 0;
  for (size_t r = 0; r < kVgaHeight; ++r) {
    for (size_t c = 0; c < kVgaWidth; ++c) {
      WriteCell(r, c, ' ');
    }
  }
}

void VgaPutc(const char ch) {
  if (ch == '\r') {
    g_cursor_col = 0;
    return;
  }
  if (ch == '\n') {
    g_cursor_col = 0;
    ++g_cursor_row;
    if (g_cursor_row >= kVgaHeight) {
      ScrollUpOneRow();
      g_cursor_row = kVgaHeight - 1;
    }
    return;
  }
  WriteCell(g_cursor_row, g_cursor_col, ch);
  ++g_cursor_col;
  if (g_cursor_col >= kVgaWidth) {
    g_cursor_col = 0;
    ++g_cursor_row;
    if (g_cursor_row >= kVgaHeight) {
      ScrollUpOneRow();
      g_cursor_row = kVgaHeight - 1;
    }
  }
}

void VgaWrite(const char* const str) {
  if (str == nullptr) {
    return;
  }
  for (size_t i = 0; str[i] != '\0'; ++i) {
    VgaPutc(str[i]);
  }
}

void VgaWriteHex(const uint64_t value) {
  constexpr const char* kHexDigits = "0123456789ABCDEF";
  VgaWrite("0x");
  if (value == 0) {
    VgaPutc('0');
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
    VgaPutc(buffer[count]);
  }
}

void VgaWriteDec(const uint64_t value) {
  if (value == 0) {
    VgaPutc('0');
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
    VgaPutc(buffer[count]);
  }
}

void VgaAttachFramebuffer(const uintptr_t fb_phys_addr,  //
                          const uint32_t pitch,          //
                          const uint32_t width,          //
                          const uint32_t height,         //
                          const uint8_t bpp) {
  if (fb_phys_addr == 0 || pitch == 0 || width < kVgaWidth * kGlyphWidth ||
      height < kVgaHeight * kGlyphHeight || (bpp != 32 && bpp != 24)) {
    return;
  }
  const size_t fb_bytes = static_cast<size_t>(pitch) * height;
  if (!PagingMapBootstrapRange(fb_phys_addr, fb_bytes)) {
    return;
  }
  g_fb_addr = fb_phys_addr;
  g_fb_pitch = pitch;
  g_fb_width = width;
  g_fb_height = height;
  g_fb_bpp = bpp;

  for (size_t r = 0; r < kVgaHeight; ++r) {
    for (size_t c = 0; c < kVgaWidth; ++c) {
      const char ch = (g_text_shadow[r][c] != '\0') ? g_text_shadow[r][c] : ' ';
      RenderCellToFramebuffer(r, c, ch);
    }
  }
}

}  // namespace protos
