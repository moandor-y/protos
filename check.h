#ifndef PROTOS_CHECK_H_
#define PROTOS_CHECK_H_

#if __STDC_HOSTED__
#include <cstdio>
#include <cstdlib>
#else
#include <cstdint>

#include "uart.h"
#include "vga.h"
#endif

namespace protos {
namespace internal {

[[noreturn]] inline void CheckFailure(const char* const expr,  //
                                      const char* const file,  //
                                      const int line) {
#if __STDC_HOSTED__
  std::fprintf(stderr, "[FATAL] Check failed: %s at %s:%d\n", expr, file, line);
  std::fflush(stderr);
  std::_Exit(134);
#else
  asm volatile("cli" : : : "memory", "cc");
  const uint64_t line_num = (line > 0) ? static_cast<uint64_t>(line) : 0;

  UartPanicWrite("[FATAL] Check failed: ");
  UartPanicWrite(expr);
  UartPanicWrite(" at ");
  UartPanicWrite(file);
  UartPanicWrite(":");
  UartPanicWriteDec(line_num);
  UartPanicWrite("\n");

  VgaPanicWrite("[FATAL] Check failed: ");
  VgaPanicWrite(expr);
  VgaPanicWrite(" at ");
  VgaPanicWrite(file);
  VgaPanicWrite(":");
  VgaPanicWriteDec(line_num);
  VgaPanicWrite("\n");

  for (;;) {
    asm volatile("cli; hlt");
  }
#endif
}

}  // namespace internal
}  // namespace protos

#define CHECK(condition)                                                \
  do {                                                                  \
    if (__builtin_expect(!static_cast<bool>(condition), 0)) {           \
      ::protos::internal::CheckFailure(#condition, __FILE__, __LINE__); \
    }                                                                   \
  } while (false)

#if defined(NDEBUG)
#define DCHECK(condition)                        \
  do {                                           \
    if (false && static_cast<bool>(condition)) { \
    }                                            \
  } while (false)
#else
#define DCHECK(condition) CHECK(condition)
#endif

#endif  // PROTOS_CHECK_H_
