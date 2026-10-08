#ifndef PROTOS_UART_H_
#define PROTOS_UART_H_

#include <cstdint>

namespace protos {

// Initializes COM1 (0x3F8) at 38400 baud, 8N1, with FIFOs enabled.
void UartInit();
// Writes a single byte to COM1, polling until the transmit holding register is
// empty.
void UartPutc(char c);
// Writes a null-terminated string to COM1, translating '\n' to "\r\n".
void UartWrite(const char* str);
// Writes a 64-bit unsigned integer in hexadecimal ("0x...") to COM1.
void UartWriteHex(uint64_t value);
// Writes a 64-bit unsigned integer in base-10 decimal to COM1.
void UartWriteDec(uint64_t value);
// Lockless emergency panic writer for COM1; must only be called with local
// interrupts disabled in unrecoverable panic paths (e.g., CheckFailure).
void UartPanicWrite(const char* str);
// Lockless emergency base-10 decimal writer for COM1; must only be called with
// local interrupts disabled in unrecoverable panic paths.
void UartPanicWriteDec(uint64_t value);

}  // namespace protos

#endif  // PROTOS_UART_H_
