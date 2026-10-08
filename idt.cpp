#include "idt.h"

#include <atomic>
#include <cstdint>
#if __STDC_HOSTED__
#include <cstdio>
#include <cstdlib>
#endif

#include "check.h"
#include "smp.h"
#include "uart.h"
#include "vga.h"

#if __STDC_HOSTED__
extern "C" {
uint64_t g_isr_stub_table[protos::kIdtEntryCount] = {};
}
#endif

namespace protos {

namespace {

constexpr uint8_t kMaxIstIndex = 7;

alignas(16) IdtGateDescriptor g_idt[kIdtEntryCount] = {};
IdtPointer g_idt_pointer = {};
std::atomic<InterruptHandlerFn> g_handlers[kIdtEntryCount] = {};
std::atomic<bool> g_idt_initialized{false};

#if __STDC_HOSTED__
std::atomic<int> g_host_idt_load_count{0};
std::atomic<uint64_t> g_host_cr2{0};
#endif

static bool IsValidGateTypeAttr(const uint8_t type_attr) {
  // Must have Present=1 (bit 7), S=0 (bit 4), and 64-bit Interrupt Gate (0xE)
  // or 64-bit Trap Gate (0xF) in bits 0..3.
  const uint8_t gate_type = static_cast<uint8_t>(type_attr & 0x0Fu);
  return (type_attr & 0x90u) == 0x80u &&
         (gate_type == 0x0Eu || gate_type == 0x0Fu);
}

static uint64_t ReadCr2() {
#if !__STDC_HOSTED__
  uint64_t cr2 = 0;
  asm volatile("mov %0, cr2" : "=r"(cr2));
  return cr2;
#else
  return g_host_cr2.load(std::memory_order_relaxed);
#endif
}

static void PanicWriteStr(const char* const str) {
  UartPanicWrite(str);
  VgaPanicWrite(str);
}

static void PanicWriteDec(const uint64_t value) {
  UartPanicWriteDec(value);
  VgaPanicWriteDec(value);
}

static void PanicWriteHex(const uint64_t value) {
  constexpr const char* kHexDigits = "0123456789ABCDEF";
  char buf[19];
  buf[0] = '0';
  buf[1] = 'x';
  if (value == 0) {
    buf[2] = '0';
    buf[3] = '\0';
    PanicWriteStr(buf);
    return;
  }
  char rev[16];
  int count = 0;
  uint64_t remaining = value;
  while (remaining > 0) {
    rev[count] = kHexDigits[remaining & 0xFu];
    remaining >>= 4;
    ++count;
  }
  for (int i = 0; i < count; ++i) {
    buf[2 + i] = rev[count - 1 - i];
  }
  buf[2 + count] = '\0';
  PanicWriteStr(buf);
}

[[noreturn]] static void PanicUnhandledException(
    const InterruptFrame* const frame) {
#if !__STDC_HOSTED__
  asm volatile("cli" : : : "memory", "cc");
#endif
  const int vector = static_cast<int>(frame->vector);
  const char* const mnemonic = IdtExceptionMnemonic(vector);
  const uint64_t cr2 = ReadCr2();
  const CpuLocal* const cpu = CurrentCpuOrNull();
  const int cpu_id = (cpu != nullptr) ? cpu->cpu_id : -1;

  PanicWriteStr("[FATAL] Unhandled CPU exception ");
  PanicWriteDec(frame->vector);
  PanicWriteStr(" (");
  PanicWriteStr(mnemonic);
  PanicWriteStr(") err=");
  PanicWriteHex(frame->error_code);
  PanicWriteStr(" rip=");
  PanicWriteHex(frame->rip);
  PanicWriteStr(" rsp=");
  PanicWriteHex(frame->rsp);
  PanicWriteStr(" rflags=");
  PanicWriteHex(frame->rflags);
  PanicWriteStr(" cr2=");
  PanicWriteHex(cr2);
  PanicWriteStr(" cpu=");
  if (cpu_id >= 0) {
    PanicWriteDec(static_cast<uint64_t>(cpu_id));
  } else {
    PanicWriteStr("unbound");
  }
  PanicWriteStr("\n");

#if __STDC_HOSTED__
  std::fprintf(stderr,
               "[FATAL] Unhandled CPU exception %llu (%s) err=0x%llx "
               "rip=0x%llx rsp=0x%llx rflags=0x%llx cr2=0x%llx cpu=%d\n",
               static_cast<unsigned long long>(frame->vector),  //
               mnemonic,                                        //
               static_cast<unsigned long long>(frame->error_code),
               static_cast<unsigned long long>(frame->rip),     //
               static_cast<unsigned long long>(frame->rsp),     //
               static_cast<unsigned long long>(frame->rflags),  //
               static_cast<unsigned long long>(cr2),            //
               cpu_id);
  std::fflush(stderr);
  std::_Exit(134);
#else
  for (;;) {
    asm volatile("cli; hlt");
  }
#endif
}

}  // namespace

IdtGateDescriptor IdtEncodeGate(const uint64_t handler_addr,  //
                                const uint16_t selector,      //
                                const uint8_t ist,            //
                                const uint8_t type_attr) {
  DCHECK(handler_addr != 0);
  DCHECK(selector != 0);
  DCHECK(ist <= kMaxIstIndex);
  DCHECK(IsValidGateTypeAttr(type_attr));

  IdtGateDescriptor desc = {};
  desc.offset_low = static_cast<uint16_t>(handler_addr & 0xFFFFu);
  desc.selector = selector;
  desc.ist = static_cast<uint8_t>(ist & 0x07u);
  desc.type_attr = type_attr;
  desc.offset_mid = static_cast<uint16_t>((handler_addr >> 16) & 0xFFFFu);
  desc.offset_high = static_cast<uint32_t>((handler_addr >> 32) & 0xFFFFFFFFu);
  desc.reserved = 0;
  return desc;
}

uint64_t IdtDecodeGateOffset(const IdtGateDescriptor& desc) {
  return static_cast<uint64_t>(desc.offset_low) |
         (static_cast<uint64_t>(desc.offset_mid) << 16) |
         (static_cast<uint64_t>(desc.offset_high) << 32);
}

bool IdtVectorPushesErrorCode(const int vector) {
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  switch (vector) {
    case 8:
    case 10:
    case 11:
    case 12:
    case 13:
    case 14:
    case 17:
    case 21:
    case 29:
    case 30:
      return true;
    default:
      return false;
  }
}

const char* IdtExceptionMnemonic(const int vector) {
  DCHECK(vector >= 0 && vector < kCpuExceptionCount);
  static constexpr const char* kMnemonics[kCpuExceptionCount] = {
      "#DE",  "#DB",  "#NMI", "#BP",  "#OF", "#BR", "#UD",  "#NM",
      "#DF",  "#CSO", "#TS",  "#NP",  "#SS", "#GP", "#PF",  "#RES",
      "#MF",  "#AC",  "#MC",  "#XM",  "#VE", "#CP", "#RES", "#RES",
      "#RES", "#RES", "#RES", "#RES", "#HV", "#VC", "#SX",  "#RES",
  };
  return kMnemonics[vector];
}

void IdtInit() {
  for (int v = 0; v < kIdtEntryCount; ++v) {
#if __STDC_HOSTED__
    g_isr_stub_table[v] =
        0xFFFFFFFF80100000ULL + static_cast<uint64_t>(v) * 16ULL;
#endif
    g_handlers[v].store(nullptr, std::memory_order_relaxed);
    g_idt[v] = IdtEncodeGate(g_isr_stub_table[v],     //
                             kIdtKernelCodeSelector,  //
                             0,                       //
                             kIdtInterruptGateAttr);
  }
  g_idt_pointer.limit = static_cast<uint16_t>(sizeof(g_idt) - 1);
  g_idt_pointer.base = reinterpret_cast<uint64_t>(&g_idt[0]);
  g_idt_initialized.store(true, std::memory_order_release);
  IdtLoad();
}

void IdtLoad() {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
#if !__STDC_HOSTED__
  asm volatile("lidt [%0]" : : "r"(&g_idt_pointer) : "memory");
#else
  g_host_idt_load_count.fetch_add(1, std::memory_order_relaxed);
#endif
}

bool IdtIsInitialized() {
  return g_idt_initialized.load(std::memory_order_acquire);
}

const IdtGateDescriptor* IdtGetGate(const int vector) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  return &g_idt[vector];
}

IdtPointer IdtGetPointer() {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  return g_idt_pointer;
}

void IdtRegisterHandler(const int vector, const InterruptHandlerFn handler) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  DCHECK(handler != nullptr);
  g_handlers[vector].store(handler, std::memory_order_release);
}

void IdtUnregisterHandler(const int vector) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  g_handlers[vector].store(nullptr, std::memory_order_release);
}

InterruptHandlerFn IdtGetHandler(const int vector) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(vector >= 0 && vector < kIdtEntryCount);
  return g_handlers[vector].load(std::memory_order_acquire);
}

extern "C" void IdtDispatch(InterruptFrame* const frame) {
  DCHECK(g_idt_initialized.load(std::memory_order_acquire));
  DCHECK(frame != nullptr);
  DCHECK(frame->vector < static_cast<uint64_t>(kIdtEntryCount));

  const int vector = static_cast<int>(frame->vector);
  const InterruptHandlerFn handler =
      g_handlers[vector].load(std::memory_order_acquire);

  if (vector == kVectorNmi) {
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector < kCpuExceptionCount) {
    if (handler != nullptr) {
      handler(frame);
      return;
    }
    PanicUnhandledException(frame);
  }

  if (vector == static_cast<int>(kVectorApicTimer)) {
    CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu != nullptr) {
      cpu->timer_ticks.fetch_add(1, std::memory_order_relaxed);
    }
    SmpSendLocalApicEoi();
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector == static_cast<int>(kVectorWakeupIpi)) {
    CpuLocal* const cpu = CurrentCpuOrNull();
    if (cpu != nullptr) {
      cpu->ipi_count.fetch_add(1, std::memory_order_relaxed);
    }
    SmpSendLocalApicEoi();
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  if (vector == static_cast<int>(kVectorSpurious)) {
    if (handler != nullptr) {
      handler(frame);
    }
    return;
  }

  SmpSendLocalApicEoi();
  if (handler != nullptr) {
    handler(frame);
  }
}

#if __STDC_HOSTED__
void IdtResetForTest() {
  for (int v = 0; v < kIdtEntryCount; ++v) {
    g_idt[v] = {};
    g_handlers[v].store(nullptr, std::memory_order_relaxed);
  }
  g_idt_pointer = {};
  g_idt_initialized.store(false, std::memory_order_release);
  g_host_idt_load_count.store(0, std::memory_order_relaxed);
  g_host_cr2.store(0, std::memory_order_relaxed);
}

int IdtGetHostLoadCountForTest() {
  return g_host_idt_load_count.load(std::memory_order_relaxed);
}

void IdtSetHostCr2ForTest(const uint64_t cr2) {
  g_host_cr2.store(cr2, std::memory_order_relaxed);
}
#endif

}  // namespace protos
