// Cortex-M3 startup: vector table and reset handler. No C library is linked,
// so this file does everything crt0 would normally do.

#include <stdint.h>

#include "semihosting.hpp"

extern "C" {
// Symbols defined by linker.ld
extern uint32_t _estack;
extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();

void Reset_Handler();
void Fault_Handler();
}

int main();

namespace {

using Handler = void (*)();

struct VectorTable {
    void* initialStackPointer;
    Handler exceptions[15];
};

}  // namespace

// Placed at address 0 by the linker: the CPU loads SP from word 0 and jumps
// to the reset handler in word 1.
__attribute__((section(".isr_vector"), used)) extern const VectorTable g_vectorTable = {
    &_estack,
    {
        Reset_Handler,  // Reset
        Fault_Handler,  // NMI
        Fault_Handler,  // HardFault
        Fault_Handler,  // MemManage
        Fault_Handler,  // BusFault
        Fault_Handler,  // UsageFault
        nullptr, nullptr, nullptr, nullptr,  // Reserved
        Fault_Handler,  // SVCall
        Fault_Handler,  // DebugMonitor
        nullptr,        // Reserved
        Fault_Handler,  // PendSV
        Fault_Handler,  // SysTick
    },
};

extern "C" void Reset_Handler() {
    // Copy initialised data from flash to RAM, zero .bss.
    //
    // Deviation (cppcheck comparePointers): _sdata/_edata and _sbss/_ebss are
    // linker-script symbols marking the start and end of the same memory
    // region, so comparing their addresses is well-defined on this target.
    const uint32_t* src = &_sidata;
    // cppcheck-suppress comparePointers
    for (uint32_t* dst = &_sdata; dst < &_edata; ++dst, ++src) {
        *dst = *src;
    }
    // cppcheck-suppress comparePointers
    for (uint32_t* dst = &_sbss; dst < &_ebss; ++dst) {
        *dst = 0U;
    }
    // Run static constructors.
    for (void (**ctor)() = __init_array_start; ctor < __init_array_end; ++ctor) {
        (*ctor)();
    }

    const int status = main();
    semihost::exit(status == 0);
}

extern "C" void Fault_Handler() {
    semihost::write("FATAL: CPU fault exception\n");
    semihost::exit(false);
}
