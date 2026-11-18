#pragma once

// ARM semihosting: the program asks the debugger (here: QEMU) to perform I/O
// on its behalf with a BKPT 0xAB instruction. This replaces a UART driver for
// the emulated board and also lets the firmware report an exit code.

namespace semihost {

void write(const char* text);

[[noreturn]] void exit(bool success);

}  // namespace semihost
