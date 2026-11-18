#include "semihosting.hpp"

#include <stdint.h>

namespace semihost {

namespace {

constexpr uint32_t kSysWrite0 = 0x04U;
constexpr uint32_t kSysExit = 0x18U;
constexpr uint32_t kAdpStoppedApplicationExit = 0x20026U;  // exit code 0
constexpr uint32_t kAdpStoppedRunTimeError = 0x20023U;     // exit code 1

// The semihosting ABI passes the operation in r0 and the argument in r1, and
// returns the result in r0.
uint32_t call(uint32_t operation, uint32_t argument) {
    uint32_t result;
    asm volatile(
        "mov r0, %[op]\n"
        "mov r1, %[arg]\n"
        "bkpt 0xAB\n"
        "mov %[res], r0\n"
        : [res] "=r"(result)
        : [op] "r"(operation), [arg] "r"(argument)
        : "r0", "r1", "memory");
    return result;
}

}  // namespace

void write(const char* text) {
    call(kSysWrite0, reinterpret_cast<uint32_t>(text));
}

void exit(bool success) {
    call(kSysExit, success ? kAdpStoppedApplicationExit : kAdpStoppedRunTimeError);
    for (;;) {
    }
}

}  // namespace semihost
