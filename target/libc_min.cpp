// The compiler may emit calls to these four functions for struct copies and
// zero-initialisation even in freestanding code. With -nostdlib we provide
// minimal versions. This file is compiled with
// -fno-tree-loop-distribute-patterns so GCC does not turn the loops below back
// into calls to memset/memcpy (infinite recursion).

#include <stddef.h>
#include <stdint.h>

extern "C" {

void* memcpy(void* dest, const void* src, size_t count) {
    auto* d = static_cast<uint8_t*>(dest);
    const auto* s = static_cast<const uint8_t*>(src);
    for (size_t i = 0; i < count; ++i) {
        d[i] = s[i];
    }
    return dest;
}

void* memmove(void* dest, const void* src, size_t count) {
    auto* d = static_cast<uint8_t*>(dest);
    const auto* s = static_cast<const uint8_t*>(src);
    if (d < s) {
        for (size_t i = 0; i < count; ++i) {
            d[i] = s[i];
        }
    } else {
        for (size_t i = count; i > 0; --i) {
            d[i - 1] = s[i - 1];
        }
    }
    return dest;
}

void* memset(void* dest, int value, size_t count) {
    auto* d = static_cast<uint8_t*>(dest);
    for (size_t i = 0; i < count; ++i) {
        d[i] = static_cast<uint8_t>(value);
    }
    return dest;
}

int memcmp(const void* lhs, const void* rhs, size_t count) {
    const auto* a = static_cast<const uint8_t*>(lhs);
    const auto* b = static_cast<const uint8_t*>(rhs);
    for (size_t i = 0; i < count; ++i) {
        if (a[i] != b[i]) {
            return a[i] < b[i] ? -1 : 1;
        }
    }
    return 0;
}

}  // extern "C"
