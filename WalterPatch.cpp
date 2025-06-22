#include <cstdio>

extern "C" int _intToStrDigit(int val, int digits)
{
    // Safe fallback patch that prevents linker error.
    return snprintf(nullptr, 0, "%0*d", digits, val);
}
