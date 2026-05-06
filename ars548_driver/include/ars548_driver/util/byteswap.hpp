#pragma once

/**
 * @brief Optimized byte-swap using std::byteswap (C++23) and compiler intrinsics.
 */
#include <cstdint>
#include <cstring>
#include <bit>

// uint8_t — no-op (single byte needs no swapping)
inline uint8_t byteswap(uint8_t v) { return v; }

// Integer types — std::byteswap compiles to a single BSWAP instruction
inline uint16_t byteswap(uint16_t v) { return std::byteswap(v); }
inline uint32_t byteswap(uint32_t v) { return std::byteswap(v); }
inline uint64_t byteswap(uint64_t v) { return std::byteswap(v); }

// Float — reinterpret as uint32_t, swap, reinterpret back (safe via memcpy)
inline float byteswap(float v) {
    uint32_t tmp;
    std::memcpy(&tmp, &v, sizeof(tmp));
    tmp = std::byteswap(tmp);
    std::memcpy(&v, &tmp, sizeof(v));
    return v;
}

// Double — same approach via uint64_t
inline double byteswap(double v) {
    uint64_t tmp;
    std::memcpy(&tmp, &v, sizeof(tmp));
    tmp = std::byteswap(tmp);
    std::memcpy(&v, &tmp, sizeof(v));
    return v;
}