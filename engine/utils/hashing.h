#pragma once

#include <cstddef>
#include <cstdint>

uint32_t murmur3_32(const uint8_t* key, size_t len, uint32_t seed);

inline size_t stl_container_hash(const uint8_t* key, size_t len)
{
    return murmur3_32(key, len, 0);
}
