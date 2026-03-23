#include <cassert>
#include <cstdint>
#include <cstdio>

#include "engine/utils/myarray.h"

struct BufferPair {
    uint32_t a;
    uint32_t b;
};

static BufferT<BufferPair, int> make_buffer(int count)
{
    BufferT<BufferPair, int> buf;
    for (int i = 0; i < count; ++i) {
        BufferPair p;
        p.a = static_cast<uint32_t>(i * 10 + 1);
        p.b = static_cast<uint32_t>(0xABCD0000u + i);
        buf.push(p);
    }
    return buf;
}

static void assert_equal(const BufferT<BufferPair, int>& a, const BufferT<BufferPair, int>& b)
{
    assert(a.size() == b.size());
    for (int i = 0; i < a.size(); ++i) {
        assert(a[i].a == b[i].a);
        assert(a[i].b == b[i].b);
    }
}

static void test_buffer_copy_constructor()
{
    BufferT<BufferPair, int> empty;
    BufferT<BufferPair, int> empty_copy(empty);
    assert(empty_copy.size() == 0);

    BufferT<BufferPair, int> src = make_buffer(8);
    BufferT<BufferPair, int> copy(src);

    assert(copy.size() == src.size());
    assert(copy.data() != src.data());
    assert_equal(copy, src);

    src[0].a = 999u;
    assert(copy[0].a != src[0].a);
}

static void test_buffer_copy_assignment()
{
    BufferT<BufferPair, int> src = make_buffer(6);
    BufferT<BufferPair, int> dst = make_buffer(3);

    dst = src;
    assert(dst.size() == src.size());
    assert(dst.data() != src.data());
    assert_equal(dst, src);

    src[1].b = 123456u;
    assert(dst[1].b != src[1].b);
}

void test_myarray()
{
    test_buffer_copy_constructor();
    test_buffer_copy_assignment();
}
