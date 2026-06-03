#pragma once
#include "my_types.h"
#include <stdlib.h> // aligned_alloc, maybe just use MS and posix variants to avoid dependencies (and C++17 requirement)
#include <assert.h> 
#include <string.h>  // memcpy
#include <atomic>

template <typename T, int Align = alignof(T) >
class RingBufferT
{
    u32 Capacity;
    u32 Mask;

	T* data_ = nullptr;
    T* begin_ = nullptr;
    T* end_ = nullptr;
    mutable uint64_t read_idx = 0;
    uint64_t write_idx = 0;

    RingBufferT<T>& operator=(const RingBufferT<T>& rb) = delete;
    RingBufferT(const RingBufferT<T>&) = delete;

    static int min(int a, int b) { return a < b ? a : b; }

    public:


    explicit RingBufferT(u32 CapPow):
        Capacity(1<<CapPow), Mask(Capacity-1),
        data_(nullptr), begin_(nullptr), end_(nullptr),
        read_idx(0), write_idx(0) {}

    bool reset() {

        if(!data_) {
            data_ = (T*)aligned_alloc(Align > sizeof(T) ? Align : sizeof(T), sizeof(T)*Capacity);
        }

        begin_ = end_ = data_;
        read_idx = write_idx = 0;

        return data_ != nullptr;
    }

    ~RingBufferT() {
        free(data_);
    }

    int get_size() const { return write_idx - read_idx; }
    int get_free() const { return Capacity - get_size(); }

    bool push_batch(const u8* arr, const u32 bytes) {
        const u32 count = bytes / sizeof(T);
        const T* src = (const T*)arr;

        assert(data_);
        assert(bytes % sizeof(T) == 0);
        assert(count <= (u32)get_free());

        if(!data_ || bytes % sizeof(T) != 0 || count > (u32)get_free())
            return false;

        u32 head_count = min((u32)(Capacity - (write_idx & Mask)), count);
        memcpy(&data_[write_idx & Mask], src, sizeof(T)*head_count);
        if(count > head_count) {
            //printf("wrapped\n");
            memcpy(&data_[0], src + head_count, sizeof(T)*(count - head_count));
        }

        write_idx += count;
        dbg_print();
        return true;
    }

	void push(T p) {
        assert(data_);
        assert(write_idx - read_idx < Capacity);
        data_[write_idx++ & Mask] = p;
        dbg_print();
	}

    T pop() {
        assert(read_idx < write_idx);
        T v = data_[read_idx++ & Mask];
        dbg_print();
        return v;
    }

    T peek(int i) {
        assert(i>=0 && read_idx + i < write_idx);
        T v = data_[(read_idx + i) & Mask];
        dbg_print();
        return v;
    }

    void advance(u32 i) {
        assert(read_idx + i <= write_idx);
        read_idx += i;
    }

    void dbg_print() const;
};

// TODO: use https://github.com/cameron314/readerwriterqueue / concurrentqueue
template<typename T, u32 Capacity >
struct SPSCRingBufferT {
    // pretending I care about performance
    //can use compiler param instead: --param hardware_destructive_interference_size=64
    //and then: alignas(std::hardware_destructive_interference_size)
    alignas(64) std::atomic_uint ri = 0;
    alignas(64) std::atomic_uint wi = 0;

    // one extra element to distinguish between full and empty
    static constexpr unsigned int RealCapacity = Capacity + 1;
    T data_[RealCapacity];

    bool push(const T& v) {
        unsigned int widx = wi.load(std::memory_order_relaxed);
        unsigned int ridx = ri.load(std::memory_order_acquire);
        if((widx + 1) % RealCapacity == ridx) return false;

        data_[widx] = v;
        wi.store((wi + 1) % RealCapacity, std::memory_order_release);
        //wi.fetch_add(1, std::memory_order_release);
        return true;
    }

    bool pop(T& v) {
        unsigned int ridx = ri.load(std::memory_order_relaxed);
        unsigned int widx = wi.load(std::memory_order_acquire);
        if(ridx == widx) { return false; }

        v = data_[ri % RealCapacity];
        ri.store((ri + 1) % RealCapacity, std::memory_order_release);
        //ri.fetch_add(1, std::memory_order_release);
        return true;
    }
};

