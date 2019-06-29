#include <sys/types.h>
#include <cassert>

bool is_aligned(void* ptr, size_t alignment)
{
    return (((size_t)ptr) & ~(alignment - 1)) == 0;
}

size_t adjust_size_top(size_t size, size_t alignment)
{
    return (size + alignment - 1) & (~(alignment - 1));
}

void* align_top(void* ptr, size_t alignment)
{
    return (void*)(((size_t)ptr + alignment - 1) & (~(alignment - 1)));
}

template <typename T>
class FixedBlockAllocator {

    struct Block {
        Block* next_;
    };

    Block* free_list_;
    void* buffer_;
    size_t buffer_size_;

    enum {
        kBlockSize = sizeof(T),
        kAlignment = alignof(T),
    };
    public:

    FixedBlockAllocator(void* buffer, size_t size);

    void* Allocate();
    void Deallocate(void* ptr);

    bool IsOwned(void* ptr);
};

template<typename T>
FixedBlockAllocator<T>::FixedBlockAllocator(void* buffer, size_t size)
{
    static_assert(kAlignment >= 8, "Alignment should be minimum 8 bytes");
    assert(buffer);

    buffer_ = buffer;
    buffer_size_ = size;


    void* aligned_ptr = align_top(buffer_, kAlignment);
    T* objects = (T*)aligned_ptr;
    Block* cb = (Block*)aligned_ptr;
    free_list_ = cb;
    free_list_->next_ = nullptr;

    // initialize free list
    while((unsigned char*)(objects + 1) < (unsigned char*)buffer_ + buffer_size_)
    {
        Block* next = (Block*)objects++;
        cb->next_ = next;
        cb = next;
        cb->next_ = nullptr;
    }
}

template<typename T>
void* FixedBlockAllocator<T>::Allocate()
{
    if(!free_list_)
        return nullptr;

    void* ptr = free_list_;
    free_list_ = free_list_->next_;
    return ptr;
}

template<typename T>
void FixedBlockAllocator<T>::Deallocate(void* ptr)
{
    assert(ptr);
    assert(IsOwned(ptr));

    Block* next = free_list_;
    free_list_ = (Block*)ptr; 
    free_list_->next_ = next;
}

template<typename T>
bool FixedBlockAllocator<T>::IsOwned(void* ptr)
{
    return ptr >= buffer_ && ptr < (unsigned char*)buffer_ + buffer_size_ - adjust_size_top(kBlockSize, kAlignment);
}








