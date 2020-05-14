#pragma once

#define ENABLE_STATS

#ifdef ENABLE_STATS
#include "utils/json_writer.h"
#endif

#include <sys/types.h>
#include <cassert>
#include <algorithm> // min/max

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

namespace alloc
{
    enum {kMAX_MEMORY_HEAPS = 16, kMAX_MEMORY_TYPES = 16};

    // this is, strictly speaking, should not belong to statistic as this is just 
    // a registry of all heaps and their info
    typedef struct MemHeap {
        size_t size;
        uint32_t flags;
        size_t blockCount;
    } MemHeap;

    typedef struct MemProps {
        MemHeap memoryHeaps[kMAX_MEMORY_HEAPS];
    } MemProps;
    //
    
    struct Allocation {
        int m_SuballocationType; // index in SUBALLOCATION_TYPE_NAMES
        uint32_t m_CreationFrameIndex;
        uint32_t m_BufferImageUsage;
        size_t m_Size;

        uint32_t GetLastUseFrameIndex() const { return 0; }

        void PrintParameters(class JsonWriter& json) const;

        Allocation():
             m_SuballocationType(0)
            ,m_CreationFrameIndex(0)
            ,m_BufferImageUsage(0)
            ,m_Size(0)
        {} 
    };
   
    typedef struct StatInfo
    {
        /// Number of `VkDeviceMemory` Vulkan memory blocks allocated.
        uint32_t blockCount;
        /// Number of #VmaAllocation allocation objects allocated.
        uint32_t allocationCount;
        /// Number of free ranges of memory between allocations.
        uint32_t unusedRangeCount;
        /// Total number of bytes occupied by all allocations.
        size_t usedBytes;
        /// Total number of bytes occupied by unused ranges.
        size_t unusedBytes;
        size_t allocationSizeMin, allocationSizeAvg, allocationSizeMax;
        size_t unusedRangeSizeMin, unusedRangeSizeAvg, unusedRangeSizeMax;
    } StatInfo;

    /// General statistics from current state of Allocator.
    typedef struct Stats
    {
        StatInfo memoryType[kMAX_MEMORY_TYPES];
        StatInfo memoryHeap[kMAX_MEMORY_HEAPS];
        StatInfo total;
    } Stats;

    // Division with mathematical rounding to nearest number.
    template <typename T>
        static inline T RoundDiv(T x, T y)
        {
            return (x + (y / (T)2)) / y;
        }


    static void InitStatInfo(alloc::StatInfo& outInfo)
    {
        memset(&outInfo, 0, sizeof(outInfo));
        outInfo.allocationSizeMin = UINT64_MAX;
        outInfo.unusedRangeSizeMin = UINT64_MAX;
    }

// Adds statistics srcInfo into inoutInfo, like: inoutInfo += srcInfo.
    static void AddStatInfo(alloc::StatInfo& inoutInfo, const alloc::StatInfo& srcInfo)
    {
        inoutInfo.blockCount += srcInfo.blockCount;
        inoutInfo.allocationCount += srcInfo.allocationCount;
        inoutInfo.unusedRangeCount += srcInfo.unusedRangeCount;
        inoutInfo.usedBytes += srcInfo.usedBytes;
        inoutInfo.unusedBytes += srcInfo.unusedBytes;
        inoutInfo.allocationSizeMin = std::min(inoutInfo.allocationSizeMin, srcInfo.allocationSizeMin);
        inoutInfo.allocationSizeMax = std::max(inoutInfo.allocationSizeMax, srcInfo.allocationSizeMax);
        inoutInfo.unusedRangeSizeMin = std::min(inoutInfo.unusedRangeSizeMin, srcInfo.unusedRangeSizeMin);
        inoutInfo.unusedRangeSizeMax = std::max(inoutInfo.unusedRangeSizeMax, srcInfo.unusedRangeSizeMax);
    }

    static void PostprocessCalcStatInfo(alloc::StatInfo& inoutInfo)
    {
        inoutInfo.allocationSizeAvg = (inoutInfo.allocationCount > 0) ?
            RoundDiv<size_t>(inoutInfo.usedBytes, inoutInfo.allocationCount) : 0;
        inoutInfo.unusedRangeSizeAvg = (inoutInfo.unusedRangeCount > 0) ?
            RoundDiv<size_t>(inoutInfo.unusedBytes, inoutInfo.unusedRangeCount) : 0;
    }

} // namespace alloc

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

    #if defined(ENABLE_STATS)
    size_t block_count_; 
    void PrintDetailedMap(JsonWriter& jw) const ;
    void CalculateStats(alloc::Stats* s) const ;
    void CalculateStatInfo(alloc::StatInfo& si) const;
    void BlockVector_PrintDetailedMap(class JsonWriter& json) const ;
    uint32_t GetMemoryTypeCount() const { return 1; }
    uint32_t GetMemoryHeapCount() const { return 1; }
    uint32_t MemoryTypeIndexToHeapIndex(uint32_t type) const { return type; }
    alloc::MemProps m_MemProps;

    void BlockMetadata_PrintDetailedMap_Begin(
        class JsonWriter& json,
        size_t unusedBytes,
        size_t allocationCount,
        size_t unusedRangeCount) const;

    void BlockMetadata_PrintDetailedMap_Allocation(
        class JsonWriter& json,
        size_t offset,
        alloc::Allocation* hAllocation) const;

    void BlockMetadata_PrintDetailedMap_UnusedRange(
        class JsonWriter& json,
        size_t offset,
        size_t size) const;

    void BlockMetadata_PrintDetailedMap_End(class JsonWriter& json) const;
    void BlockMetadata_PrintDetailedMap(class JsonWriter& json) const;
    #endif
};

template<typename T>
FixedBlockAllocator<T>::FixedBlockAllocator(void* buffer, size_t size)
{
    static_assert(kAlignment >= 8, "Alignment should be minimum 8 bytes");
    assert(buffer);

#ifdef ENABLE_STATS
    block_count_ = 0;
    m_MemProps.memoryHeaps[0].size = size;
    m_MemProps.memoryHeaps[0].flags = 0;
#endif

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
#ifdef ENABLE_STATS
    block_count_++;
#endif
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
#ifdef ENABLE_STATS
    block_count_--;
#endif
}

template<typename T>
bool FixedBlockAllocator<T>::IsOwned(void* ptr)
{
    return ptr >= buffer_ && ptr < (unsigned char*)buffer_ + buffer_size_ - adjust_size_top(kBlockSize, kAlignment);
}

#if defined(ENABLE_STATS)

template<typename T>
void::FixedBlockAllocator<T>::CalculateStats(alloc::Stats* pStats) const 
{
    // Initialize.
    alloc::InitStatInfo(pStats->total);
    for(size_t i = 0; i < alloc::kMAX_MEMORY_TYPES; ++i)
        alloc::InitStatInfo(pStats->memoryType[i]);
    for(size_t i = 0; i < alloc::kMAX_MEMORY_HEAPS; ++i)
        alloc::InitStatInfo(pStats->memoryHeap[i]);

    CalculateStatInfo(pStats->memoryType[0]);
    alloc::PostprocessCalcStatInfo(pStats->memoryType[0]);
    alloc::AddStatInfo(pStats->total, pStats->memoryType[0]);
}

template<typename T>
void FixedBlockAllocator<T>::CalculateStatInfo(alloc::StatInfo& si) const 
{
    si.blockCount = block_count_;
    si.allocationCount = block_count_;
    si.unusedRangeCount = 0; // ?

    si.usedBytes = sizeof(T) * block_count_;
    si.unusedBytes = buffer_size_ - sizeof(T) * block_count_;

    si.allocationSizeMin = sizeof(T);
    si.allocationSizeAvg = sizeof(T);
    si.allocationSizeMax = sizeof(T);

    si.unusedRangeSizeMin = si.unusedRangeSizeAvg = si.unusedRangeSizeMax = 0;
}
#endif


// statistic related functions

namespace alloc {

enum SuballocationType
{
    SUBALLOCATION_TYPE_FREE = 0,
    SUBALLOCATION_TYPE_UNKNOWN = 1,
    SUBALLOCATION_TYPE_BUFFER = 2,
    SUBALLOCATION_TYPE_IMAGE_UNKNOWN = 3,
    SUBALLOCATION_TYPE_IMAGE_LINEAR = 4,
    SUBALLOCATION_TYPE_IMAGE_OPTIMAL = 5,
    SUBALLOCATION_TYPE_MAX_ENUM = 0x7FFFFFFF
};

// Correspond to values of enum VmaSuballocationType.
static const char* SUBALLOCATION_TYPE_NAMES[] = {
    "FREE",
    "UNKNOWN",
    "BUFFER",
    "IMAGE_UNKNOWN",
    "IMAGE_LINEAR",
    "IMAGE_OPTIMAL",
};

enum { 
    POOL_CREATE_LINEAR_ALGORITHM_BIT = 0x01, 
    POOL_CREATE_BUDDY_ALGORITHM_BIT = 0x2,
    POOL_CREATE_FIXED_BLOCK_ALGORITHM_BIT = 0x4
};

static const char* AlgorithmToStr(uint32_t algorithm)
{
    switch(algorithm)
    {
    case POOL_CREATE_LINEAR_ALGORITHM_BIT:
        return "Linear";
    case POOL_CREATE_BUDDY_ALGORITHM_BIT:
        return "Buddy";
    case POOL_CREATE_FIXED_BLOCK_ALGORITHM_BIT:
        return "FixedBlock";
    case 0:
        return "Default";
    default:
        assert(0);
        return "";
    }
}

void Allocation::PrintParameters(class JsonWriter& json) const
{
    json.WriteString("Type");
    json.WriteString(SUBALLOCATION_TYPE_NAMES[m_SuballocationType]);

    json.WriteString("Size");
    json.WriteNumber(m_Size);
#if 0
    if(m_pUserData != VMA_NULL)
    {
        json.WriteString("UserData");
        if(IsUserDataString())
        {
            json.WriteString((const char*)m_pUserData);
        }
        else
        {
            json.BeginString();
            json.ContinueString_Pointer(m_pUserData);
            json.EndString();
        }
    }
#endif

    json.WriteString("CreationFrameIndex");
    json.WriteNumber(m_CreationFrameIndex);

    json.WriteString("LastUseFrameIndex");
    json.WriteNumber(GetLastUseFrameIndex());

    if(m_BufferImageUsage != 0)
    {
        json.WriteString("Usage");
        json.WriteNumber(m_BufferImageUsage);
    }
}

}; // namespace alloc


template<typename T>
void FixedBlockAllocator<T>::BlockMetadata_PrintDetailedMap_Begin(
        class JsonWriter& json,
        size_t unusedBytes,
        size_t allocationCount,
        size_t unusedRangeCount) const
{
    json.BeginObject();

    json.WriteString("TotalBytes");
    //json.WriteNumber(GetSize());
    json.WriteNumber(buffer_size_);

    json.WriteString("UnusedBytes");
    json.WriteNumber(unusedBytes);

    json.WriteString("Allocations");
    json.WriteNumber((uint64_t)allocationCount);

    json.WriteString("UnusedRanges");
    json.WriteNumber((uint64_t)unusedRangeCount);

    json.WriteString("Suballocations");
    json.BeginArray();
}

template<typename T>
void FixedBlockAllocator<T>::BlockMetadata_PrintDetailedMap_Allocation(
        class JsonWriter& json,
        size_t offset,
        alloc::Allocation* hAllocation) const
{
    json.BeginObject(true);
        
    json.WriteString("Offset");
    json.WriteNumber(offset);

    hAllocation->PrintParameters(json);

    json.EndObject();
}

template<typename T>
void FixedBlockAllocator<T>::BlockMetadata_PrintDetailedMap_UnusedRange(
        class JsonWriter& json,
        size_t offset,
        size_t size) const
{
    json.BeginObject(true);

    json.WriteString("Offset");
    json.WriteNumber(offset);

    json.WriteString("Type");
    //just for tests:
    //json.WriteString(alloc::SUBALLOCATION_TYPE_NAMES[alloc::SUBALLOCATION_TYPE_FREE]);
    //json.WriteString(alloc::SUBALLOCATION_TYPE_NAMES[rand()%5 + 1]);
    json.WriteString(alloc::SUBALLOCATION_TYPE_NAMES[2]);

    json.WriteString("Size");
    json.WriteNumber(size);

    json.EndObject();
}

template<typename T>
void FixedBlockAllocator<T>::BlockMetadata_PrintDetailedMap_End(
        class JsonWriter& json) const
{
    json.EndArray();
    json.EndObject();
}

template<typename T>
void FixedBlockAllocator<T>::BlockMetadata_PrintDetailedMap(class JsonWriter& json) const
{
    size_t total_block_count = buffer_size_/sizeof(T);

    BlockMetadata_PrintDetailedMap_Begin(json,
            buffer_size_ - block_count_*sizeof(T), // unusedBytes
            block_count_, // allocationCount
            total_block_count - block_count_); // unusedRangeCount

    Block* iter = free_list_;
    while(iter)
    {
        size_t offset = (char*)iter - (char*)buffer_;
        BlockMetadata_PrintDetailedMap_UnusedRange(json, offset, sizeof(T));
        iter = iter->next_;
    }
    // all others a not free but it is hard to print them out
    //BlockMetadata_PrintDetailedMap_Allocation(json, suballocItem->offset, suballocItem->hAllocation);

    BlockMetadata_PrintDetailedMap_End(json);
}

template<typename T>
void FixedBlockAllocator<T>::BlockVector_PrintDetailedMap(class JsonWriter& json) const
{
    //VmaMutexLockRead lock(m_Mutex, m_hAllocator->m_UseMutex);

    json.BeginObject();

    //if(m_IsCustomPool)
    if(1)
    {
        json.WriteString("MemoryTypeIndex");
        //json.WriteNumber(m_MemoryTypeIndex);
        json.WriteNumber(0u);

        json.WriteString("BlockSize");
        //json.WriteNumber(m_PreferredBlockSize);
        json.WriteNumber(sizeof(T));

        json.WriteString("BlockCount");
        json.BeginObject(true);
#if 0
        if(m_MinBlockCount > 0)
        {
            json.WriteString("Min");
            //json.WriteNumber((uint64_t)m_MinBlockCount);
            json.WriteNumber((uint64_t)0);
        }
        if(m_MaxBlockCount < SIZE_MAX)
        {
            json.WriteString("Max");
            json.WriteNumber((uint64_t)(buffer_size_/sizeof(T)));
        }
#endif
        json.WriteString("Cur");
        //json.WriteNumber((uint64_t)m_Blocks.size());
        json.WriteNumber((uint64_t)block_count_);
        json.EndObject();
#if 0
        if(m_FrameInUseCount > 0)
        {
            json.WriteString("FrameInUseCount");
            //json.WriteNumber(m_FrameInUseCount);
            json.WriteNumber(0);
        }

#endif
        //if(m_Algorithm != 0)
        {
            json.WriteString("Algorithm");
            json.WriteString(alloc::AlgorithmToStr(alloc::POOL_CREATE_FIXED_BLOCK_ALGORITHM_BIT));
        }
    }
    else
    {
        json.WriteString("PreferredBlockSize");
        //json.WriteNumber(m_PreferredBlockSize);
        json.WriteNumber(sizeof(T));
    }

    json.WriteString("Blocks");
    json.BeginObject();
    //for(size_t i = 0; i < m_Blocks.size(); ++i)
    size_t i=0;
    //for(size_t i = 0; i < block_count_; ++i)
    {
        json.BeginString();
        //json.ContinueString(m_Blocks[i]->GetId());
        json.ContinueString(i);
        json.EndString();

        //m_Blocks[i]->m_pMetadata->PrintDetailedMap(json);
        BlockMetadata_PrintDetailedMap(json);
    }
    json.EndObject();

    json.EndObject();
}

template<typename T>
void FixedBlockAllocator<T>::PrintDetailedMap(JsonWriter& json) const
{
#if 0
    bool dedicatedAllocationsStarted = false;
    for(uint32_t memTypeIndex = 0; memTypeIndex < GetMemoryTypeCount(); ++memTypeIndex)
    {
        VmaMutexLockRead dedicatedAllocationsLock(m_DedicatedAllocationsMutex[memTypeIndex], m_UseMutex);
        AllocationVectorType* const pDedicatedAllocVector = m_pDedicatedAllocations[memTypeIndex];
        VMA_ASSERT(pDedicatedAllocVector);
        if(pDedicatedAllocVector->empty() == false)
        {
            if(dedicatedAllocationsStarted == false)
            {
                dedicatedAllocationsStarted = true;
                json.WriteString("DedicatedAllocations");
                json.BeginObject();
            }

            json.BeginString("Type ");
            json.ContinueString(memTypeIndex);
            json.EndString();
                
            json.BeginArray();

            for(size_t i = 0; i < pDedicatedAllocVector->size(); ++i)
            {
                json.BeginObject(true);
                const VmaAllocation hAlloc = (*pDedicatedAllocVector)[i];
                hAlloc->PrintParameters(json);
                json.EndObject();
            }

            json.EndArray();
        }
    }
    if(dedicatedAllocationsStarted)
    {
        json.EndObject();
    }

    {
        bool allocationsStarted = false;
        for(uint32_t memTypeIndex = 0; memTypeIndex < GetMemoryTypeCount(); ++memTypeIndex)
        {
            if(m_pBlockVectors[memTypeIndex]->IsEmpty() == false)
            {
                if(allocationsStarted == false)
                {
                    allocationsStarted = true;
                    json.WriteString("DefaultPools");
                    json.BeginObject();
                }

                json.BeginString("Type ");
                json.ContinueString(memTypeIndex);
                json.EndString();

                m_pBlockVectors[memTypeIndex]->PrintDetailedMap(json);
            }
        }
        if(allocationsStarted)
        {
            json.EndObject();
        }
    }

#endif

#if 0
    // Custom pools
    {
        VmaMutexLockRead lock(m_PoolsMutex, m_UseMutex);
        const size_t poolCount = m_Pools.size();
        if(poolCount > 0)
        {
            json.WriteString("Pools");
            json.BeginObject();
            for(size_t poolIndex = 0; poolIndex < poolCount; ++poolIndex)
            {
                json.BeginString();
                json.ContinueString(m_Pools[poolIndex]->GetId());
                json.EndString();

                m_Pools[poolIndex]->m_BlockVector.PrintDetailedMap(json);
            }
            json.EndObject();
        }
    }

#endif

    json.WriteString("Pools");
    json.BeginObject();
    {
        json.BeginString();
        json.ContinueString(0u);
        json.EndString();

        BlockVector_PrintDetailedMap(json);
    }
    json.EndObject();
}



static void PrintStatInfo(JsonWriter& json, const alloc::StatInfo& stat)
{
    json.BeginObject();

    json.WriteString("Blocks");
    json.WriteNumber(stat.blockCount);

    json.WriteString("Allocations");
    json.WriteNumber(stat.allocationCount);

    json.WriteString("UnusedRanges");
    json.WriteNumber(stat.unusedRangeCount);

    json.WriteString("UsedBytes");
    json.WriteNumber(stat.usedBytes);

    json.WriteString("UnusedBytes");
    json.WriteNumber(stat.unusedBytes);

    if(stat.allocationCount > 1)
    {
        json.WriteString("AllocationSize");
        json.BeginObject(true);
        json.WriteString("Min");
        json.WriteNumber(stat.allocationSizeMin);
        json.WriteString("Avg");
        json.WriteNumber(stat.allocationSizeAvg);
        json.WriteString("Max");
        json.WriteNumber(stat.allocationSizeMax);
        json.EndObject();
    }

    if(stat.unusedRangeCount > 1)
    {
        json.WriteString("UnusedRangeSize");
        json.BeginObject(true);
        json.WriteString("Min");
        json.WriteNumber(stat.unusedRangeSizeMin);
        json.WriteString("Avg");
        json.WriteNumber(stat.unusedRangeSizeAvg);
        json.WriteString("Max");
        json.WriteNumber(stat.unusedRangeSizeMax);
        json.EndObject();
    }

    json.EndObject();
}

// Dump allocator info


template<typename T>
void BuildStatsString(const FixedBlockAllocator<T>* allocator, char** ppStatsString, bool detailedMap)
{
    assert(allocator && ppStatsString);

    StringBuilder sb;
    {
        JsonWriter json(sb);
        json.BeginObject();

        alloc::Stats stats;
        allocator->CalculateStats(&stats);

        json.WriteString("Total");
        PrintStatInfo(json, stats.total);
    
        for(uint32_t heapIndex = 0; heapIndex < allocator->GetMemoryHeapCount(); ++heapIndex)
        {
            json.BeginString("Heap ");
            json.ContinueString(heapIndex);
            json.EndString();
            json.BeginObject();

            json.WriteString("Size");
            json.WriteNumber(allocator->m_MemProps.memoryHeaps[heapIndex].size);

            json.WriteString("Flags");
            json.BeginArray(true);
            //if((allocator->m_MemProps.memoryHeaps[heapIndex].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0)
            //{
            //    json.WriteString("DEVICE_LOCAL");
            //}
            json.EndArray();

            if(stats.memoryHeap[heapIndex].blockCount > 0)
            {
                json.WriteString("Stats");
                PrintStatInfo(json, stats.memoryHeap[heapIndex]);
            }

            for(uint32_t typeIndex = 0; typeIndex < allocator->GetMemoryTypeCount(); ++typeIndex)
            {
                if(allocator->MemoryTypeIndexToHeapIndex(typeIndex) == heapIndex)
                {
                    json.BeginString("Type ");
                    json.ContinueString(typeIndex);
                    json.EndString();

                    json.BeginObject();

                    json.WriteString("Flags");
                    json.BeginArray(true);
#if 0
                    VkMemoryPropertyFlags flags = allocator->m_MemProps.memoryTypes[typeIndex].propertyFlags;
                    if((flags & VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT) != 0)
                    {
                        json.WriteString("DEVICE_LOCAL");
                    }
                    if((flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT) != 0)
                    {
                        json.WriteString("HOST_VISIBLE");
                    }
                    if((flags & VK_MEMORY_PROPERTY_HOST_COHERENT_BIT) != 0)
                    {
                        json.WriteString("HOST_COHERENT");
                    }
                    if((flags & VK_MEMORY_PROPERTY_HOST_CACHED_BIT) != 0)
                    {
                        json.WriteString("HOST_CACHED");
                    }
                    if((flags & VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT) != 0)
                    {
                        json.WriteString("LAZILY_ALLOCATED");
                    }
#endif
                    json.EndArray();
                    if(stats.memoryType[typeIndex].blockCount > 0)
                    {
                        json.WriteString("Stats");
                        PrintStatInfo(json, stats.memoryType[typeIndex]);
                    }

                    json.EndObject();
                }
            }

            json.EndObject();
        }
        if(detailedMap)
        {
            allocator->PrintDetailedMap(json);
        }

        json.EndObject();
    }

    const size_t len = sb.GetLength();
    char* const pChars = new char[len + 1];
    if(len > 0)
    {
        memcpy(pChars, sb.GetData(), len);
    }
    pChars[len] = '\0';
    *ppStatsString = pChars;
}





