#include <string>
#include <cstddef>
#include <vector>

#include "utils/memory_system.h"

struct SomeStruct {
    size_t member1;
    int member2;
    int member3;
    int member4;
    std::string name;
    char c;
};

const char* const g_names[] = {
    "alpha", "omega", "beta", "gamma" };


void test_fixed_block_allocator()
{
    unsigned char buf[1024 * 10];
    FixedBlockAllocator<SomeStruct> allocator(buf, sizeof(buf));
    const size_t num_el = 10000;
    std::vector<SomeStruct*> bookkeping;
    bookkeping.resize(num_el);

    for(size_t i=0; i < num_el; ++i)
        bookkeping[i] = nullptr;

    for(size_t i=0; i < num_el; ++i)
    {
        const size_t index = rand() % bookkeping.size();

        if(rand()%2)
        {
            SomeStruct* s =  new (allocator.Allocate()) SomeStruct;
            if(nullptr == s)
                continue;
            s->c = (char)(rand()%256);
            s->member2 = rand()%10 + 100;
            s->member3 = rand()%10 + 1000;
            s->member4 = rand()%10 + 10000;
            s->name = g_names[rand()%4];
            s->member1 = index;
            SomeStruct* old_s = bookkeping[index];
            if(old_s)
            {
                old_s->~SomeStruct();
                allocator.Deallocate(old_s);
            }
            bookkeping[index] = s;

        }
        else
        {
            size_t index = rand() % bookkeping.size();
            SomeStruct* s = bookkeping[index];
            if(nullptr == s)
                continue;
            assert(allocator.IsOwned(s));
            assert(s->member1 == index);
            allocator.Deallocate(s);
            bookkeping[index] = nullptr;

        }
    }

    for(size_t i=0; i < num_el; ++i)
    {
        SomeStruct* s = bookkeping[i];
        if(s)
        {
            s->~SomeStruct();
            allocator.Deallocate(s);
            bookkeping[i] = nullptr;
        }
    }
}
