#include <string>
#include <cstddef>
#include <vector>
#include <list>

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
    const size_t num_el = 150;//10000;
    std::vector<SomeStruct*> bookkeping;
    bookkeping.resize(num_el);

    for(size_t i=0; i < num_el; ++i)
        bookkeping[i] = nullptr;
#if 0
    for(size_t i=0; i < num_el; ++i)
    {
        const size_t index = rand() % bookkeping.size();

        if(rand()%2)
        {

            auto ptr = allocator.Allocate();
            if(nullptr == ptr)
                continue;
            SomeStruct* s =  new (ptr) SomeStruct;
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
            printf("Create\n");

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

            printf("Destroy\n");

        }
    }
#endif
#if 1
    std::vector<SomeStruct*> objlist;
    constexpr size_t c = sizeof(buf)/sizeof(SomeStruct);
    for(size_t i=0; i < c; ++i)
    {
        //if((rand()%3))
        {
            auto ptr = allocator.Allocate();
            if(nullptr == ptr)
                continue;
            SomeStruct* s =  new (ptr) SomeStruct;
            s->c = (char)(rand()%256);
            s->member2 = rand()%10 + 100;
            s->member3 = rand()%10 + 1000;
            s->member4 = rand()%10 + 10000;
            s->name = g_names[rand()%4];
            objlist.push_back(s);
            printf("Create\n");
        }
    }

    for(size_t i=0; i < c/2; ++i)
    {
        //else if(objlist.size()>0)
        {
            size_t index = rand() % objlist.size();
            SomeStruct* s = objlist[index];
            assert(allocator.IsOwned(s));
            allocator.Deallocate(s);
            objlist.erase(objlist.begin()+index);

            printf("Destroy\n");
        }
    }
#endif

    char* pStatsString = nullptr;
    bool detailedMap = true;
    BuildStatsString(&allocator, &pStatsString, detailedMap);
    FILE* f = fopen("mem_stats.json", "wb");
    if(f)
    {
        printf("Dumping memory stats...");
        size_t l = strlen(pStatsString);
        fwrite(pStatsString, l, 1, f);
        fclose(f);  
        printf("done.\n");
    }
    else
    {
        printf("Error opening file to dump memory stats\n");
    }
    delete[] pStatsString;


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
