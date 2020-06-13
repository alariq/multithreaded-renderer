#include <string>
#include <cstddef>
#include <vector>
#include <list>
#include <cassert>
#include <unordered_map>

#include "utils/memory_system.h"
#include "utils/threading.h"
#include "utils/timing.h"

struct SomeStruct {
  private:
    static uint32_t s_allocator_obj_count_;
    static FixedBlockAllocator<SomeStruct> *s_allocator_;
    // static threading::CriticalSection *_PoolCS;
    static uint32_t s_allocated;
    static void *s_allcator_backing_mem;

  public:
    uint32_t member1;
    int member2;
    int member3;
    int member4;
    std::string name;
    char c;

    static FixedBlockAllocator<SomeStruct>* GetAllocator() {
        return s_allocator_;
    }

    static bool StaticInit(uint32_t obj_count) {
        assert(/*!_PoolCS && */ !s_allocator_);
        //_PoolCS = threading::CreateCriticalSection();
        s_allocator_obj_count_ = obj_count;
        const size_t buffer_size =
            s_allocator_obj_count_ * sizeof(SomeStruct);
        s_allcator_backing_mem = malloc(buffer_size);
        s_allocator_ = new FixedBlockAllocator<SomeStruct>(
            s_allcator_backing_mem, buffer_size);
        return /*_PoolCS &&*/ s_allocator_ != nullptr;
    }

    static void StaticDeinit() {
        delete s_allocator_;
        s_allocator_ = nullptr;
        free(s_allcator_backing_mem);
        s_allcator_backing_mem = nullptr;
        // threading::DestroyCriticalSection(_PoolCS);
        //_PoolCS = nullptr;
    }

    static void *operator new(std::size_t sz) throw() {
        assert(sz == sizeof(SomeStruct));

        // threading::ScopeLock sl(_PoolCS);
        void *ptr = s_allocator_->Allocate();
        //if (!ptr) {
        //    printf("Ran out of object pool\n");
        //}
        return ptr;
    }

    static void *operator new[](std::size_t sz) throw() {
        (void)sz;
        assert(0);
        return nullptr;
    }

    static void operator delete(void *ptr) {
        // threading::ScopeLock sl(_PoolCS);
        assert(s_allocator_->IsOwned(ptr));
        s_allocator_->Deallocate(ptr);
    }
};
uint32_t SomeStruct::s_allocator_obj_count_ = 0;
FixedBlockAllocator<SomeStruct> *SomeStruct::s_allocator_;
uint32_t SomeStruct::s_allocated = 0;
void *SomeStruct::s_allcator_backing_mem = nullptr;

const char *const g_names[] = {"alpha", "omega", "beta", "gamma"};

volatile bool exit_mt_test = false;
void mt_test(int num_threads) {

    struct worker_A {
        typedef worker_A worker;
        std::vector<SomeStruct *> objlist_;

        static int run(void *data) {
            // timing::sleep(10e+9);
            //
            worker *me = (worker *)data;
            assert(me);
            while (!exit_mt_test) {

                if (0 == (rand() % 10)) {
                    auto *o = new SomeStruct();
                    if(!o) continue;
                    o->member1 = (uint32_t)me->objlist_.size();
                    me->objlist_.push_back(o);
                    //printf("+");
                } else {
                    if(me->objlist_.empty())
                        continue;
                    bool success = false;
                    int tries = 10;
                    while (!success && tries>0) {
                        const int idx = rand() % (int)me->objlist_.size();
                        SomeStruct *o = me->objlist_[idx];
                        if (!o) {
                            tries--;
                            continue;
                        }
                        assert(idx == (int)o->member1);
                        delete o;
                        me->objlist_[idx] = nullptr;
                        success = true;
                        //printf("-");
                    }
                }
            }
            for(auto* o: me->objlist_) {
                delete o;
            };
            delete me;
            printf("done\n");
            return 0;
        };
    };

    struct worker_B {
        typedef worker_B worker;
        std::unordered_map<uint32_t, SomeStruct *> objlist_;
        std::vector<uint32_t> keylist_;

        static int run(void *data) {
            // timing::sleep(10e+9);
            //
            worker *me = (worker *)data;
            assert(me);
            while (!exit_mt_test) {

                if (rand() % 2) {
                    auto *o = new SomeStruct();
                    if(!o) continue;
                    o->member1 = rand();
                    while(me->objlist_.count(o->member1)) {
                        o->member1 = rand();
                    }
                    me->objlist_.insert(std::make_pair(o->member1, o));
                    me->keylist_.push_back(o->member1);
                } else {
                    if(me->keylist_.empty())
                        continue;
                    uint32_t idx = rand()%(int)me->keylist_.size();
                    SomeStruct *o = me->objlist_[me->keylist_[idx]];
                    assert(o);
                    assert(me->keylist_[idx] == o->member1);
                    me->objlist_.erase(o->member1);
                    me->keylist_.erase(me->keylist_.begin() + idx);
                    delete o;
                }
            }
            for(auto o: me->objlist_) {
                delete o.second;        
            };
            me->keylist_.clear();
            me->objlist_.clear();
            delete me;
            printf("done\n");
            return 0;
        };
    };


    std::vector<threading::Thread*> threads(num_threads);
    for(int i=0;i<(int)threads.size();++i) {
        char name[16];
        sprintf(name, "worker%d\n", i);
        if(i<num_threads/2)
            threads[i] = new threading::Thread(worker_A::run, name, new worker_A());
        else
            threads[i] = new threading::Thread(worker_B::run, name, new worker_B());
    }

    timing::sleep((uint64_t)10e+9);
    exit_mt_test = true;

    for(auto* thread: threads) {
        thread->Wait();
    };
}

void test_fixed_block_allocator()
{
    uint32_t num_el = 50000;
    SomeStruct::StaticInit(num_el);

    mt_test(8);

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
            SomeStruct* s =  new SomeStruct;
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
                delete old_s;
            bookkeping[index] = s;
            printf("Create\n");

        }
        else
        {
            size_t index = rand() % bookkeping.size();
            SomeStruct* s = bookkeping[index];
            if(nullptr == s)
                continue;
            assert(s->member1 == index);
            delete s;
            bookkeping[index] = nullptr;
            printf("Destroy\n");
        }
    }

    for(int i=0; i<(int)bookkeping.size();++i) {
        const auto obj = bookkeping[i];
        if(!obj) continue;

        assert(i == (int)obj->member1);
        printf("%d == %d, name: %s\n", i, (int)obj->member1,
               obj->name.c_str());
    }

#endif
#if 0
    std::vector<SomeStruct*> objlist;
    constexpr size_t c = sizeof(buf)/sizeof(SomeStruct);
    for(size_t i=0; i < c; ++i)
    {
        //if((rand()%3))
        {
            SomeStruct* s = new SomeStruct;
            if(!s)
                continue;
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
        {
            size_t index = rand() % objlist.size();
            SomeStruct* s = objlist[index];
            objlist.erase(objlist.begin()+index);
            delete s;
            printf("Destroy\n");
        }
    }
#endif

    char* pStatsString = nullptr;
    bool detailedMap = true;
    BuildStatsString(SomeStruct::GetAllocator(), &pStatsString, detailedMap);
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
            delete s;
            bookkeping[i] = nullptr;
        }
    }

    SomeStruct::StaticDeinit();
}
