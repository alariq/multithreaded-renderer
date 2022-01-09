#pragma once

#include <cassert>
#include <string.h>

// NOTE: another variant is to have a simple remapping array which will allow us to shuffle elements in a target array as soon as we update references:
// e.g. 
// v:|a|-|-|b|-|c|-|-|
// i:|0|5|3|-|-|-|-|-|
// after reorganizaton
// v:|a|c|b|-|-|-|-|-|
// i:|0|1|2|-|-|-|-|-|
// client will always have:0,1,2 indices


// NOTE: also may have a wrapper struct over T which will have header + footer to chek for overwriting
template<typename T> struct FreeListBoundsChecker {
    //unsigned int header; not so simple to do, because will have to offset into it as we store free list at the 0 offset
    T v;
    unsigned int footer;
    // also means we need to call constructors, which we do not want to do, so maybe just memset it
    FreeListBoundsChecker(T&& vv):/*header(0xcdcdcdcd),*/ v(move(vv)), footer(0xdddddddd){} 
    FreeListBoundsChecker() {}
};

// for POD data types (we do not call destructor here), this is in ToDo, maybe
template <typename T> class FreeList {
	T* data_ = nullptr; // make it non-copyable for now
	int size_ = 0;
	int num_free_ = 0;
	int free_ = -1;

	void resize(int new_size) {
		assert(new_size > 0);
		int old_size = size_;
        if(size_ >= new_size) // may need to have flag for exaxt resize 
            return;

        T* new_data = new T[new_size]; // this will call constructors, maybe just new u8[bytes_size];
        memcpy(new_data, data_, sizeof(T)*old_size);
        delete[] data_;
        data_ = new_data;
        size_ = new_size;

		for (int i = old_size; i < size_; ++i)
		    release(i);
	}

    bool is_acquired(int i) const {
        int f = free_;
        while(f!=-1) {
			if (f == i) 
                return false;

            f = *(int*)(data_ + f);
		}
        return true;
    }


  public:
    ~FreeList() {
        clear();
    }

    void clear() {
        delete[] data_;
        data_ = nullptr;
        size_ = 0;
        num_free_ = 0;
        free_ = -1;
    }

    int num_free() const { return num_free_; }
    int num_used() const { return size_ - num_free_; }
    int size() const { return size_; }

	int acquire() {
        if(free_ == -1) {
			resize(size_ * 2 < 4 ? 4 : size_ * 2);
		}
        int alloc_id = free_;
        free_ = *(int*)(data_ + free_);
        num_free_--;
        return alloc_id;
	}

	void release(int idx) {
		int prev_free = free_;
		free_ = idx;
		*(int*)(data_ + idx) = prev_free;
        num_free_++;
	}

    const T& at(int i) const {
        assert(i < size_);
        assert(is_acquired(i));
        return data_[i];
    }

    T& at(int i) {
        assert(i < size_);
        assert(is_acquired(i));
        return data_[i];
    }

    T& operator[](int i) {
        return at(i);
    }

    const T& operator[](int i) const {
        return *at(i);
    }

    int insert(const T& v) {
        int idx = acquire();
        at(idx) = v;
        return idx;
    }
};

#define ADT_ENABLE_TESTS 1

#if defined(ADT_ENABLE_TESTS)
static void adt_test() {

    struct MyData {
        float f = 1.123f;
        int i = 0xaefdca97;
    };

    FreeList<MyData> v;
    assert(v.num_free() == 0);
    assert(v.num_free() == v.size());

    int ii = v.acquire();
    v.at(ii).f = 2.0f;
    v.at(ii).i = 55;
    assert(v.at(ii).i == 55);
    v[ii].i = 77;
    assert(v.at(ii).i == 77);
    MyData& d = v[ii];
    d.f = 521.125f;
    assert(v[ii].f == 521.125f);
    
    int j = v.acquire();
    int k = v.acquire();
    int f = v.acquire();
    int z = v.acquire();
    v.at(z).f = 111.0f;
    v.at(j).i = 28.0f;
    v.at(k).f = 111.0f;
    v.at(f).i = -1.0f;

    assert(v.at(ii).i == 77);
    assert(v.at(ii).f == 521.125f);
    assert(v.at(z).f == 111.0f);
    assert(v.at(j).i == 28);
    assert(v.at(k).f == 111.0f);
    assert(v.at(f).i == -1);

    v.release(k);
    v.release(ii);
    v.release(j);
    v.release(z);
    v.release(f);
    assert(v.num_free() == v.size());
    v.clear();
    assert(v.num_free() == 0);
    assert(v.num_free() == v.size());

    int objects[128];
    for(unsigned int i=0 ;i<sizeof(objects)/sizeof(objects[0]); ++i) {
        objects[i] = v.acquire();
    }
    // v.size() can be more than acquired elements
    int* objects_hist = new int[v.size()];
	for (int i = 0; i < v.size(); ++i) {
		objects_hist[i] = 0;
	}
	for(unsigned int i=0 ;i<sizeof(objects)/sizeof(objects[0]); ++i) {
        objects_hist[objects[i]]++;
    }
    for(int i=0 ;i<v.size(); ++i) {
        assert(objects_hist[i]<=1);
    }
}

struct adt_test_runner {
	adt_test_runner() { adt_test(); }
};
static adt_test_runner adt_test_r;

#endif // ADT_ENABLE_TESTS

