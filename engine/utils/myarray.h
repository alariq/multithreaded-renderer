#pragma once

#include <stdint.h>
#include <assert.h>
#include <memory.h>
#include <stdlib.h> // malloc

// POD only 
template<typename T, typename TSize = size_t>
class BufferT {
    enum { elSize = sizeof(T) };
    T* data_;
    TSize size_;
    TSize capacity_;

    template <typename U, typename USize> void operator=(BufferT<U, USize> b) = delete;

public:
#if 1
    BufferT<T, TSize>& operator=(const BufferT<T, TSize>& o) {
        if(&o != this) {
            resize(o.size_);
            if(o.size_) {
                memcpy(data_, o.data_, o.size_ * elSize);
            }
        }
        return *this;
    }

    BufferT(const BufferT<T, TSize>& o):data_(nullptr), size_(0), capacity_(0) {
        resize(o.size_);
        if(o.size_) {
            memcpy(data_, o.data_, o.size_ * elSize);
        }
    }
#endif

    BufferT():data_(nullptr), size_(0), capacity_(0) { }
    ~BufferT() {
        free(data_);
    }

    void reset() {
        size_ = 0;
    }

    bool reserve(TSize new_cap, bool b_shrink = false) {
        if(new_cap > capacity_ || (new_cap < capacity_ && b_shrink)) {

            T* new_data = (T*)malloc(elSize*new_cap);
            assert(new_data);
            if(!new_data && new_cap!=0) {
                return false;
            }

            capacity_ = new_cap;
            size_ = new_cap < size_ ? new_cap : size_;

            if(data_) {
                if(size_) {
                    memcpy(new_data, data_, size_*elSize);
                }
                free(data_);
            }
            data_ = new_data;
        }
        return true;
    }

    bool resize(TSize new_size, bool b_shrink = false) {
        if(reserve(new_size, b_shrink)) {
            size_ = new_size;
            return true;
        }
        return false;
    }

    const T& operator[](TSize i) const {
        assert(i < size_);
        return data_[i];
    }

    T& operator[](TSize i) {
        assert(i < size_);
        return data_[i];
    }

    TSize push(T el) {
        if(size_ == capacity_) {
            reserve(capacity_ == 0 ? 16 : 2*capacity_);
        }
        data_[size_++] = el;
        return size_ - 1;
    }

    void remove_swap(int i) {
        assert(i>=0 && i < size_);
        data_[i] = data_[size_-1];
        size_--;
    }

    void remove(int idx) {
        assert(idx>=0 && idx < size_);
        for(int i=idx;i<size_-1;++i) {
            data_[i] = data_[i+i];
        }
        size_--;
    }

    T& last() {
        assert(size_);
        return data_[size_ - 1];
    }

    T* data() { return data_; }
    const T* data() const { return data_; }

    TSize size() const { return size_; }
#if 0
    void clone(BufferT<T, TSize>& c) {
        c.reserve(this->size_);
        for(TSize i = 0; i<size_; ++i) {
            c[i] = data_[i];
        }
    }
#endif

};
