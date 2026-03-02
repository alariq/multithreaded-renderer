#pragma once

#include <stdint.h>

// POD only 
template<typename T, typename TSize = size_t>
class BufferT {
    enum { elSize = sizeof(T) };
    T* data_;
    TSize size_;
    TSize capacity_;

public:

    BufferT():data_(nullptr), size_(0), capacity_(0) { }

    void reset() {
        size_ = 0;
    }

    bool resize(TSize new_cap, bool b_shrink = false) {
        if(new_cap > capacity_ || (new_cap < capacity_ && b_shrink)) {

            T* new_data = (T*)malloc(elSize*new_cap);
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
            resize(capacity_ == 0 ? 16 : 2*capacity_);
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

};
