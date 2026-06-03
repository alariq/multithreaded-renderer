#include "ringbuffer.h"

#include<stdio.h>

template<typename T, int Align>
void RingBufferT<T, Align>::dbg_print() const {
    return;

    for(u32 i=0;i<Capacity;++i) {
        printf("%c", data_[i]);
    }
    printf("\n");
    if((write_idx&Mask)== (read_idx&Mask)) {
        for(u32 i=0;i<Capacity;++i) {
            printf("%c", (write_idx&Mask) == i ? '*' : ' ');
        }
    } else {
        for(u32 i=0;i<Capacity;++i) {
            if((write_idx & Mask) == i)
                printf("+");
            else if((read_idx & Mask) == i)
                printf("-");
            else
                printf(" ");
        }
    }
    printf("\n");
}
