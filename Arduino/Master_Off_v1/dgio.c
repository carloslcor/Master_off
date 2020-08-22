#include "dgio.h"

unsigned long *dgio_tim_cont = NULL;

void dgio_set_timer(unsigned long *tim){
    dgio_tim_cont = tim;
}

void dgio_read(dgio_t *dio){
    digitalRead(dio->pin_num);
}

void dgio_write(dgio_t *dio, uint8_t state){
    digitalWrite(dio->pin_num, state);
}

int8_t dgio_update(dgio_t *dio){
    uint16_t doub;
    if (dio->blink_presc > 0) {
        if(dgio_tim_cont == NULL){
            return -1;
        }
        doub = dio->blink_presc*2;
        if( (*dgio_tim_cont % doub) < dio->blink_presc ){
            dgio_write(dio, 0);
        } else {
            dgio_write(dio, 1);
        }
    }
    return 0;
}
int8_t dgio_update_arr(dgio_t *dio, size_t siz){
    while(siz){
        dgio_update(&dio[siz]);
        siz--;
    }
    dgio_update(&dio[siz]);
}

