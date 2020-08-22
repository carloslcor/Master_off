#ifndef DGIO_H
#define DGIO_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct dgio {
    uint8_t pin_num;
    uint8_t blink_presc;
} dgio_t;

void dgio_set_timer(unsigned long *tim);
void dgio_read(dgio_t *dio);
void dgio_write(dgio_t *dio, uint8_t state);
int8_t dgio_update(dgio_t *dio);
int8_t dgio_update_arr(dgio_t *dio, size_t size);

#ifdef __cplusplus
}
#endif

#endif /* DGIO_H */
