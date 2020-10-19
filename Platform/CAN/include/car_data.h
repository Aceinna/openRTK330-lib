#ifndef _CAR_DATA_H_
#define _CAR_DATA_H_

#include <stdint.h>
#include "constants.h"
#include "timer.h"

#define  CAR_CAN_ID_WHEEL_SPEED     0xAA

typedef struct {
    uint32_t update;
    uint32_t week;
    double timestamp;
    float speed_FR;
    float speed_FL;
    float speed_RR;
    float speed_RL;
    float speed_combined;
    uint8_t fwd;
} WHEEL_SPEED_STRUCT;

void car_can_initialize(void);
void can_config_filter_car(void);
void car_can_data_process(uint32_t stdId, uint8_t* data);
uint8_t car_get_wheel_speed(double *car_speed, uint8_t *fwd, uint32_t *week, double *timestamp);



#endif /* _CAR_DATA_H_ */
