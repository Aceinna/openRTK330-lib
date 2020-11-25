#include "car_data.h"
#include "can.h"
#include "string.h"
#include "user_config.h"
#include "ins_interface_API.h"

WHEEL_SPEED_STRUCT wheel_speed;


void car_can_initialize(void)
{
    memset(&wheel_speed, 0, sizeof(WHEEL_SPEED_STRUCT));
    wheel_speed.fwd = 1;
    can_config(0, gUserConfiguration.can_baudrate);
}

void can_config_filter_car(void)
{
    uint8_t i;

    // can_config_filter_list_message(CAR_CAN_ID_WHEEL_SPEED, 0x00);

    filterNum = 0;
    for (i = 0; i < 3; i++) {
        if (gUserConfiguration.odo_mesg[i].usage == 0x55) {
            can_config_filter_list_message(gUserConfiguration.odo_mesg[i].mesgID, 0x00);
        }
    }
}

/*
Now the default data is Toyota Corolla 2019.
If it is another vehicle, modify this function to fit the communication protocol
*/
void car_can_data_process(uint32_t stdId, uint8_t* data)
{
    gtime_t time;
    mcu_time_base_t odo_time;
    int week = 0;
    double timestamp = 0.0;
    int64_t value = 0;
    uint8_t endian = 0, sign = 0, unit = 0, source = 0;
    uint8_t index_byte = 0, index_bitofbyte = 0, bitlen = 0, bitlent = 0, lastbitofbyte = 0;
    double factor = 0.0, offset = 0.0;
    double svalue = 0.0;
    uint8_t i;

    odo_time = g_MCU_time;
    time.time = odo_time.time;
    time.sec = (double)odo_time.msec / 1000;
    timestamp = time2gpst(time, &week);
    if (!is_gnss_start_week_valid() || timestamp < 0.0) {
        return;
    }
    timestamp += (week - get_gnss_start_week()) * SECONDS_IN_WEEK;

    for (i = 0; i < 3; i++) {
        if (gUserConfiguration.odo_mesg[i].usage == 0x55) {
            if (gUserConfiguration.odo_mesg[i].mesgID == stdId) {
                
                if (gUserConfiguration.odo_mesg[i].startbit >= 64 ||
                    gUserConfiguration.odo_mesg[i].length > 64 ||
                    gUserConfiguration.odo_mesg[i].length == 0 ||
                    gUserConfiguration.odo_mesg[i].endian >= 2 ||
                    gUserConfiguration.odo_mesg[i].sign >= 2 ||
                    gUserConfiguration.odo_mesg[i].unit > 2 ||
                    gUserConfiguration.odo_mesg[i].source > 3) {
                    continue;
                }
                if (gUserConfiguration.odo_mesg[i].endian == 0) {
                    if (gUserConfiguration.odo_mesg[i].startbit + gUserConfiguration.odo_mesg[i].length > 64) {
                        continue;
                    }
                } else {
                    if ((((gUserConfiguration.odo_mesg[i].startbit / 8) + 1) * 8 - (gUserConfiguration.odo_mesg[i].startbit % 8)) < gUserConfiguration.odo_mesg[i].length) {
                        continue;
                    }
                }

                value = 0;
                unit = gUserConfiguration.odo_mesg[i].unit;
                source = gUserConfiguration.odo_mesg[i].source;
                factor = gUserConfiguration.odo_mesg[i].factor;
                offset = gUserConfiguration.odo_mesg[i].offset;
                endian = gUserConfiguration.odo_mesg[i].endian;
                sign = gUserConfiguration.odo_mesg[i].sign;
                index_byte = gUserConfiguration.odo_mesg[i].startbit / 8;
                index_bitofbyte = gUserConfiguration.odo_mesg[i].startbit % 8;
                bitlen = gUserConfiguration.odo_mesg[i].length;
                bitlent = bitlen;

                while (bitlen > 0) {
                    lastbitofbyte = index_bitofbyte + bitlen >= 8 ? 0 : 8 - (index_bitofbyte + bitlen);
                    value |= ((data[index_byte] >> index_bitofbyte) & (0xff >> lastbitofbyte)) << (bitlent - bitlen);
                    bitlen = bitlen - (8 - index_bitofbyte - lastbitofbyte);
                    if (endian == 0) {
                        index_byte++;
                    } else {
                        index_byte--;
                    }
                    index_bitofbyte = 0;
                }

                if (sign == 1) {
                    if ((value & (1 << (bitlent - 1))) != 0) {
                        value = -(value - (1 << (bitlent - 1)));
                    }
                }
                svalue = (value + offset) * factor;

                // printf("value = %lld, svalue = %lf\r\n", value, svalue);

                if (source == 0x03) {
                    if (svalue == gUserConfiguration.gears[0]) {
                        wheel_speed.fwd = 1;
                    } else if (svalue == gUserConfiguration.gears[1]) {
                        wheel_speed.fwd = 0;
                    } else if (svalue == gUserConfiguration.gears[2]) {
                        wheel_speed.fwd = 1;
                    } else if (svalue == gUserConfiguration.gears[3]) {
                        wheel_speed.fwd = 1;
                    }

                } else {
                    if (unit == 0) {
                        svalue = svalue / 3.6;
                    } else if (unit == 1) {
                        svalue = svalue * 0.44704;
                    }

                    if (source == 0) {
                        wheel_speed.speed_RR = svalue;
                    } else if (source == 1) {
                        wheel_speed.speed_RL = svalue;
                    } else if (source == 2) {
                        wheel_speed.speed_combined = svalue;
                    }
                    wheel_speed.update |= 1 << source;
                    wheel_speed.week = get_gnss_start_week();
                    wheel_speed.timestamp = timestamp;
                }
            }

        }
    }
}

uint8_t car_get_wheel_speed(double *car_speed, uint8_t *fwd, uint32_t *week, double *timestamp)
{
    if (wheel_speed.update >= 3)
    {
        //printf("FR %f, FL %f, RR %f, RL %f\r\n", wheel_speed.speed_FR, wheel_speed.speed_FL, wheel_speed.speed_RR, wheel_speed.speed_RL);
        // *car_speed = (wheel_speed.speed_RR + wheel_speed.speed_RL) * 0.5 / 3.6;

        *week = wheel_speed.week;
        *timestamp = wheel_speed.timestamp;
        *fwd = wheel_speed.fwd;
        if ((wheel_speed.update & 3) == 3) {
            *car_speed = (wheel_speed.speed_RR + wheel_speed.speed_RL) * 0.5;
        } else if ((wheel_speed.update & (1 << 2)) != 0) {
            *car_speed = wheel_speed.speed_combined;
        }

        wheel_speed.update = 0;
        return 1;
    }
    
    return 0;
}
