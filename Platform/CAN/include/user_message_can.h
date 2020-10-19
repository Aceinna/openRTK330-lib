#ifndef USER_MESSAGING_CAN_H
#define USER_MESSAGING_CAN_H

#include <stdint.h>
#include "constants.h"

void process_request_packet(void *dsc);
void process_data_packet(void *dsc);

void enqeue_periodic_packets(void);
void process_ecu_commands(void * command, uint8_t ps, uint8_t addr);

void can_config_filter_j1939(void);

#endif /* USER_CONFIGURATION_H */
