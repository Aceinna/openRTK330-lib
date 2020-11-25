
#include "user_message_can.h"
#include "can.h"
#include "sae_j1939.h"
#include "sensorsAPI.h"
#include "user_config.h"
#include "app_version.h"
#include "gnss_data_api.h"
#include "ins_interface_API.h"

/** ***************************************************************************
 * @name  process_request_packet() an API of processing request message
 * @brief decode identifier of incoming request message
 *
 * @param [in] desc, rx descriptor
 * @retval N/A
 ******************************************************************************/
void process_request_packet(void *dsc)
{
    struct sae_j1939_rx_desc *desc = (struct sae_j1939_rx_desc *)dsc;
    SAE_J1939_IDENTIFIER_FIELD *ident;
    uint8_t pf_val, req_pf_val, req_ps_val;
    uint8_t *command;
    uint8_t data[8];

    // check desc
    if (desc == NULL)
        return;

    // check identifier
    ident = &(desc->rx_identifier);
    if (ident == NULL)
        return;

    // check receiving buffer
    if (desc->rx_buffer.rx_header.RTR || !desc->rx_buffer.rx_header.IDE || (desc->rx_buffer.rx_header.DLC != 3))
        return;

    // check commands
    command = desc->rx_buffer.data;
    if (command == NULL)
        return;

    pf_val = ident->pdu_format;
    req_pf_val = command[1];
    req_ps_val = command[2];

    if (pf_val != SAE_J1939_PDU_FORMAT_REQUEST)
        return;

    // dispatch the requests to the corresponding handlers
    switch(req_pf_val) {
        case SAE_J1939_PDU_FORMAT_254:
            {
                if (req_ps_val == SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION){
            	    aceinna_j1939_send_software_version();
         	    }
            }
            break;
        case SAE_J1939_PDU_FORMAT_ECU:
            {
                if (req_ps_val == SAE_J1939_GROUP_EXTENSION_ECU){
            	    aceinna_j1939_send_ecu_id();
         	    }
            }
            break;
        case SAE_J1939_PDU_FORMAT_GLOBAL:
            {
                // pasket rate request
                if ((req_ps_val == gEcuConfigPtr->packet_rate_ps)) {
                    aceinna_j1939_send_packet_rate(gEcuConfigPtr->packet_rate);
                }
                // pasket type request 
                else if ((req_ps_val == gEcuConfigPtr->packet_type_ps)) {
                    aceinna_j1939_send_packet_type(gEcuConfigPtr->packet_type);
                }
            }
            break;
        // address claim request
        case SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM:
            process_request_pg(desc);
            break;
        default:
            break;
    }

    msg_params_t params;
    struct sae_j1939_rx_desc *rx_desc = dsc;



    //Get version number
    if (rx_desc->rx_identifier.pdu_format == SAE_J1939_PDU_FORMAT_254 && rx_desc->rx_identifier.pdu_specific == SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION)
    {
      params.data_page = 0;
      params.ext_page = 0;
      params.pkt_type = SAE_J1939_REQUEST_PACKET;
      params.priority = SAE_J1939_REQUEST_PRIORITY;
      params.PF       = SAE_J1939_PDU_FORMAT_254;
      params.PS       = SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION;
      params.len      = SAE_J1939_PAYLOAD_LEN_5_BYTES;   
      data[0] = 1;
    }
    //get ECU ID
    else if (rx_desc->rx_identifier.pdu_format == SAE_J1939_PDU_FORMAT_ECU && rx_desc->rx_identifier.pdu_specific == SAE_J1939_GROUP_EXTENSION_ECU)
    {
      params.data_page = 0;
      params.ext_page = 0;
      params.pkt_type = SAE_J1939_REQUEST_PACKET;
      params.priority = SAE_J1939_REQUEST_PRIORITY;
      params.PF       = SAE_J1939_PDU_FORMAT_ECU;
      params.PS       = SAE_J1939_GROUP_EXTENSION_ECU;
      params.len      = SAE_J1939_PAYLOAD_LEN_8_BYTES;   
      data[0] = 1;
    }
    //get packet type
    else if (rx_desc->rx_identifier.pdu_format == SAE_J1939_PDU_FORMAT_GLOBAL && rx_desc->rx_identifier.pdu_specific == SAE_J1939_GROUP_EXTENSION_PACKET_TYPE)
    {
      params.data_page = 0;
      params.ext_page = 0;
      params.pkt_type = SAE_J1939_REQUEST_PACKET;
      params.priority = SAE_J1939_REQUEST_PRIORITY;
      params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
      params.PS       = SAE_J1939_GROUP_EXTENSION_PACKET_TYPE;
      params.len      = SAE_J1939_REQUEST_LEN;   
      data[0] = 1;
    }
    //get packet rate
    else if (rx_desc->rx_identifier.pdu_format == SAE_J1939_PDU_FORMAT_GLOBAL && rx_desc->rx_identifier.pdu_specific == SAE_J1939_GROUP_EXTENSION_PACKET_RATE)
    {
      params.data_page = 0;
      params.ext_page = 0;
      params.pkt_type = SAE_J1939_REQUEST_PACKET;
      params.priority = SAE_J1939_REQUEST_PRIORITY;
      params.PF       = SAE_J1939_PDU_FORMAT_GLOBAL;
      params.PS       = SAE_J1939_GROUP_EXTENSION_PACKET_RATE;
      params.len      = SAE_J1939_PAYLOAD_LEN_2_BYTES;   
      data[0] = 1;
    }  
	aceinna_j1939_build_msg((void *)data, &params);
}

/** ***************************************************************************
 * @name  process_data_packet() an API of processing data message
 * @brief decode identifier of incoming data message
 *
 * @param [in] desc, rx descriptor
 * @retval N/A
 ******************************************************************************/
void process_data_packet(void *dsc)
{
  	
}


/** ***************************************************************************
 * @name  enqeue_periodic_packets() an API of data packet generation
 * @brief build up MTLT's data packets and send out
 *
 * @param [in]
 * @retval N/A
 ******************************************************************************/
void enqeue_periodic_packets(void)
{
    uint16_t packet_type = gEcuConfigPtr->packet_type;

    if (packet_type & ACEINNA_SAE_J1939_PACKET_ACCELERATION) {
        ACCELERATION_SENSOR accel_sensor;
        double accel[3];
        GetAccelData_mPerSecSq(accel);
        accel_sensor.acceleration_x = (uint16_t)(accel[0] * 100 + 320);
        accel_sensor.acceleration_y = (uint16_t)(accel[1] * 100 + 320);
        accel_sensor.acceleration_z = (uint16_t)(accel[2] * 100 + 320);
        aceinna_j1939_send_acceleration(&accel_sensor);
    }

    if (packet_type & ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE) {
        AUGULAR_RATE rate_sensor;
        float rate[3];
        GetRateData_degPerSec(rate);
        rate_sensor.rate_x = (uint16_t)(rate[0] * 128 + 250);
        rate_sensor.rate_y = (uint16_t)(rate[1] * 128 + 250);
        rate_sensor.rate_z = (uint16_t)(rate[2] * 128 + 250);
        aceinna_j1939_send_angular_rate(&rate_sensor);
    }

    if (packet_type & ACEINNA_SAE_J1939_PACKET_LATLONG) {
        GPS_DATA gps_data;
#ifdef INS_APP
        if (get_mGnssInsSystem_mlc_STATUS() == 4){
            gps_data.latitude = g_ptr_ins_sol->latitude * 10000000;
            gps_data.longitude = g_ptr_ins_sol->longitude * 10000000;
        } else {
            gps_data.latitude = g_gnss_sol.latitude * RAD_TO_DEG * 10000000;
            gps_data.longitude = g_gnss_sol.longitude * RAD_TO_DEG * 10000000;
        }
#else
        gps_data.latitude = g_gnss_sol.latitude * RAD_TO_DEG * 10000000;
        gps_data.longitude = g_gnss_sol.longitude * RAD_TO_DEG * 10000000;
#endif
        aceinna_j1939_send_GPS(&gps_data);
    }

#ifdef INS_APP
    if (packet_type & ACEINNA_SAE_J1939_PACKET_ATTITUDE) {
        ATTITUDE_DATA attitude_data;
        attitude_data.SID = 0;
        attitude_data.Yaw = g_ptr_ins_sol->azimuth * DEG_TO_RAD * 10000;
        attitude_data.Pitch = g_ptr_ins_sol->pitch * DEG_TO_RAD * 10000;
        attitude_data.Roll = g_ptr_ins_sol->roll * DEG_TO_RAD * 10000;
        aceinna_j1939_send_attitude(&attitude_data);
    }
#endif
}

/** ***************************************************************************
 * @name  process_ecu_commands 
* @brief decode incoming SET packets and put the request into the corresponding 
 *       handlers
 * @param [in] command, SET request
 *             ps, pdu specific value
 *             addr, host address
 * @retval 1 successfuk or 0 failure
 ******************************************************************************/
void process_ecu_commands(void * command, uint8_t ps, uint8_t addr)
{ 
    uint8_t* data = (uint8_t*)command;
    BOOL ret;

    if (command == NULL)
        return;

    switch (ps)
    {
    case SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION:
        if (data[0] == ACEINNA_SAE_J1939_REQUEST && data[1] == gEcuConfigPtr->address) {
            ret = SaveUserConfig();
            aceinna_j1939_send_cfgsave(addr, ret);
        }
        break;
    case SAE_J1939_GROUP_EXTENSION_TEST_STATUS:

        break;
    case SAE_J1939_GROUP_EXTENSION_PACKET_RATE:
        if (data[0] == gEcuConfigPtr->address) {
            gEcuConfigPtr->packet_rate = data[1] | (data[2] << 8);
            set_can_packet_rate(gEcuConfigPtr->packet_rate);
        }
        break;
    case SAE_J1939_GROUP_EXTENSION_PACKET_TYPE:
        if (data[0] == gEcuConfigPtr->address) {
            gEcuConfigPtr->packet_type = data[1];
            set_can_packet_type(gEcuConfigPtr->packet_type);
        }
        break;
    default:
        break;
    }

}

/** ***************************************************************************
 * @name  can_config_filter_j1939 
* @brief configure CAN controller for selective reception of CAN messages  
******************************************************************************/
void can_config_filter_j1939(void)
{
    // initialize filter for ECU ID
    can_config_filter_mask_message(SAE_J1939_ECU_ID_BASE, SAE_J1939_ECU_FILTER_BASE_MASK);
    // initialize filter for control messages
    can_config_filter_mask_message(SAE_J1939_CONTROL1_ID_BASE, SAE_J1939_CONTROL1_FILTER_BASE_MASK);
    // initialize filter for requests
    can_config_filter_mask_message(SAE_J1939_REQUEST_ID_BASE, SAE_J1939_REQUEST_FILTER_BASE_MASK);
    // initialize filter for address claim
    can_config_filter_mask_message(SAE_J1939_ADDRESS_CLAIM_ID_BASE, SAE_J1939_ADDRESS_CLAIM_FILTER_BASE_MASK);
    // initialize filter for Bank 1 and Bank 0 commands
    can_config_filter_mask_message(ACEINNA_BANK_ID_BASE, ACEINNA_BANK_FILTER_BASE_MASK);
    // initialize Filter for incoming data messages
    can_config_filter_mask_message(VEHICLE_DATA_ID_BASE, VEHICLE_DATA_FILTER_BASE_MASK);
}
