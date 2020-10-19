/** ***************************************************************************
 * @file sae_j1939.c the definitions of basic functions
 * @Author Feng
 * @date   May 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "platformAPI.h"
#include "osapi.h"
#include "can.h"
#include "sae_j1939.h"
#include "user_config.h"


ECU_INSTANCE gEcuInst;
EcuConfigurationStruct gEcuConfig;
ECU_ADDRESS_ENTRY addrMappingTable[SAE_J1939_MAX_TABLE_ENTRY];

ECU_ADDRESS_ENTRY      *gAddrMapTblPtr = &addrMappingTable[0];
EcuConfigurationStruct *gEcuConfigPtr  = &gEcuConfig;

struct sae_j1939_tx_desc ecu_tx_desc[SAE_J1939_MAX_TX_DESC];
struct sae_j1939_rx_desc ecu_rx_desc[SAE_J1939_MAX_RX_DESC];

#define SAE_J1939_TX_THRESHOLD                     16
#define SAE_J1939_MAX_IDLE_TIME                    1000000    //ms

static ACEINNA_ECU_ADDR ecu_pool[ACEINNA_ECU_ADDRESS_MAX];


/** ***************************************************************************
 * @name add_ecu_mapping_table() add an entry in address pool
 * @brief  insert a new occupied address in table
 *
 * @param [in] addr, address value
 *             name, ecu's name
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t add_ecu_mapping_table(uint8_t addr, SAE_J1939_NAME_FIELD name) 
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for ( i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (addrTbl->status == _ECU_IDLE){
      break;
    }
    addrTbl++;
  }
  
  if ( i == SAE_J1939_MAX_TABLE_ENTRY)
    return 0;
  
  addrTbl->ecu_name.words = name.words;
  addrTbl->address = addr;
  addrTbl->status = _ECU_NORMAL;
  addrTbl->category = _ECU_SLAVE;
  addrTbl->last_scan_time = 0;
  addrTbl->idle_time = 0;
  addrTbl->alive_time = 0;
  
  return 1;
}

/** ***************************************************************************
 * @name send_j1939_packet() a general API of sending packet
 * @brief  perform transmitting function
 *
 * @param [in] desc, tx descriptor
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t send_j1939_packet(struct sae_j1939_tx_desc *desc)
{
  CanTxMsg* TxMsg;
  uint8_t result = 0;
  
  // assign tx buffer  
  TxMsg = &(desc->tx_buffer);
  
  // call transmitting API
  result = can_transmit(TxMsg);
  
  // check transmission
//  result = CAN_TransmitStatus(CAN1, 0);
  
  return result;
  
}


/** ***************************************************************************
 * @name initialize_j1939_config() initialize basic CAN's global parameters
 * @brief  called in init routine to assign values to CAN's global parameters
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void initialize_j1939_config(uint16_t baudRate, uint8_t address)
{  
  // assign the global bau rate value, go to default value if invalid
  gEcuConfigPtr->baudRate  = baudRate;
  
  // set address and data packet type
  gEcuConfigPtr->address   = address;
//  gEcuConfigPtr->packet_type = platformGetCANPacketType();
  
  // init config save ps values
  if (!gEcuConfigPtr->save_cfg_ps)
      gEcuConfigPtr->save_cfg_ps = SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION;
  
  // init status ps values
  if (!gEcuConfigPtr->status_ps)
    gEcuConfigPtr->status_ps = SAE_J1939_GROUP_EXTENSION_TEST_STATUS;
  
  // init odr ps values
  if (!gEcuConfigPtr->packet_rate_ps)
    gEcuConfigPtr->packet_rate_ps = SAE_J1939_GROUP_EXTENSION_PACKET_RATE;
  
  // init packet type ps value
  if (!gEcuConfigPtr->packet_type_ps)
    gEcuConfigPtr->packet_type_ps = SAE_J1939_GROUP_EXTENSION_PACKET_TYPE;
  
  return;
}

/** ***************************************************************************
 * @name process_j1939_packet() call ecu_process() to decode the incoming message
 * @brief  a genetal API called by caller
 *
 * @param [in] rx_desc, current rx descriptor
 * @retval N/A
 ******************************************************************************/
void process_j1939_packet(struct sae_j1939_rx_desc * rx_desc)
{
  if (rx_desc->rx_pkt_ready != DESC_OCCUPIED)
    return;
  
   // process incoming packets
   ecu_process();
  
  return;
}

/** ***************************************************************************
 * @name sae_j1939_initialize() general initialization of all data structures
 * @brief  called in initial routine 
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void sae_j1939_initialize(uint16_t baudRate, uint8_t address)
{
  int i;
    
  initialize_j1939_config(baudRate, address);


  memset(ecu_tx_desc, 0, sizeof(ecu_tx_desc));
  memset(ecu_rx_desc, 0, sizeof(ecu_rx_desc));

  
  for ( i = 0; i < SAE_J1939_MAX_TX_DESC; i++) {
    if (i == SAE_J1939_MAX_TX_DESC - 1)
    {
        ecu_tx_desc[i].next = &ecu_tx_desc[0]; 
    }
    else 
    {
        ecu_tx_desc[i].next = &ecu_tx_desc[i+1]; 
    }
  }

  for ( i = 0; i < SAE_J1939_MAX_RX_DESC; i++) {
    if (i == SAE_J1939_MAX_RX_DESC - 1)
    {
        ecu_rx_desc[i].next = &ecu_rx_desc[0]; 
    }
    else 
    {
        ecu_rx_desc[i].next = &ecu_rx_desc[i+1]; 
    }
  }
  
  for (i = 0; i < ACEINNA_ECU_ADDRESS_MAX; i++) {
    ecu_pool[i].status = _ECU_ADDR_AVAILABLE;
    ecu_pool[i].addr = i + 128;
  }
  
  initialize_mapping_table();
  
  
  gEcuInst.name = (SAE_J1939_NAME_FIELD *)&gEcuConfigPtr->ecu_name;
  gEcuInst.addr = &(gEcuConfig.address);
  gEcuInst.category = _ECU_SLAVE;
  if (gEcuConfig.baud_rate_detect_enable) {
    gEcuInst.state = _ECU_BAUDRATE_DETECT;
  } else {
    if (*gEcuInst.addr)
      gEcuInst.state = _ECU_CHECK_ADDRESS;
    else
      gEcuInst.state = _ECU_WAIT_ADDRESS;
  }
  
  gEcuInst.addrTbl = gAddrMapTblPtr;
  
  // FL hardcode address here
#ifdef DEMO_PROTO
  gEcuInst.state = _ECU_READY;
#endif
  
  gEcuInst.curr_tx_desc = &(ecu_tx_desc[0]);
  gEcuInst.curr_rx_desc = &(ecu_rx_desc[0]);
  gEcuInst.curr_process_desc = &(ecu_rx_desc[0]);
  gEcuInst.init_table = initialize_mapping_table;
  gEcuInst.update_table = update_mapping_table;
  gEcuInst.add_entry = add_ecu_mapping_table;
  gEcuInst.del_entry = del_ecu_mapping_table;
  gEcuInst.xmit = send_j1939_packet;
  

  can_rtx_fun_config(aceinna_j1939_transmit_isr, aceinna_j1939_receive_isr);

  return ;
}


/** ***************************************************************************
 * @name initialize_mapping_table() initialize the mapping table
 * @brief  called in intial routine 
 *
 * @param [in] 
 * @retval N/A
 ******************************************************************************/
void initialize_mapping_table(void)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for (i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (!addrTbl->ecu_name.words)
        addrTbl->status = _ECU_IDLE;
    else if (!addrTbl->address)
        addrTbl->status = _ECU_EMPTY_ADDRESS;
    else
        addrTbl->status = _ECU_EXPIRED;
    
    addrTbl++;
  }
    
  return;
}

/** ***************************************************************************
 * @name build_request_pkt() build up a general request packet
 * @brief  perform the header of request packet 
 *
 * @param [in] req_desc, a tx descriptor
 * @retval N/A
 ******************************************************************************/
void build_request_pkt(struct sae_j1939_tx_desc * req_desc)
{
  req_desc->tx_pkt_type = SAE_J1939_REQUEST_PACKET;
  req_desc->tx_payload_len = SAE_J1939_REQUEST_LEN;
  req_desc->tx_identifier.control_bits.priority = SAE_J1939_REQUEST_PRIORITY;
  req_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_REQUEST;
  req_desc->tx_identifier.pdu_specific = SAE_J1939_PDU_FORMAT_GLOBAL;
  req_desc->tx_identifier.source = *(uint8_t *)gEcuInst.addr;
  req_desc->tx_pkt_ready = DESC_OCCUPIED;
  
  return;
}

/** ***************************************************************************
 * @name send_address_claim() send out adress claim packet
 * @brief  perform the entire packet of address claim, header and payload
 *
 * @param [in] ecu, self ecu object
 * @retval N/A
 ******************************************************************************/
void send_address_claim(ECU_INSTANCE *ecu)
{
  struct sae_j1939_tx_desc *addr_claim_desc;
  ADDR_CLAIM_PG_PACKET     addr_claim_pg;
  uint8_t address = *ecu->addr;
    
  if (find_tx_desc(&addr_claim_desc) == 0)
    return;
  
  if (!address) {
    addr_claim_pg.addr_claim_pg_pgn.r = 0;
    addr_claim_pg.addr_claim_pg_pgn.control_bits.priority = SAE_J1939_CONTROL_PRIORITY;
    addr_claim_pg.addr_claim_pg_pgn.pdu_format = SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM;
    addr_claim_pg.addr_claim_pg_pgn.pdu_specific = SAE_J1939_GROUP_EXTENSION_ACK;
  
    build_request_pkt(addr_claim_desc);
    
    addr_claim_desc->tx_identifier.source = SAE_J1939_GROUP_EXTENSION_ADDR;
  
    addr_claim_desc->tx_buffer.data[0] = addr_claim_pg.addr_claim_pg_pgn.r;
    addr_claim_desc->tx_buffer.data[1] = addr_claim_pg.addr_claim_pg_pgn.pdu_format;
    addr_claim_desc->tx_buffer.data[2] = addr_claim_pg.addr_claim_pg_pgn.pdu_specific;
  } else { 
      addr_claim_desc->tx_pkt_type = ACEINNA_J1939_ADDRESS_CLAIM;
      addr_claim_desc->tx_payload_len = SAE_J1939_PAYLOAD_MAX_LEN;
      addr_claim_desc->tx_identifier.r = 0;
      addr_claim_desc->tx_identifier.control_bits.priority = SAE_J1939_REQUEST_PRIORITY;
      addr_claim_desc->tx_identifier.pdu_format = SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM;
      addr_claim_desc->tx_identifier.pdu_specific = SAE_J1939_GROUP_EXTENSION_ACK;
      addr_claim_desc->tx_identifier.source = *(uint8_t *)gEcuInst.addr;

      memcpy((void *)addr_claim_desc->tx_buffer.data, (void *)(&(ecu->name->words)), SAE_J1939_PAYLOAD_MAX_LEN);
  }
  
  addr_claim_desc->tx_buffer.tx_header.ExtId = ((addr_claim_desc->tx_identifier.r << 24) |
                                      (addr_claim_desc->tx_identifier.pdu_format << 16) |
                                      (addr_claim_desc->tx_identifier.pdu_specific << 8) |
                                      (addr_claim_desc->tx_identifier.source));
  addr_claim_desc->tx_buffer.tx_header.IDE = CAN_ID_EXT;
  addr_claim_desc->tx_buffer.tx_header.RTR = CAN_RTR_DATA;
  addr_claim_desc->tx_buffer.tx_header.DLC = addr_claim_desc->tx_payload_len;
  addr_claim_desc->tx_pkt_ready  = DESC_OCCUPIED;
  
  return;
}

/** ***************************************************************************
 * @name allocate_ecu_addr() find out the available address 
 * @brief  search address in the pool and find out first available address
 *         otherwise, return max address
 * @param [in] 
 * @retval address or max address
 ******************************************************************************/
uint8_t allocate_ecu_addr(void)
{
  int i;
  
  for (i = 0; i < ACEINNA_ECU_ADDRESS_MAX; i++) {
    if (ecu_pool[i].status == _ECU_ADDR_AVAILABLE) {
      ecu_pool[i].status = _ECU_ADDR_OCCUPIED;
      return ecu_pool[i].addr;
    }
  }
  
  return 247;
}


/** ***************************************************************************
 * @name del_ecu_mapping_table() an API of erasing an used address message
 * @brief  search address in the mapping table and delete the record
 *         otherwise, return 0
 * @param [in] input, ecu object
 * @retval 0
 ******************************************************************************/
uint8_t del_ecu_mapping_table(ECU_ADDRESS_ENTRY *input)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for ( i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (addrTbl->ecu_name.words == input->ecu_name.words) {
      ecu_pool[addrTbl->address - 128].status = _ECU_ADDR_AVAILABLE;
      memset((void *)addrTbl, '\0', sizeof(ECU_ADDRESS_ENTRY));
      addrTbl->status = _ECU_IDLE;
      
      break;
    }
  }
  
  return 0;
}

/** ***************************************************************************
 * @name update_mapping_table() an API of updating the table by the input message
 * @brief  search address in the mapping table and find out the record
 *         otherwise, add a new entry
 * @param [in] entry, ecu object
 * @retval 
 ******************************************************************************/
void update_mapping_table(ECU_ADDRESS_ENTRY *entry)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
   
  for (i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (entry->ecu_name.words == addrTbl->ecu_name.words) {
      addrTbl->address = entry->address;
      addrTbl->status = entry->status;
      addrTbl->category = entry->category;
      if (entry->status == _ECU_NORMAL)
          addrTbl->idle_time = 0;
      
      return;
    }
    
    addrTbl++;
  }
  
  add_ecu_mapping_table(entry->address, entry->ecu_name);
 
  return;    
}

/** ***************************************************************************
 * @name find_remote_ecu() an API of finding out peer device
 * @brief  search address in the mapping table and find out peer device
 *        
 * @param [in] address, ecu's address
 *             name, ecu's name
 * @retval ecu object or NULL
 ******************************************************************************/
ECU_ADDRESS_ENTRY * find_remote_ecu(uint8_t addr, SAE_J1939_NAME_FIELD name)
{
  int i;
  ECU_ADDRESS_ENTRY *addrTbl =  gAddrMapTblPtr;
  
  for (i = 0; i < SAE_J1939_MAX_TABLE_ENTRY; i++) {
    if (name.words == addrTbl->ecu_name.words) {
      if (addr != addrTbl->address)
        addrTbl->address = addr;
      return addrTbl;
    }
    
    addrTbl++;
  }
  
  return NULL;  
}

/** ***************************************************************************
 * @name find_tx_desc() get the available tx descriptor
 * @brief perform an available tx descriptor for ready-to-go packet
 *        
 * @param [in] input, tx descriptor pointer
 *             
 * @retval 0 successful and 1 failure
 ******************************************************************************/
uint8_t find_tx_desc(struct sae_j1939_tx_desc **input)
{
  *input = gEcuInst.curr_tx_desc;
  
  while ((*input)->tx_pkt_ready != DESC_IDLE) 
  {
    *input =  (*input)->next;
    if (*input == gEcuInst.curr_tx_desc) {
//      gEcuInst.state = _ECU_TX_OVERFLOW;
      return 0;
    }
  }
  
  return 1;
}


/** ***************************************************************************
 * @name is_valid_pf() check pf is supported or not
 * @brief a general API of pf value checking
 *        
 * @param [in] pf_val, pf number
 *             
 * @retval 1 successful or 0 failure
 ******************************************************************************/
uint8_t is_valid_pf(uint8_t pf_val)
{
  if ((pf_val == SAE_J1939_PDU_FORMAT_ACK) ||
      (pf_val == SAE_J1939_PDU_FORMAT_REQUEST) ||
      (pf_val == SAE_J1939_PDU_FORMAT_DATA) ||
      (pf_val == SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM) ||
      (pf_val == SAE_J1939_PDU_FORMAT_ECU) ||
      (pf_val == SAE_J1939_PDU_FORMAT_254) ||
      (pf_val == SAE_J1939_PDU_FORMAT_GLOBAL))
      return 1;
       
   return 0;
}

/** ***************************************************************************
 * @name process_request_pg() process the incoming request packets
 * @brief a general API of request message handler
 *        
 * @param [in] rx_desc, rx descriptor
 *             
 * @retval N/A
 ******************************************************************************/
void process_request_pg(struct sae_j1939_rx_desc *rx_desc)
{
     
  if (rx_desc == NULL)
    return;
  
  send_address_claim(&gEcuInst);
  
  return;  
}


/** ***************************************************************************
 * @name process_address_claim() process the incoming address claiming request
 * @brief a general API of address claim handler
 *        
 * @param [in] rx_desc, rx descriptor
 *             
 * @retval N/A
 ******************************************************************************/
void process_address_claim(struct sae_j1939_rx_desc *desc)
{
  uint8_t source_addr;
  uint8_t result;
  SAE_J1939_NAME_FIELD ecu_name;
  SAE_J1939_IDENTIFIER_FIELD * ident;
  ECU_ADDRESS_ENTRY * remote_ecu;
  
  if (desc == NULL)
    return;
  
  ident = &desc->rx_identifier;
  if (ident == NULL)
    return;
  
  source_addr = ident->source;
  memcpy((void *)&ecu_name.words, (void *)desc->rx_buffer.data, 8); 
  remote_ecu = find_remote_ecu(source_addr, ecu_name);
  if ((remote_ecu == NULL) && (source_addr != *gEcuInst.addr)) {
      if (!(result = add_ecu_mapping_table(source_addr, ecu_name))) {
//      ERROR_STRING("\r\naddress table full.\r\n");
      }
  } else if (source_addr == *gEcuInst.addr) {
    if (remote_ecu->ecu_name.words > gEcuInst.name->words)
      send_address_claim(&gEcuInst);
    else {
      *gEcuInst.addr = allocate_ecu_addr();
      save_ecu_address((uint16_t)gEcuConfigPtr->address);
    }
  }
  
  gEcuInst.state = _ECU_READY;
  
  return;  
  
}

/** ***************************************************************************
 * @name is_valid_sae_j1939_identifier() check identifier is valid or invalid
 * @brief a general API of identifier checking
 *        
 * @param [in] identifier, identifier message
 *             
 * @retval 1 valid or 0 invalid
 ******************************************************************************/
uint8_t is_valid_sae_j1939_identifier(SAE_J1939_IDENTIFIER_FIELD *identifier)
{
  if (identifier->control_bits.data_page) 
    return 0;
  
  if (!is_valid_pf(identifier->pdu_format))
    return 0;
  
  return 1;
}

/** ***************************************************************************
 * @name is_valid_address_claim() check incoming address claim valid or invalid
 * @brief a general API of address claim message 
 *        
 * @param [in] ident, identifier message
 *             
 * @retval ACEINNA_J1939_ADDRESS_CLAIM, ACEINNA_J1939_INVALID_IDENTIFIER, or ACEINNA_J1939_IGNORE
 ******************************************************************************/
ACEINNA_J1939_PACKET_TYPE is_valid_address_claim(SAE_J1939_IDENTIFIER_FIELD *ident)
{
  uint8_t pf_val, ps_val;
 
  if (ident == NULL)
    return ACEINNA_J1939_INVALID_IDENTIFIER;
    
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM) &&
      (ps_val == SAE_J1939_GROUP_EXTENSION_ACK)) {
       return ACEINNA_J1939_ADDRESS_CLAIM;
  }
  
  return ACEINNA_J1939_IGNORE;
}

/** ***************************************************************************
 * @name is_aceinna_data_packet() check incoming packet is data packet 
 * @brief a general API of checking the data packets belong to ss2, angular or accel
 *        
 * @param [in] ident, identifier message
 *             
 * @retval ACEINNA_J1939_DATA or ACEINNA_J1939_IGNORE
 ******************************************************************************/
ACEINNA_J1939_PACKET_TYPE is_aceinna_data_packet(SAE_J1939_IDENTIFIER_FIELD *ident)
{
  uint8_t pf_val, ps_val;
 
  pf_val = ident->pdu_format;
  ps_val = ident->pdu_specific;
  
  if ((pf_val == SAE_J1939_PDU_FORMAT_DATA) &&
      ((ps_val == SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_ACCELERATION))) {
       return ACEINNA_J1939_DATA;
  }
  if (pf_val == SAE_J1939_PDU_FORMAT_241 &&
      ps_val == SAE_J1939_PDU_SPECIFIC_25) {

  }
  if (pf_val == SAE_J1939_PDU_FORMAT_254 &&
      ps_val == SAE_J1939_PDU_SPECIFIC_243) {

  }

  return ACEINNA_J1939_IGNORE;
}

/** ***************************************************************************
 * @name is_data_packet() check incoming packet is data packet 
 * @brief a general API of checking the data packets belong to ss2, angular or accel
 *        
 * @param [in] ident, identifier message
 *             
 * @retval ACEINNA_J1939_DATA or ACEINNA_J1939_IGNORE
 ******************************************************************************/
ACEINNA_J1939_PACKET_TYPE is_algorithm_data_packet(SAE_J1939_IDENTIFIER_FIELD *ident)
{
//   uint8_t pf_val, ps_val;
 
  
//   pf_val = ident->pdu_format;
//   ps_val = ident->pdu_specific;

//   if ((pf_val == SAE_J1939_PDU_FORMAT_254) &&
//       ((ps_val == SAE_J1939_PDU_SPECIFIC_243))) {
//        return ACEINNA_J1939_DATA;
//   }

  return ACEINNA_J1939_IGNORE;
}

