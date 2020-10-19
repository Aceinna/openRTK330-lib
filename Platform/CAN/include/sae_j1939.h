/** ***************************************************************************
 * @file sae_j1939.h definitions of SAE J1939 standard
 * @Author Feng
 * @date   Feb. 2017
 * @brief  Copyright (c) 2017 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef SAE_J1939_H
#define SAE_J1939_H
#include "can.h"


// J1939 Name definition
#define ACEINNA_SAE_J1939_VEHICLE_SYSTEM               0
#define ACEINNA_SAE_J1939_VEHICLE_SYSTEM_INSTANCE      0
#define ACEINNA_SAE_J1939_FUNCTION                     131
#define ACEINNA_SAE_J1939_FUNCTION_INSTANCE            0
#define ACEINNA_SAE_J1939_ECU                          0
#define ACEINNA_SAE_J1939_MANUFACTURER_CODE            823


// J1939 64-bit name structure
typedef union {
  struct {
    uint64_t arbitrary_address       : 1;     // arbit bits
    uint64_t industry_group          : 3;     // group bits
    uint64_t vehicle_system_instance : 4;     // system instance bits
    uint64_t vehicle_system          : 7;     // system bits
    uint64_t reserved                : 1;
    uint64_t function                : 8;     // function bits
    uint64_t function_instance       : 5;     // function instance bits
    uint64_t ecu                     : 3;     // ecu bits
    uint64_t manufacture_code        : 11;    // manufacture code from SAE, 823 belongs to Aceinna
    uint64_t identity_number         : 21;    // id bits
  } bits;
  
  uint64_t words;
} SAE_J1939_NAME_FIELD;

// J1939 29-bit identifier
typedef struct {
    uint8_t source;               // source address
    uint8_t pdu_specific;         // ps value
    uint8_t pdu_format;           // pf value
    union {
        struct {
        uint8_t data_page  : 1;   // data page bit
        uint8_t ext_page   : 1;   // extended tata page
        uint8_t priority   : 3;   // priority bits
        uint8_t reserved   : 3;
    } control_bits;
    uint8_t r;
  };
} SAE_J1939_IDENTIFIER_FIELD;


// J1939 PF definition, see protocol spec
// Broadcast PFs - 240 to 255
#define SAE_J1939_PDU_FORMAT_DATA                    240
#define SAE_J1939_PDU_FORMAT_ECU                     253
#define SAE_J1939_PDU_FORMAT_254                     254	
#define SAE_J1939_PDU_FORMAT_GLOBAL                  255
#define SAE_J1939_PDU_FORMAT_241                     241
#define SAE_J1939_PDU_FORMAT_ACK                     232
#define SAE_J1939_PDU_FORMAT_REQUEST                 234
#define SAE_J1939_PDU_FORMAT_ADDRESS_CLAIM           238



// J1939 PS definition, see protocol spec
#define SAE_J1939_PDU_SPECIFIC_25                    25
#define SAE_J1939_PDU_SPECIFIC_243                   243	// 65267 Vehicle Position 1


// J1939 PS definition
#define SAE_J1939_GROUP_EXTENSION_ECU                197
#define SAE_J1939_GROUP_EXTENSION_SOFTWARE_VERSION   218
#define SAE_J1939_GROUP_EXTENSION_SAVE_CONFIGURATION 81
#define SAE_J1939_GROUP_EXTENSION_TEST_STATUS        84
#define SAE_J1939_GROUP_EXTENSION_PACKET_RATE        85
#define SAE_J1939_GROUP_EXTENSION_PACKET_TYPE        86
#define SAE_J1939_GROUP_EXTENSION_ADDR               254
#define SAE_J1939_GROUP_EXTENSION_ACK                255

#define SAE_J1939_GROUP_EXTENSION_ANGULAR_RATE       42
#define SAE_J1939_GROUP_EXTENSION_ACCELERATION       45

// J1939 message priority
#define SAE_J1939_CONTROL_PRIORITY                   6
#define SAE_J1939_REQUEST_PRIORITY                   6
#define SAE_J1939_ACCELERATION_PRIORITY              2
#define SAE_J1939_ANGULAR_PRIORITY                   3
#define SAE_J1939_POSITION_PRIORITY                  6
#define SAE_J1939_ATTITUDE_PRIORITY                  6



// MTLT message's length, see protocol spec
#define SAE_J1939_REQUEST_LEN                          3
#define SAE_J1939_PAYLOAD_MAX_LEN                      8
#define SAE_J1939_PAYLOAD_LEN_8_BYTES                  8
#define SAE_J1939_PAYLOAD_LEN_5_BYTES                  5
#define SAE_J1939_PAYLOAD_LEN_2_BYTES                  2

#define ACEINNA_SAE_J1939_VERSION_PACKET_LEN            6
#define ACEINNA_SAE_J1939_ECU_PACKET_LEN                8

#define ACEINNA_SAE_J1939_CFG_SAVE_LEN                  3
#define ACEINNA_SAE_J1939_PACKET_RATE_LEN               2
#define ACEINNA_SAE_J1939_PACKET_TYPE_LEN               3

#define ACEINNA_SAE_J1939_ACCELERATION_LEN              8
#define ACEINNA_SAE_J1939_ANGULAR_RATE_LEN              8


// MTLT's operation 
#define ACEINNA_SAE_J1939_REQUEST                       0
#define ACEINNA_SAE_J1939_RESPONSE                      1

#define ACEINNA_SAE_J1939_SUCCESS                       0
#define ACEINNA_SAE_J1939_FAILURE                       1


enum {
  ACEINNA_SAE_J1939_PACKET_ACCELERATION     =           0x01,   // acceleration
  ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE     =           0x02,   // angular rate
  ACEINNA_SAE_J1939_PACKET_LATLONG          =           0x04,   // Lattitude, longitude
  ACEINNA_SAE_J1939_PACKET_ATTITUDE         =           0x08,   // Attitude
};

typedef enum {
  _ECU_IDLE                  =   0,        // ready
  _ECU_LOST_CONNECTION       =   1,        // connection lost
  _ECU_NORMAL                =   2,        // normal state
  _ECU_EMPTY_ADDRESS         =   3,        // no address assigned
  _ECU_EXPIRED               =   4         // address expires
} _ECU_STATUS;

// MTLT's states
typedef enum {
  _ECU_INVALID_NAME          =   -1,       // invalid name
  _ECU_TX_OVERFLOW           =   -2,       // tx queue overflow
  _ECU_RX_OVERFLOW           =   -3,       // rx queue overflow
  _ECU_BAUDRATE_DETECT       =   1,        // baud rate detection
  _ECU_WAIT_ADDRESS          =   2,        // none of address
  _ECU_CHECK_ADDRESS         =   3,        // check address conflicted
  _ECU_ALGORITHM_RESET       =   4,        // algo reset
  _ECU_READY                 =   64,       // ready to be used
  _ECU_WAIT_ID               =   65,       // wait for ecu id packets
  _ECU_WAIT_SOFTWARE_VER     =   66,       // wait for software version packets
  _ECU_WAIT_ALG_RESET        =   68,       // wait for algo reset packet
  _ECU_WAIT_CONFIG_SAVE      =   72,       // wait config save packet
  _ECU_WAIT_BUILTIN_TEST     =   80        // wait for bit status packet
} _ECU_STATE;


typedef enum {
  _ECU_MASTER                =   0,        // host device
  _ECU_SLAVE                 =   1         // slave device
} _ECU_CATEGORY;


typedef enum {
  SAE_J1939_REQUEST_PACKET            =    1,       // request packet
  SAE_J1939_ACK_PACKET                =    2,       // ack packet
  SAE_J1939_RESPONSE_PACKET           =    3,       // response packet
  SAE_J1939_SET_PACKET                =    4,       // set packet
  SAE_J1939_DATA_PACKET               =    5        // data packet
} SAE_J1939_PACKET_TYPE;


struct sae_j1939_tx_desc {
  SAE_J1939_PACKET_TYPE       tx_pkt_type;          // tx packet type
  uint8_t                     tx_payload_len;       // tx payload length
  DESC_STATE                  tx_pkt_ready;         // tx state
  SAE_J1939_IDENTIFIER_FIELD  tx_identifier;        // idnetifier of tx packet
  CanTxMsg                    tx_buffer;            // tx buffer

  struct sae_j1939_tx_desc *next;  
};


#define SAE_J1939_MAX_TX_DESC             32
#define SAE_J1939_MAX_RX_DESC             32


struct sae_j1939_rx_desc {
  uint8_t                     rx_pkt_len;         // rx packet length
  DESC_STATE                  rx_pkt_ready;       // rx state
  SAE_J1939_IDENTIFIER_FIELD  rx_identifier;      // indentifier of rx packet
  CanRxMsg                    rx_buffer;          // rx buffer
  
  struct sae_j1939_rx_desc * next;
};


// address claim type
typedef struct {
  SAE_J1939_IDENTIFIER_FIELD addr_claim_pg_pgn;    // address claim PGN
} ADDR_CLAIM_PG_PACKET;


// accleration data payload format
typedef struct {
    uint16_t   acceleration_x;                      // x-axis acceleration
    uint16_t   acceleration_y;                      // y-axis acceleration
    uint16_t   acceleration_z;                      // z-axis acceleration
    uint8_t    lateral_merit        :       2;      // laterar acc merit
    uint8_t    longitudinal_merit   :       2;      // longitudinal merit
    uint8_t    vertical_merit       :       2;      // vertical merit
    uint8_t    transmit_rate        :       2;      // repetition rate
    uint8_t    rsvd;
} ACCELERATION_SENSOR;

// angular rate data payload format
typedef struct {
    uint16_t rate_x;                                // x-axis rate
    uint16_t rate_y;                                // y-axis rate
    uint16_t rate_z;                                // z-axis rate
    uint8_t  rate_x_merit           :       2;      // x-axis rate merit
    uint8_t  rate_y_merit           :       2;      // y-axis  rate merit
    uint8_t  rate_z_merit           :       2;      // z-axis  rate merit
    uint8_t  rsvd                   :       2;      // rsvd
    uint8_t  measurement_latency;                   // latency
} AUGULAR_RATE;

// gps position data payload format
// PGN 65267
// PGN 129025 
typedef struct {
    uint32_t    latitude;                            // gps latitude  0.0000001 deg/bit
    uint32_t    longitude;                           // gps longitude 0.0000001 deg/bit
} GPS_DATA;

// Wheel speed data
// PGN 65215
#pragma pack (1)

// Attitude
// PGN 127257
typedef struct{
	uint8_t SID;
	int16_t Yaw;						// 0.0001 rad/bit
	int16_t Pitch;						// 0.0001 rad/bit
	int16_t Roll;						// 0.0001 rad/bit
	uint8_t Rsvd;
}ATTITUDE_DATA;

#pragma pack ()

#define SAE_J1939_MAX_TABLE_ENTRY         128

// address table
typedef struct {
  	SAE_J1939_NAME_FIELD ecu_name;             	  // ecu's name
  	uint8_t  address;                             // ecu's address
  	_ECU_STATUS status;                        	  // ecu's status
  	_ECU_CATEGORY  category;                   	  // ecu's category
  	uint32_t last_scan_time; // ms                // time of last message sent
  	uint32_t idle_time;    // ms                  // idle time
  	uint32_t alive_time;   // second              // alive time
} ECU_ADDRESS_ENTRY;


// CAN's configuration parameters
typedef struct {
  SAE_J1939_NAME_FIELD ecu_name;            // ecu's name
  uint8_t  address;                         // ecu's address
  uint8_t  baud_rate_detect_enable;         // auto detection enable/disable
  uint16_t baudRate;                        // baud rate
  uint8_t  version[5];                      // software version
  uint16_t packet_rate;                     // packet rate
  uint16_t packet_type;                     // packet type
  uint8_t save_cfg_ps;                      // new ps value of config save
  uint8_t status_ps;                        // new ps value of status message
  uint8_t packet_rate_ps;                   // new ps value of packet rate
  uint8_t packet_type_ps;                   // new ps value of packet type
} EcuConfigurationStruct;


// ECU structure
typedef struct {
  SAE_J1939_NAME_FIELD        *name;          // ecu's name
  uint8_t                     *addr;          // ecu's address
  _ECU_CATEGORY               category;       // ecu's category
  _ECU_STATE                  state;          // ecu's state
  
  ECU_ADDRESS_ENTRY           * addrTbl;      // address table
  
  struct sae_j1939_tx_desc    * curr_tx_desc;               // current tx desc
  struct sae_j1939_rx_desc    * curr_process_desc;          // current desc processed by hardware
  struct sae_j1939_rx_desc    * curr_rx_desc;               // current rx desc
  
  void                        (* init_table)(void);         // initilize address table
  void                        (* update_table)(ECU_ADDRESS_ENTRY *entry);  // update adddress table
  uint8_t                     (* add_entry)(uint8_t, SAE_J1939_NAME_FIELD); // add new entry 
  uint8_t                     (* del_entry)(ECU_ADDRESS_ENTRY *);     // delete an entry
   
  uint8_t                     (* xmit)(struct sae_j1939_tx_desc *);   // transmitting function
 
} ECU_INSTANCE;

extern ECU_INSTANCE *gEcu;


// packet type mask
enum {
  _ECU_CONFIG_PACKET_RATE                   =      1,   // set packet rate
  _ECU_CONFIG_PACKET_TYPE                   =      2,   // set packet type
  _ECU_CONFIG_DIGITAL_FILTER                =      4,   // set lpf
  _ECU_CONFIG_ORIENTATION                   =      8,   // set orientation
  _ECU_CONFIG_USER_BEHAVIOR                 =     16,   // set user behavior
  _ECU_CONFIG_ANGLE_ALARM                   =     32,   // set angular alarm
  _ECU_CONFIG_CONE_ALARM                    =     64,   // set cone alarm
  _ECU_CONFIG_ACCELERATION_PARAM            =    128,   // set acceleration parameters
  _ECU_CONFIG_GROUP_EXTENSION_BANK          =    256,   // set new ps
  _ECU_CONFIG_MASK                          =    511    // configuration mask
};

// command types 
typedef enum {
    ACEINNA_J1939_INVALID_IDENTIFIER         =     -1,    // invalid indentifier
    ACEINNA_J1939_IGNORE                     =      0,    // ignore
    ACEINNA_J1939_SOFTWARE_VERSION           =      2,    // sofware version packet
    ACEINNA_J1939_ECU_ID                     =      3,    // ecu id packet
    ACEINNA_J1939_ALG_RST                    =      4,    // alg reset packet
    ACEINNA_J1939_CFG_SAVE                   =      5,    // config save packet
    ACEINNA_J1939_HARDWARE_TEST              =      6,    // hardware bit packet
    ACEINNA_J1939_SOFTWARE_TEST              =      7,    // software bit packet
    ACEINNA_J1939_STATUS_TEST                =      8,    // status packet
    ACEINNA_J1939_BUILTIN_TEST               =      9,    // built-in test packet
    ACEINNA_J1939_DATA                       =      10,   // data packet
    ACEINNA_J1939_ADDRESS_CLAIM              =      11,   // address claim packet
    ACEINNA_J1939_REQUEST_PACKET             =      12,   // request packet
    ACEINNA_J1939_CONFIG                     =      13    // config packet
} ACEINNA_J1939_PACKET_TYPE;

// ECU address status
enum {
  _ECU_ADDR_AVAILABLE                       =      0,    // available address value
  _ECU_ADDR_OCCUPIED                        =      1     // occupied address value
};

// ECU address structure
typedef struct {
  uint8_t      status;                 // status, available or occupied
  uint8_t      addr;                   // address
} ACEINNA_ECU_ADDR;

// set command type
typedef struct {
    uint8_t request;                              // request or response
    uint8_t dest_address;                         // target's address
    uint8_t success;                              // successful or failure
} COMMAND_SET_PAYLOAD;

typedef struct{
  	int 	pkt_type;
	int 	priority;
	uint8_t PF;
	uint8_t PS;
	uint8_t len;
	uint8_t source;
    uint8_t data_page;
    uint8_t ext_page;
}msg_params_t;



#define ACEINNA_ECU_ADDRESS_MAX              120

extern ECU_INSTANCE gEcuInst;
extern EcuConfigurationStruct gEcuConfig;
extern EcuConfigurationStruct *gEcuConfigPtr;

extern void sae_j1939_initialize(uint16_t baudRate, uint8_t address);

extern void initialize_mapping_table();
extern void update_mapping_table(ECU_ADDRESS_ENTRY *);
extern uint8_t del_ecu_mapping_table(ECU_ADDRESS_ENTRY *);

extern uint8_t is_valid_sae_j1939_identifier(SAE_J1939_IDENTIFIER_FIELD *);

extern void ecu_process(void);
extern void ecu_transmit(void); 

extern uint8_t find_tx_desc(struct sae_j1939_tx_desc **);
extern ACEINNA_J1939_PACKET_TYPE is_valid_config_command(SAE_J1939_IDENTIFIER_FIELD *);
extern ACEINNA_J1939_PACKET_TYPE is_aceinna_data_packet(SAE_J1939_IDENTIFIER_FIELD *);
extern ACEINNA_J1939_PACKET_TYPE is_algorithm_data_packet(SAE_J1939_IDENTIFIER_FIELD *);
extern ACEINNA_J1939_PACKET_TYPE is_valid_address_claim(SAE_J1939_IDENTIFIER_FIELD *);

extern uint8_t send_j1939_packet(struct sae_j1939_tx_desc *);
extern void send_address_claim(ECU_INSTANCE *);
extern void process_request_pg(struct sae_j1939_rx_desc *);
extern void build_request_pkt(struct sae_j1939_tx_desc *);
extern void process_address_claim(struct sae_j1939_rx_desc *desc);

extern void    aceinna_j1939_transmit_isr(void);
extern void    aceinna_j1939_receive_isr(void);
extern uint8_t aceinna_j1939_send_status_packet(uint8_t built_in_type, void * bit_fields);
extern uint8_t aceinna_j1939_send_software_version(void);
extern uint8_t aceinna_j1939_send_ecu_id(void);
extern uint8_t aceinna_j1939_send_acceleration(ACCELERATION_SENSOR * data);
extern uint8_t aceinna_j1939_send_angular_rate(AUGULAR_RATE * data);
extern uint8_t aceinna_j1939_send_cfgsave(uint8_t address, uint8_t success);
extern uint8_t aceinna_j1939_send_packet_rate(uint8_t odr);
extern uint8_t aceinna_j1939_send_packet_type(uint16_t type);
extern uint8_t aceinna_j1939_send_GPS(GPS_DATA * data);
extern uint8_t aceinna_j1939_send_attitude(ATTITUDE_DATA * data);
extern uint8_t aceinna_j1939_build_msg(void *payload, msg_params_t *params);


// priority 6
// PF 254
// PS 0xA0 - 0xBF and 0xE0 - 0xFF
#define VEHICLE_DATA_ID_BASE          0x18FEA000
#define VEHICLE_DATA_FILTER_BASE_MASK 0x18FEA000

// priority 6
// PF 255
// PS 240 and 241 
#define ACEINNA_BANK_ID_BASE          0x18FFF000
#define ACEINNA_BANK_FILTER_BASE_MASK 0x18FFF000


// priority 6
// PF 238
// PS 255 
#define SAE_J1939_ADDRESS_CLAIM_ID_BASE          0x18EEFF00
#define SAE_J1939_ADDRESS_CLAIM_FILTER_BASE_MASK 0x18FFFF00

// priority 6
// PF 234
// PS 255  - global requests
#define SAE_J1939_REQUEST_ID_BASE          0x18EAFF00
#define SAE_J1939_REQUEST_FILTER_BASE_MASK 0x18EAFF00

// priority 6
// PF 255
// PS 50 - 5F
#define SAE_J1939_CONTROL1_ID_BASE          0x18FFFF00
#define SAE_J1939_CONTROL1_FILTER_BASE_MASK 0x18FF5000

// priority 6
// PF 253
// PS 197
#define SAE_J1939_ECU_ID_BASE               0x18FDC500
#define SAE_J1939_ECU_FILTER_BASE_MASK      0x18FDC500


#endif // SAE_J1939_H