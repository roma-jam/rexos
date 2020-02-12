/*
    RExOS - embedded RTOS
    Copyright (c) 2011-2019, RExOS team
    All rights reserved.

    author: RL (jam_roma@yahoo.com)
*/

#ifndef _NRF_RF_H_
#define _NRF_RF_H_

#include "../../userspace/ipc.h"
#include "../../userspace/io.h"
#include "../../userspace/nrf/nrf_driver.h"
#include "sys_config.h"
#include "nrf_exo.h"
#include <stdbool.h>

#define MAX_PDU_SIZE                    40
#define LOGGER_NUMBER                   20

/** @anchor adv-seg-type
 *  @name Advertisement segment types
 *  @{
 */
#define TYPE_FLAGS                      0x1
#define TYPE_UUID16_INC                 0x2
#define TYPE_UUID16                     0x3
#define TYPE_UUID32_INC                 0x4
#define TYPE_UUID32                     0x5
#define TYPE_UUID128_INC                0x6
#define TYPE_UUID128                    0x7
#define TYPE_NAME_SHORT                 0x8
#define TYPE_NAME                       0x9
#define TYPE_TRANSMITPOWER              0xA
#define TYPE_CONNINTERVAL               0x12
#define TYPE_SERVICE_SOLICITATION16     0x14
#define TYPE_SERVICE_SOLICITATION128    0x15
#define TYPE_SERVICEDATA                0x16
#define TYPE_APPEARANCE                 0x19
#define TYPE_MANUFACTURER_SPECIFIC      0xFF
/** @} */

/** @anchor adv-packet-type
 *  @name Advertisement packet types
 *  @{
 */
#define TYPE_ADV_IND            0
#define TYPE_ADV_DIRECT_IND     1
#define TYPE_ADV_NONCONN_IND    2
#define TYPE_SCAN_REQ           3
#define TYPE_SCAN_RSP           4
#define TYPE_CONNECT_REQ        5
#define TYPE_ADV_SCAN_IND       6
/** @} */

typedef enum {
    RADIO_IO_MODE_RX = 0,
    RADIO_IO_MODE_TX
} RADIO_IO_MODE;

typedef enum {
    RADIO_STATE_IDLE = 0,
    RADIO_STATE_TX,
    RADIO_STATE_RX,
    RADIO_STATE_RX_DATA
} RADIO_STATE;

typedef struct {
    IO* io;
    HANDLE process, timer;
    unsigned int max_size;
    bool active;
    RADIO_MODE mode;
    RADIO_STATE state;
    /* packet data for optimization */
    uint32_t addr, rssi;
    uint8_t pdu[NRF_MAX_PACKET_LENGTH];
} RADIO_DRV;

void nrf_radio_init(EXO* exo);
void nrf_radio_request(EXO* exo, IPC* ipc);


#endif /* _NRF_RF_H_ */
