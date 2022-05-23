/***************************************************************************//**
 * @file
 * @brief rftest_common.h
 *******************************************************************************
 * # License
 * <b>Copyright 2022 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * SPDX-License-Identifier: Zlib
 *
 * The licensor of this software is Silicon Laboratories Inc.
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 ******************************************************************************
 * # Experimental Quality
 * This code has not been formally tested and is provided as-is. It is not
 * suitable for production environments. In addition, this code will not be
 * maintained and there may be no bug maintenance planned for these resources.
 * Silicon Labs may update projects from time to time.
 ******************************************************************************/
#ifndef RFTEST_COMMON_H
#define RFTEST_COMMON_H

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "rail.h"
#include "em_gpio.h"
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
///Please define GPIO for PER test in here
#define SL_RAIL_TEST_PER_PORT   gpioPortA
#define SL_RAIL_TEST_PER_PIN    5

#define RFTEST_MAX_PACKET_LENGTH 128

typedef void (*sl_rail_util_on_event_callback_t)(RAIL_Handle_t rail_handle,
                                                 RAIL_Events_t events);

typedef enum {
  RFTEST_BERTEST_INDEX,     // Multi-PHY index for BER TEST
  USER_APPLICATION_INDEX,   // Multi-PHY index for user application
}config_index_t;

typedef enum {
  RFTEST_IDLE,
  RFTEST_RX,
  RFTEST_PACKET_RCEIVED,
  RFTEST_TX_CONTINUOUS,
  RFTEST_PACKET_SENT,
  RFTEST_BER_TEST,
  RFTEST_PER_TEST,
}rftest_status_t;

typedef struct Stats{
  uint32_t samples;
  int32_t min;
  int32_t max;
  float mean;
  float varianceTimesSamples;
} Stats_t;

typedef struct {
  uint32_t packet_sent;
  uint32_t packet_recevied;
  uint32_t timing_detect;
  uint32_t timing_lost;
  uint32_t preamble_detect;
  uint32_t preamble_lost;
  uint32_t sync_detect;
  uint32_t rx_error;
  uint32_t tx_error;
  uint32_t cal_error;
  uint32_t rx_overflow;
  uint32_t rx_abort;
  uint32_t frame_error;
  uint32_t crc_error;
  uint32_t tx_abort;
  uint32_t tx_block;
  uint32_t tx_underflow;
  uint32_t per_triggers;
  Stats_t  rssi;
}counters_t;

typedef struct BerStatus{
  uint32_t bytesTotal; /**< Number of bytes to receive */
  uint32_t bytesTested; /**< Number of bytes currently tested */
  uint32_t bitErrors; /**< Number of bits errors detected */
  int8_t   rssi; /**< Current RSSI value during pattern acquisition */
} BerStatus_t;

typedef struct {
volatile bool packet_rx;
volatile bool packet_tx;
volatile bool rx_error;
volatile bool tx_error;
volatile bool cal_error;
volatile bool rx_of;
volatile bool tx_request;
volatile bool infinite_tx;
volatile bool ber_enabled;
volatile bool per_end;
volatile bool rftest_enabled;
}radio_flags_t;


// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
extern sl_rail_util_on_event_callback_t sl_rail_util_on_event_callback;
extern const char buildDateTime[];
extern uint16_t rf_channel;
extern uint8_t config_index;
extern uint32_t tx_count;
extern RAIL_Handle_t rftest_handle;
extern radio_flags_t radio_flags;
extern counters_t counters;
extern BerStatus_t berStats;
// -----------------------------------------------------------------------------
//                          Public Function Declarations
// -----------------------------------------------------------------------------
/**************************************************************************//**
 * The function is to configure the packet structure for RF test function.
 *
 * @param[in] railHandle A RAIL instance handle.
 * @param[in] is_fixed_length Packet structure is Whether variable or fixed
 * @param[in] fixed_length   If the packet is fixed length, tell the RF test how
 *            long packet is used.
 * @returns None
 *
 * The function is used for Application logic.
 * It is called infinitely.
 *****************************************************************************/
void rftest_configure(RAIL_Handle_t railHandle,
                      bool is_fixed_length,
                      uint8_t length);
/**************************************************************************//**
 * The rftest application main loop
 *
 * @param None
 * @returns None
 *
 * Please put this function in main loop.
 * It is called infinitely.
 *****************************************************************************/
extern void rftest_app_main(void);
/**************************************************************************//**
 * RAIL event callback register
 *
 * @param[in] func the rail event callback function
 * @returns None
 *
 * The function should be called application initialization.
 * It is called infinitely.
 *****************************************************************************/
extern void rail_event_callback_register(sl_rail_util_on_event_callback_t func);
/**************************************************************************//**
 * The application registering event callback will be processed in this function
 * , and it will be called in sli_rail_util_on_event function.
 *
 * @param[in] rail_handle A RAIL instance handle.
 * @param[in] Events      RAIL events
 * @returns None
 *
 * The function is used for Application logic.
 * It is called infinitely.
 *****************************************************************************/
extern void rail_event_callback_process(RAIL_Handle_t rail_handle,
                                        RAIL_Events_t events);
#endif
