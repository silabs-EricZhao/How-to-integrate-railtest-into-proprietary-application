/***************************************************************************//**
 * @file
 * @brief rftest_app.c
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
// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <rftest_common.h>
#include <stdint.h>
#include "rail.h"
#include "sl_component_catalog.h"
#include "em_chip.h"
#include "app_log.h"
#include "sl_cli.h"
#include "sl_rail_util_init.h"
#include "rail_config.h"
#include "sli_rail_util_callbacks.h" // for internal-only callback signatures
#include "rftest_app.h"
#include "rftest_common.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------
#define RFTEST_FIXED_PACKET_LENGTH   16
#define TX_INTETVAL_DEFAULT          20000 // 20ms interval between two packets

#ifdef SL_CATALOG_APP_ASSERT_PRESENT
#include "app_assert.h"
#define APP_ASSERT(expr, ...) app_assert(expr,__VA_ARGS__)
#else
#define APP_ASSERT(expr, ...) \
  do {                        \
    if (!(expr)) {            \
      while (1) ;             \
    }                         \
  } while (0)
#endif

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
void rfest_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events);
void rftest_info_print(void);
void rftest_tx_timer_callback(RAIL_Handle_t arg);
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
rftest_status_t radio_state = RFTEST_RX;
uint32_t pack_num = 0;
uint32_t sent_packet_num = 0;
uint32_t tx_count = 0;
RAIL_Time_t tx_timer = 0;
radio_flags_t radio_flags;
counters_t counters;
BerStatus_t berStats = { 0 };
// Variable which holds the receive frequency offset for the period of time
// between when the frequency offset is measured (in either the
// RAIL_EVENT_RX_SYNC1_DETECT event or the RAIL_EVENT_RX_SYNC2_DETECT event)
// until reception of the packet is completed or aborted.
RAIL_FrequencyOffset_t rxFreqOffset = RAIL_FREQUENCY_OFFSET_INVALID;
// Structures that hold default TX & RX Options
RAIL_TxOptions_t txOptions = RAIL_TX_OPTIONS_DEFAULT;
RAIL_RxOptions_t rxOptions = RAIL_RX_OPTIONS_DEFAULT;

uint16_t      rf_channel = 0;
RAIL_Handle_t rftest_handle;
uint8_t       config_index;
uint8_t       tx_length;
uint8_t tx_data[RFTEST_MAX_PACKET_LENGTH] = {15, 0, 1, 2, 3, 4, 5, 6, 7,
                                             8, 9, 10, 11, 12, 13, 14};
uint8_t rx_data[RFTEST_MAX_PACKET_LENGTH];

sl_rail_util_on_event_callback_t sl_rail_util_on_event_callback = NULL;


// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Public Function Definitions
//
/******************************************************************************
 * rftest initialization, re-configure for RAIL EVENT and data configuration.
 *****************************************************************************/
void rftest_init(void)
{
  RAIL_Status_t rail_status;
  RAIL_Events_t RAIL_Events = 0;
  // Get the RAIL handler
  rftest_handle = sl_rail_util_get_handle(SL_RAIL_UTIL_HANDLE_INST0);
  RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);

  RAIL_DataConfig_t data_config = {
     .txSource = TX_PACKET_DATA,
     .rxSource = RX_PACKET_DATA,
     .txMethod = PACKET_MODE,
     .rxMethod = PACKET_MODE,
  };
  rail_status = RAIL_ConfigData(rftest_handle, &data_config);
  APP_ASSERT((RAIL_STATUS_NO_ERROR == rail_status),
            "RAIL_ConfigData failed, return value: %d", rail_status);

//  // Load the channel configuration for the specified index.
//  config_index = RFTEST_PACKET_INDEX;
//  rf_channel = RAIL_ConfigChannels(rftest_handle,
//                                   channelConfigs[config_index],
//                                   &sli_rail_util_on_channel_config_change);

  // Configure the Event for RF test
    RAIL_Events = RAIL_EVENT_RX_PREAMBLE_DETECT
                    | RAIL_EVENT_RX_PREAMBLE_LOST
                    | RAIL_EVENT_RX_TIMING_DETECT
                    | RAIL_EVENT_RX_TIMING_LOST
                    | RAIL_EVENT_RX_SYNC1_DETECT
                    | RAIL_EVENT_RX_SYNC2_DETECT
                    | RAIL_EVENTS_RX_COMPLETION
                    | RAIL_EVENTS_TX_COMPLETION;

    rail_status = RAIL_ConfigEvents(rftest_handle, RAIL_EVENTS_ALL, RAIL_Events);
    APP_ASSERT((RAIL_STATUS_NO_ERROR == rail_status),
               "RAIL_ConfigEvents failed, return value: %d", rail_status);
    // Configure transition
    RAIL_StateTransitions_t tx_transitions = {
       .success = RAIL_RF_STATE_RX,
       .error = RAIL_RF_STATE_RX
     };
     RAIL_StateTransitions_t rx_transitions = {
       .success = RAIL_RF_STATE_RX,
       .error = RAIL_RF_STATE_RX
     };
     rail_status = RAIL_SetTxTransitions(rftest_handle,
                                         &tx_transitions);
     APP_ASSERT((RAIL_STATUS_NO_ERROR == rail_status),
                "RAIL_SetTxTransitions failed, return value: %d",
                rail_status);
     rail_status = RAIL_SetRxTransitions(rftest_handle,
                                         &rx_transitions);
     APP_ASSERT((RAIL_STATUS_NO_ERROR == rail_status),
                   "RAIL_SetRxTransitions failed, return value: %d",
                   rail_status);



    // Configure the event callback for rftest
    rail_event_callback_register(rfest_util_on_event);

    // Init the parameters
    memset(&radio_flags, 0, sizeof(radio_flags));
    memset(&counters, 0, sizeof(counters));
    radio_flags.rftest_enabled = true;
// For PER test
#if defined (SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
  GPIO_PinModeSet(SL_RAIL_TEST_PER_PORT,
                  SL_RAIL_TEST_PER_PIN,
                  gpioModePushPull,
                  1);
#endif

  // print the basic info
  rftest_info_print();

  rail_status = RAIL_StartRx(rftest_handle, rf_channel, NULL);
  if (rail_status != RAIL_STATUS_NO_ERROR) {
    app_log_error("RAIL_StartRx(), Error:%d\r\n", rail_status);
  }
}

/******************************************************************************
 * Print chip info and RAIL version
 *****************************************************************************/
void rftest_info_print(void)
{
#define EVALIT(a, b)  a # b
#define PASTEIT(a, b) EVALIT(a, b)
#if defined(_SILICON_LABS_32B_SERIES_1)
 #if   (_SILICON_LABS_32B_SERIES_1_CONFIG == 1)
  #define FAMILY_NAME "EFR32XG1"
 #else
  #define FAMILY_NAME PASTEIT("EFR32XG1", _SILICON_LABS_32B_SERIES_1_CONFIG)
 #endif
#elif defined(_SILICON_LABS_32B_SERIES_2)
  #define FAMILY_NAME PASTEIT("EFR32XG2", _SILICON_LABS_32B_SERIES_2_CONFIG)
#else
  #define FAMILY_NAME "??"
#endif
#ifndef _SILICON_LABS_GECKO_INTERNAL_SDID
  #define _SILICON_LABS_GECKO_INTERNAL_SDID 0U
#endif

  // Print chip information.
 app_log_info("{rftest}{Built:%s}", buildDateTime);
#if defined(_SILICON_LABS_32B_SERIES_2)
  uint32_t moduleName32[8] = {
    DEVINFO->MODULENAME0, DEVINFO->MODULENAME1, DEVINFO->MODULENAME2,
    DEVINFO->MODULENAME3, DEVINFO->MODULENAME4, DEVINFO->MODULENAME5,
    DEVINFO->MODULENAME6, 0UL
  };
  char *moduleName = (char *) moduleName32;
  for (uint8_t i = 0U; i < sizeof(moduleName32); i++) {
    if ((moduleName[i] == '\0') || (moduleName[i] == '\xFF')) {
      moduleName[i] = '\0';
      break;
    }
  }
  app_log_info("{radio}"
               "{FreqHz:%u}"
               "{ModuleInfo:0x%08x}"
               "{ModuleName:%s}\r\n",
                RAIL_GetRadioClockFreqHz(rftest_handle),
                DEVINFO->MODULEINFO,
                ((moduleName[0] == '\0') ? "N/A" : moduleName));
#else
  responsePrint("radio", "FreqHz:%u,ModuleInfo:0x%08x",
                RAIL_GetRadioClockFreqHz(railHandle),
                DEVINFO->MODULEINFO);
#endif
  SYSTEM_ChipRevision_TypeDef chipRev = { 0, };
  SYSTEM_ChipRevisionGet(&chipRev);

  // SYSTEM_ChipRevision_TypeDef defines either a partNumber field or a family field.
  // If _SYSCFG_CHIPREV_PARTNUMBER_MASK is defined, there is a partNumber field.
  // Otherwise, there is a family field.
  app_log_info("{system}"
              "{Family:%s}"
#if defined(_SYSCFG_CHIPREV_PARTNUMBER_MASK)
                "Part#:%u,"
#else
                "{Fam#:%u}"
#endif
                "{ChipRev:%u.%u}"
                "{sdid:%u}"
                "{Part:0x%08x}\r\n",
                FAMILY_NAME,
#if defined(_SYSCFG_CHIPREV_PARTNUMBER_MASK)
                chipRev.partNumber,
#else
                chipRev.family,
#endif
                chipRev.major,
                chipRev.minor,
                _SILICON_LABS_GECKO_INTERNAL_SDID,
                DEVINFO->PART);

}

/******************************************************************************
 * Register the RAIL event, rftest event and user application's is separate.
 *****************************************************************************/
void rail_event_callback_register(sl_rail_util_on_event_callback_t func)
{
  sl_rail_util_on_event_callback = func;
}

/******************************************************************************
 * RAIL Event callback process, this function replace the sl_rail_util_on_event()
 * and call it in sli_rail_util_on_event()
 *****************************************************************************/
void rail_event_callback_process(RAIL_Handle_t rail_handle,
                                 RAIL_Events_t events)
{
  if (sl_rail_util_on_event_callback != NULL) {
    sl_rail_util_on_event_callback(rail_handle, events);
  }
}

/******************************************************************************
 * Reset status counters
 *****************************************************************************/
void rftest_reset_counters(void)
{
  memset(&counters, 0, sizeof(counters_t));
}

/******************************************************************************
 * Print packet hex dump
 *****************************************************************************/
void rftest_print_tx_packet(void)
{
  // print out the transmit payload
  app_log_info("{Packet:");
  app_log_hexdump_info(tx_data, tx_length);
  app_log_info("}\r\n");
}

/******************************************************************************
 * set payload value
 *****************************************************************************/
void rftest_set_tx_paylaod(uint8_t index, uint8_t value)
{
  tx_data[index] = value;
}

/******************************************************************************
 * BER test, count the error bits.
 *****************************************************************************/
// count number of 1s in a byte without a loop
static uint8_t countBits(uint8_t num)
{
  uint8_t count = 0;
  static const uint8_t nibblebits[] = { 0, 1, 1, 2, 1, 2, 2, 3,\
                                        1, 2, 2, 3, 2, 3, 3, 4 };
  count += nibblebits[num & 0x0F];
  count += nibblebits[num >> 4];
  return count;
}

/******************************************************************************
 * BER FIFO mode, almost full event process
 *****************************************************************************/
static void berSource_RxFifoAlmostFull(uint16_t bytesAvailable)
{
  (void)bytesAvailable;
  // All incoming bytes are received and validated here.
  uint16_t numBytes;
  bool stopBerRx = false;

  // If rxOfEvent is > 0, then we're overflowing the incoming RX buffer
  // probably because the BER test isn't processing incoming bits fast
  // enough. The test will automatically try to re-synchronize and read in bits
  // from the stream, but the bits under test will not be continuous. Abort
  // testing and notify the user if this is the case.
  if (radio_flags.rx_of) {
    stopBerRx = true;
  }

  while ((RAIL_GetRxFifoBytesAvailable(rftest_handle)
          > RAIL_GetRxFifoThreshold(rftest_handle))
         && !stopBerRx) {
    // Read multiple bytes in if they're available.
    // Reuse the txData[SL_RAIL_TEST_MAX_PACKET_LENGTH] array since we won't be
    // transmitting in BER Test mode anyway.
    numBytes = RAIL_ReadRxFifo(rftest_handle, rx_data, RFTEST_MAX_PACKET_LENGTH);

    for (uint16_t x = 0; x < numBytes && !stopBerRx; x++) {
      // Update BER statistics
      if (berStats.bytesTested < berStats.bytesTotal) {
        // Counters will not overflow since bytesTotal max value is capped.
        berStats.bitErrors += countBits(rx_data[x]);
        berStats.bytesTested++;
      } else {
        stopBerRx = true; // statistics are all gathered - stop now
      }
    }
  }
  // disregard decimal point
  berStats.rssi = (int8_t)(RAIL_GetRssiAlt(rftest_handle,
                                           RAIL_GET_RSSI_WAIT_WITHOUT_TIMEOUT) / 4);

  // stop RXing when enough bits are acquired or an error (i.e. RX overflow)
  if (stopBerRx) {
    RAIL_Idle(rftest_handle, RAIL_IDLE_FORCE_SHUTDOWN, true);
    RAIL_ResetFifo(rftest_handle, true, true);
    radio_flags.ber_enabled = false;
  }
}

/******************************************************************************
 * BER FIFO mode, almost full event process
 *****************************************************************************/
void RAILCb_RxFifoAlmostFull(RAIL_Handle_t railHandle)
{
  uint16_t bytesAvailable = RAIL_GetRxFifoBytesAvailable(railHandle);

  if (radio_flags.ber_enabled) {
    berSource_RxFifoAlmostFull(bytesAvailable);
  }
}

/******************************************************************************
 * PER test, update the statistics
 *****************************************************************************/
void per_update_stats(int32_t newValue, Stats_t *stats)
{
  stats->samples++;
  if (stats->samples == 1) {
    stats->min = newValue;
    stats->max = newValue;
    stats->mean = newValue;
    stats->varianceTimesSamples = 0;
  } else {
    stats->min = (newValue < stats->min) ? newValue : stats->min;
    stats->max = (newValue > stats->max) ? newValue : stats->max;

    float delta = newValue - stats->mean;
    stats->mean += delta / stats->samples;
    // wikipedia.org/wiki/Algorithms_for_calculating_variance#On-line_algorithm
    // Update by adding (newValue - oldMean) * (newValue - newMean)
    stats->varianceTimesSamples += delta * (newValue - stats->mean);
  }
}

/******************************************************************************
 * PER variance
 *****************************************************************************/
float per_variance(const Stats_t stats)
{
  return stats.varianceTimesSamples / (stats.samples - 1);
}

/******************************************************************************
 * Set the rftest radio states.
 *****************************************************************************/
void rftest_set_state(rftest_status_t state)
{
  radio_state = state;
}

/******************************************************************************
 * Get current radio state of rftest
 *****************************************************************************/
rftest_status_t rftest_get_state(void)
{
  return radio_state;
}

/******************************************************************************
 * rftest configuration, set the packet length and packet structure.
 *****************************************************************************/
void rftest_configure(RAIL_Handle_t railHandle,
                      bool is_fixed_length,
                      uint8_t length)
{
  if (is_fixed_length) {
    // Fixed length
    tx_length = length;
    RAIL_SetFixedLength(railHandle, length);
  }else {
    // Variable length, Assume the length do not include CRC
    tx_length = length;
    tx_data[0] = length;
  }

  // Add the RF test command to group
  rftest_add_cli_cmd_group();
}

/******************************************************************************
 * rftest application main loop
 *****************************************************************************/
void rftest_app_main(void)
{
  // RAIL Rx packet handles
  RAIL_RxPacketHandle_t packet_handle;
  RAIL_RxPacketInfo_t packet_info;
  RAIL_RxPacketDetails_t details;
  RAIL_Status_t rail_status;
  // RF test is not enabled.
  if (!radio_flags.rftest_enabled) {
      return;
  }

  if (radio_flags.packet_tx) {
    radio_state = RFTEST_PACKET_SENT;
    radio_flags.packet_tx = false;
  }

  if (radio_flags.packet_rx) {
    radio_state = RFTEST_PACKET_RCEIVED;
    radio_flags.packet_rx = false;
  }
  // PER test is end
  if (radio_flags.per_end) {
    radio_flags.per_end = false;
    app_log_info("{PerEnd}\r\n");
 }

  switch (radio_state) {
    case RFTEST_IDLE:
      if (radio_flags.tx_request) {
        radio_flags.tx_request = false;
        rftest_tx_timer_callback(rftest_handle);
      }
      break;
    case RFTEST_RX:

      break;
    case RFTEST_PACKET_RCEIVED:
      // Received one packet
      packet_handle = RAIL_GetRxPacketInfo(rftest_handle,
                                           RAIL_RX_PACKET_HANDLE_OLDEST_COMPLETE,
                                           &packet_info);
      if (packet_handle != RAIL_RX_PACKET_HANDLE_INVALID) {
        rail_status = RAIL_GetRxPacketDetailsAlt(rftest_handle,
                                                 packet_handle,
                                                 &details);
        RAIL_CopyRxPacket(rx_data, &packet_info);
        rail_status = RAIL_ReleaseRxPacket(rftest_handle, packet_handle);
        if (rail_status != RAIL_STATUS_NO_ERROR) {
          app_log_warning("RAIL_ReleaseRxPacket() result:%d", rail_status);
        }

        if (packet_info.packetStatus == RAIL_RX_PACKET_READY_CRC_ERROR) {
          counters.crc_error++;
        }
        // PER test
        per_update_stats(details.rssi, &counters.rssi);

        app_log_info("{%s}{len:%d}{timeUs:%u}{timePos:%u}{crc:%s}"
                    "{rssi:%d}{lqi:%d}{phy:%d}{isAck:%s}{syncWordId:%d}"
                    "{antenna:%d}{channelHopIdx:%d}{rxFreqOffset:%u}",
                    "rxPacket",
                    packet_info.packetBytes,
                    details.timeReceived.packetTime,
                    details.timeReceived.timePosition,
                    details.crcPassed ? "Pass" : "Fail",
                    details.rssi,
                    details.lqi,
                    details.subPhyId,
                    details.isAck ? "True": "False",
                    details.syncWordId,
                    details.channelHoppingChannelIndex,
                    rxFreqOffset);

        app_log_info("{payload: ");
        app_log_hexdump_info(rx_data, packet_info.packetBytes);
        app_log_info("}\r\n");
      }
      radio_state = RFTEST_RX;
      break;
    case RFTEST_TX_CONTINUOUS:
      break;
    case RFTEST_PACKET_SENT:
      rail_status = RAIL_SetTimer(rftest_handle,
                                  TX_INTETVAL_DEFAULT,
                                  RAIL_TIME_DELAY,
                                  rftest_tx_timer_callback);
      if (rail_status != RAIL_STATUS_NO_ERROR) {
        app_log_warning("RAIL_SetTimer() result:%d\r\n", rail_status);
      }
      radio_state = RFTEST_IDLE;
      break;
    case RFTEST_BER_TEST:
      break;
    case RFTEST_PER_TEST:
      break;
    default:
      break;
  }
}

/******************************************************************************
 * rftest send multiply packets callback
 *****************************************************************************/
void rftest_tx_timer_callback(RAIL_Handle_t arg)
{
  RAIL_Status_t rail_status;
  (void)arg;

  if ((tx_count > 0) || (radio_flags.infinite_tx)) {
    tx_count--;
    uint16_t bytes_num = 0;
    tx_data[0] = tx_length;
    bytes_num = RAIL_WriteTxFifo(rftest_handle,
                                tx_data,
                                tx_length,
                                true);
    if (bytes_num != tx_length) {
      app_log_warning("RAIL_WriteTxFifo() failed:%d\r\n", bytes_num);
    }

    rail_status = RAIL_StartTx(rftest_handle,
                               rf_channel,
                               txOptions,
                               NULL);
    if (rail_status != RAIL_STATUS_NO_ERROR) {
      app_log_warning("RAIL_StartTx() result:%d ", rail_status);
    }

  }else {
    // TX mode is packet mode, transmission is completed
    radio_flags.tx_request = false;
    app_log_info("{Tx end}{Transmited:%u}{failed:%d}{Time:%u}\r\n",
                  counters.packet_sent,
                  counters.tx_error,
                  RAIL_GetTime());
  }
}

/******************************************************************************
 * rftest RAIL event callback
 *****************************************************************************/
void rfest_util_on_event(RAIL_Handle_t rail_handle, RAIL_Events_t events)
{
  RAIL_Status_t status;

  // Handle Rx events
  if ( events & RAIL_EVENTS_RX_COMPLETION ) {
    if (events & RAIL_EVENT_RX_PACKET_RECEIVED) {
      // Keep the packet in the radio buffer, download it later at the state machine
      RAIL_HoldRxPacket(rail_handle);
      radio_flags.packet_rx = true;
      counters.packet_recevied++;
    } else {
      // Handle Rx error
        radio_flags.rx_error = true;
        counters.rx_error++;
        if (events & RAIL_EVENT_RX_FIFO_OVERFLOW) {
          radio_flags.rx_of = true;
          counters.rx_overflow++;
        }
        if (events & RAIL_EVENT_RX_PACKET_ABORTED) {
          counters.rx_abort++;
        }
        if (events & RAIL_EVENT_RX_FRAME_ERROR) {
          counters.frame_error++;
        }
    }
  }

  // Handle Tx events
  if ( events & RAIL_EVENTS_TX_COMPLETION) {
    if (events & RAIL_EVENT_TX_PACKET_SENT) {
      radio_flags.packet_tx = true;
      counters.packet_sent++;
    } else {
      // Handle Tx error
      counters.tx_error++;
      if (events & RAIL_EVENT_TX_ABORTED) {
        counters.tx_abort++;
      }
      if (events & RAIL_EVENT_TX_BLOCKED) {
        counters.tx_block++;
      }
      if (events & RAIL_EVENT_TX_UNDERFLOW) {
        counters.tx_underflow++;
      }
    }
  }

  if (events & RAIL_EVENT_RX_FIFO_ALMOST_FULL) {
    // BER test
    RAILCb_RxFifoAlmostFull(rail_handle);
  }

  // RX Events
   if (events & RAIL_EVENT_RX_TIMING_DETECT) {
     counters.timing_detect++;
   }
   if (events & RAIL_EVENT_RX_TIMING_LOST) {
     counters.timing_lost++;
   }
   if (events & RAIL_EVENT_RX_PREAMBLE_LOST) {
     counters.preamble_lost++;
   }
   if (events & RAIL_EVENT_RX_PREAMBLE_DETECT) {
     counters.preamble_detect++;
   }
   if (events & (RAIL_EVENT_RX_SYNC1_DETECT | RAIL_EVENT_RX_SYNC2_DETECT)) {
     counters.sync_detect++;
     rxFreqOffset = RAIL_GetRxFreqOffset(rail_handle);
   }

  // Perform all calibrations when needed
  if ( events & RAIL_EVENT_CAL_NEEDED ) {
    status = RAIL_Calibrate(rail_handle, NULL, RAIL_CAL_ALL_PENDING);
    if (status != RAIL_STATUS_NO_ERROR) {
      radio_flags.cal_error = true;
      counters.cal_error++;
    }
  }
}
