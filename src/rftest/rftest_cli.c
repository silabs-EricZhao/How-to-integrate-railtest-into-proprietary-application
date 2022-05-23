/***************************************************************************//**
 * @file
 * @brief rftest_cli.c
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
#include "rftest_common.h"
#include "rftest_app.h"
// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------

int sprintfFloat(char *buffer, int8_t len, float f, uint8_t precision);
// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
const char buildDateTime[] = __DATE__ " " __TIME__;

// Variables for BER and PER testing
static uint32_t ber_bytes;
static uint32_t per_count;
static uint32_t per_delay;
// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------
static const char *config_index_name[] = {
    "RFTEST BER PHY",
    "User application PHY"
};
static const char *powerModes[] = RAIL_TX_POWER_MODE_NAMES;
// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * CLI - Enable RF test mode
 *****************************************************************************/
void rftest_cli_enable(sl_cli_command_arg_t *args)
{
  // Switch to rftest packet PHY
  uint8_t rftest_enable;

  rftest_enable = sl_cli_get_argument_uint8(args, 0);
  if (rftest_enable) {
  // Re-configure the RAIL EVENT and data configuration.
  rftest_init();
  app_log_info("{%s}{Enable RF test and switch to %s}\r\n",
               sl_cli_get_command_string(args, 0),
               config_index_name[config_index]);
  }else {
    // Disable rftest function, reset system to restore the user settings.
    app_log_info("{%s}{%s}\r\n",
                  sl_cli_get_command_string(args, 0),
                  "Disable RF test and reboot system...");
    // Delay 20ms to wait print is completed.
    RAIL_DelayUs(20000);
    // Use the NVIC to reset the chip
    NVIC_SystemReset();
  }
}

/******************************************************************************
 * Get RAIL library version
 *****************************************************************************/
void rftest_cli_get_rail_version(sl_cli_command_arg_t *args)
{
  bool verbose = false;
  RAIL_Version_t rail_ver;
  if (sl_cli_get_argument_count(args) >= 1) {
    verbose = sl_cli_get_argument_uint8(args, 0);
  }

  RAIL_GetVersion(&rail_ver, verbose);
  if (verbose) {
    app_log_info("{%s}"
                  "{RAIL:%d.%d.%d.%d,}"
                  "{hash:0x%.8X,}"
                  "{flags:0x%.2X}",
                  sl_cli_get_command_string(args, 1),
                  rail_ver.major,
                  rail_ver.minor,
                  rail_ver.rev,
                  rail_ver.build,
                  rail_ver.hash,
                  rail_ver.flags);
  } else {
      app_log_info("{%s}"
                   "{RAIL:%d.%d.%d}",
                   sl_cli_get_command_string(args, 1),
                   rail_ver.major,
                   rail_ver.minor,
                   rail_ver.rev);
  }
  app_log_info("{Multiprotocol:%s}"
               "{Built:%s}\r\n",
               rail_ver.multiprotocol ? "True" : "False",
               buildDateTime);


}

/******************************************************************************
 * Check whether radio is idle.
 *****************************************************************************/
bool rftest_radio_is_idle(void)
{
  if ((RAIL_RF_STATE_IDLE == RAIL_GetRadioState(rftest_handle))
   && (RFTEST_IDLE == rftest_get_state())) {
    return true;
  }else {
    return false;
  }
}

/******************************************************************************
 * Print warning info if trying to configurate radio if state is not idle.
 *****************************************************************************/
void rftest_error_state_print(void)
{
  app_log_error("Need to be in Idle radio state for this command\r\n");
}

void rftest_cli_get_data_rate(sl_cli_command_arg_t *args)
{
  app_log_info("{%s}"
               "{Symbolrate:%d,Bitrate:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               RAIL_GetSymbolRate(rftest_handle),
               RAIL_GetBitRate(rftest_handle));
}

/******************************************************************************
 * Print the Antenna info
 *****************************************************************************/
const char *configuredRxAntenna(RAIL_RxOptions_t rxOptions)
{
  switch (rxOptions & (RAIL_RX_OPTION_ANTENNA_AUTO)) {
    case (RAIL_RX_OPTION_ANTENNA_AUTO): {
      return "Auto";
      break;
    }
    case (RAIL_RX_OPTION_ANTENNA0): {
      return "Antenna0";
      break;
    }
    case (RAIL_RX_OPTION_ANTENNA1): {
      return "Antenna1";
      break;
    }
    default: {
      return "Any";
      break;
    }
  }
}

/******************************************************************************
 * CLI - Set RX options
 *****************************************************************************/
void rftest_cli_set_RxOptions(sl_cli_command_arg_t *args)
{
  // Only update the rxOptions if a parameter is given otherwise just print the
  // current settings
  if (sl_cli_get_argument_count(args) >= 1) {
    RAIL_RxOptions_t newRxOptions = sl_cli_get_argument_uint32(args, 0);
    RAIL_Status_t status = RAIL_ConfigRxOptions(rftest_handle,
                                                RAIL_RX_OPTIONS_ALL,
                                                newRxOptions);

    // Make sure there was no error setting the new options
    if (status != RAIL_STATUS_NO_ERROR) {
      app_log_error("{%s}{error:%d}{%s}\r\n",
                    sl_cli_get_command_string(args, 1),
                    31,
                    "RxOptions:Failed");
      return;
    }
    // Update the global rxOptions
    rxOptions = newRxOptions;
  }

  app_log_info("{%s}"
         "{storeCrc:%s}{ignoreCrcErrors:%s}{enableDualSync:%s}"
         "{trackAborted:%s}{removeAppendedInfo:%s}{rxAntenna:%s}"
         "{frameDet:%s}\r\n",
          sl_cli_get_command_string(args, 1),
          (rxOptions & RAIL_RX_OPTION_STORE_CRC) ? "True" : "False",
          (rxOptions & RAIL_RX_OPTION_IGNORE_CRC_ERRORS) ? "True" : "False",
          (rxOptions & RAIL_RX_OPTION_ENABLE_DUALSYNC) ? "True" : "False",
          (rxOptions & RAIL_RX_OPTION_TRACK_ABORTED_FRAMES) ? "True" : "False",
          (rxOptions & RAIL_RX_OPTION_REMOVE_APPENDED_INFO) ? "True" : "False",
           configuredRxAntenna(rxOptions),
          (rxOptions & RAIL_RX_OPTION_DISABLE_FRAME_DETECTION) ? "Off" : "On");
}

/******************************************************************************
 * CLI - Enable RX or disable
 *****************************************************************************/
void rftest_cli_rx(sl_cli_command_arg_t *args)
{
  uint8_t rx_enable;
  RAIL_Status_t status;

  rx_enable = sl_cli_get_argument_uint8(args, 0);
  if (rx_enable) {
    // start RX
    status = RAIL_StartRx(rftest_handle, rf_channel, NULL);
    if (status != RAIL_STATUS_NO_ERROR) {
      app_log_error("Enable RX failed, Error:%d\r\n", status);
    }
  }else {
    // Tranfer to radio IDLE state
    RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);
    RAIL_ResetFifo(rftest_handle, true, true);
    rftest_set_state(RFTEST_IDLE);
  }

  app_log_info("{%s}"
               "{ENABLE:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               rx_enable);
}

/******************************************************************************
 * CLI - Output continuous modulated signal or disable.
 *****************************************************************************/
void rftest_cli_tx_stream(sl_cli_command_arg_t *args)
{
  uint8_t enable;

  enable = sl_cli_get_argument_uint8(args, 0);
  if (enable) {
    if (!rftest_radio_is_idle()) {
      rftest_error_state_print();
      return;
    }
    RAIL_StartTxStream(rftest_handle, rf_channel, RAIL_STREAM_PN9_STREAM);
    rftest_set_state(RFTEST_TX_CONTINUOUS);
  }else {
    // set radio to IDLE state and reset the FIFO
    RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);
    RAIL_ResetFifo(rftest_handle, true, true);
    rftest_set_state(RFTEST_IDLE);
  }

  app_log_info("{%s}"
               "{ENABLE:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               enable);
}

/******************************************************************************
 * CLI - Output the tone signal or disable.
 *****************************************************************************/
void rftest_cli_tx_tone(sl_cli_command_arg_t *args)
{
  uint8_t enable;

  enable = sl_cli_get_argument_uint8(args, 0);
  if (enable) {
    if (!rftest_radio_is_idle()) {
      rftest_error_state_print();
      return;
    }
    RAIL_StartTxStream(rftest_handle, rf_channel, RAIL_STREAM_CARRIER_WAVE);
    rftest_set_state(RFTEST_TX_CONTINUOUS);
  }else {
   // set radio to IDLE state and reset the FIFO
   RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);
   RAIL_ResetFifo(rftest_handle, true, true);
   rftest_set_state(RFTEST_IDLE);
  }

   app_log_info("{%s}"
                "{ENABLE:%d}\r\n",
                sl_cli_get_command_string(args, 1),
                enable);
}

/******************************************************************************
 * CLI - Get current power settings
 *****************************************************************************/
void rftest_cli_get_power(sl_cli_command_arg_t *args)
{
  app_log_info("{%s}"
               "{powerLevel:%d}"
               "{power:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               RAIL_GetTxPower(rftest_handle),
               RAIL_GetTxPowerDbm(rftest_handle));
}

/******************************************************************************
 * CLI - Get the power limits
 *****************************************************************************/
void rftest_cli_get_power_limits(sl_cli_command_arg_t *args)
{
  RAIL_TxPowerMode_t powerMode = RAIL_TX_POWER_MODE_NONE;
  RAIL_Status_t status = RAIL_STATUS_NO_ERROR;

  if (sl_cli_get_argument_count(args) >= 1) {
    powerMode = sl_cli_get_argument_uint8(args, 0);
  } else {
    RAIL_TxPowerConfig_t config;
    status = RAIL_GetTxPowerConfig(rftest_handle, &config);
    powerMode = config.mode;
  }
  if (powerMode >= RAIL_TX_POWER_MODE_NONE || status != RAIL_STATUS_NO_ERROR ) {
    app_log_info("{%s}{Invalid PA enum value selected: %d}\r\n",
                 sl_cli_get_command_string(args, 1),
                 powerMode);
    return;
  }
  RAIL_TxPowerLevel_t maxPowerlevel = RAIL_TX_POWER_LEVEL_INVALID;
  RAIL_TxPowerLevel_t minPowerlevel = RAIL_TX_POWER_LEVEL_INVALID;
  bool success = RAIL_SupportsTxPowerModeAlt(rftest_handle,
                                             &powerMode,
                                             &maxPowerlevel,
                                             &minPowerlevel);
  app_log_info("{%s}"
              "{success:%s}"
              "{powerMode:%s}"
              "{minPowerLevel:%d}"
              "{maxPowerLevel:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               success ? "Success" : "Failure",
               powerModes[powerMode],
               minPowerlevel,
               maxPowerlevel);
}

/******************************************************************************
 * CLI - Set tx power, 100 represent 10dBm
 *****************************************************************************/
void rftest_cli_set_tx_power(sl_cli_command_arg_t *args)
{
  bool setPowerError = false;

  if (!rftest_radio_is_idle()) {
    rftest_error_state_print();
    return;
  }

  if (sl_cli_get_argument_count(args) >= 2
      && strcmp(sl_cli_get_argument_string(args, 1), "raw") == 0) {

    RAIL_TxPowerLevel_t rawLevel = sl_cli_get_argument_uint8(args, 0);
    // Set the power and update the RAW level global
    if (RAIL_SetTxPower(rftest_handle, rawLevel) != RAIL_STATUS_NO_ERROR) {
      setPowerError = true;
    }
  } else {
    RAIL_TxPowerConfig_t tempCfg;
    RAIL_TxPower_t powerDbm = sl_cli_get_argument_int16(args, 0);

    // Set the power in dBm and figure out what RAW level to store based on what
    // was requested NOT what is actually applied to the hardware after limits.
    if ((RAIL_SetTxPowerDbm(rftest_handle, powerDbm)
         != RAIL_STATUS_NO_ERROR)
        || (RAIL_GetTxPowerConfig(rftest_handle, &tempCfg)
            != RAIL_STATUS_NO_ERROR)) {
      setPowerError = true;
    }
  }

  if (setPowerError) {
    app_log_error("Set power error!\r\n");
  }else {
    rftest_cli_get_power(args);
  }
}

/******************************************************************************
 * CLI - Get current channel
 *****************************************************************************/
void rftest_cli_get_channel(sl_cli_command_arg_t *args)
{
  app_log_info("{%s}"
               "{channel:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               rf_channel);
}

/******************************************************************************
 * CLI - Set channel
 *****************************************************************************/
void rftest_cli_set_channel(sl_cli_command_arg_t *args)
{
  uint16_t proposedChannel = sl_cli_get_argument_uint16(args, 0);
   bool success = false;

  if (!rftest_radio_is_idle()) {
    rftest_error_state_print();
    return;
  }

  // Make sure this is a valid channel
  if (RAIL_IsValidChannel(rftest_handle, proposedChannel)
      == RAIL_STATUS_NO_ERROR) {
    rf_channel = proposedChannel;
    RAIL_StartRx(rftest_handle, proposedChannel, NULL);
    success = true;
  }

  if (!success) {
    app_log_error("Invalid channel %d", proposedChannel);
    return;
  }

  rftest_cli_get_channel(args);
}

/******************************************************************************
 * CLI - Reboot the chip
 *****************************************************************************/
void rftest_cli_reset_chip(sl_cli_command_arg_t *args)
{
  (void)args;
  app_log_info("{%s}{%s}\r\n",
               sl_cli_get_command_string(args, 1),
               "Reboot system...");

  RAIL_DelayUs(20000);
  // Use the NVIC to reset the chip
  NVIC_SystemReset();
}

/******************************************************************************
 * CLI - Get Ctune value
 *****************************************************************************/
void rftest_cli_get_ctune(sl_cli_command_arg_t *args)
{
  uint32_t ctune = RAIL_GetTune(rftest_handle);

#ifdef _SILICON_LABS_32B_SERIES_1
  app_log_info("{%s}{CTUNE:0x%.3x}\r\n",
               sl_cli_get_command_string(args, 1),
               ctune);
#else
  app_log_info("{%s}"
               "{CTUNEXIANA:0x%.3x}"
               "{CTUNEXOANA:0x%.3x}\r\n",
               sl_cli_get_command_string(args, 1),
               ctune,
               (ctune + RAIL_GetTuneDelta(rftest_handle)));
#endif
}

/******************************************************************************
 * CLI - Set Ctune value
 *****************************************************************************/
void rftest_cli_set_ctune(sl_cli_command_arg_t *args)
{
  if (!rftest_radio_is_idle()) {
   rftest_error_state_print();
   return;
  }

  RAIL_SetTune(rftest_handle, sl_cli_get_argument_uint32(args, 0));

  // Read out and print the current CTUNE value
  args->argc = sl_cli_get_command_count(args);
  rftest_cli_get_ctune(args);
}

/******************************************************************************
 * CLI - Get Ctune delta value
 *****************************************************************************/
void rftest_cli_get_ctune_delta(sl_cli_command_arg_t *args)
{
  int32_t delta = RAIL_GetTuneDelta(rftest_handle);
  app_log_info("{%s}"
               "{CTuneDelta:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               delta);
}

/******************************************************************************
 * CLI - Set Ctune value delta, the delta value is 0 by default.
 *****************************************************************************/
void rftest_cli_set_ctune_delta(sl_cli_command_arg_t *args)
{
  if (!rftest_radio_is_idle()) {
    rftest_error_state_print();
    return;
  }

  RAIL_SetTuneDelta(rftest_handle, sl_cli_get_argument_uint32(args, 0));
   // Read out and print the current CTUNE delta value
  rftest_cli_get_ctune_delta(args);
}

/******************************************************************************
 * CLI - Get the Tx antenna name
 *****************************************************************************/
static const char *configuredTxAntenna(RAIL_TxOptions_t txOptions)
{
  switch (txOptions & (RAIL_TX_OPTION_ANTENNA0 | RAIL_TX_OPTION_ANTENNA1)) {
    case (RAIL_TX_OPTION_ANTENNA0 | RAIL_TX_OPTION_ANTENNA1): {
      return "Any";
      break;
    }
    case (RAIL_TX_OPTION_ANTENNA0): {
      return "Antenna0";
      break;
    }
    case (RAIL_TX_OPTION_ANTENNA1): {
      return "Antenna1";
      break;
    }
    default: {
      return "Any";
      break;
    }
  }
}

/******************************************************************************
 * CLI - configure TX options
 *****************************************************************************/
void rftest_cli_config_TxOptions(sl_cli_command_arg_t *args)
{
  if (sl_cli_get_argument_count(args) >= 1) {
    txOptions = sl_cli_get_argument_uint32(args, 0);
  }

  app_log_info("{%s}"
            "{waitForAck:%s}{removeCrc:%s}{syncWordId:%d}"
            "{txAntenna:%s}{altPreambleLen:%s}{ccaPeakRssi:%s}"
            "{ccaOnly:%s}{resend:%s}\r\n",
            sl_cli_get_command_string(args, 1),
            ((txOptions & RAIL_TX_OPTION_WAIT_FOR_ACK) ? "True" : "False"),
            ((txOptions & RAIL_TX_OPTION_REMOVE_CRC) ? "True" : "False"),
            ((txOptions & RAIL_TX_OPTION_SYNC_WORD_ID)\
                >> RAIL_TX_OPTION_SYNC_WORD_ID_SHIFT),
            configuredTxAntenna(txOptions),
            ((txOptions & RAIL_TX_OPTION_ALT_PREAMBLE_LEN) ? "True" : "False"),
            ((txOptions & RAIL_TX_OPTION_CCA_PEAK_RSSI) ? "True" : "False"),
            ((txOptions & RAIL_TX_OPTION_CCA_ONLY) ? "True" : "False"),
            ((txOptions & RAIL_TX_OPTION_RESEND) ? "True" : "False"));
}

/******************************************************************************
 * CLI - Transmit packet
 *****************************************************************************/
void rftest_cli_tx(sl_cli_command_arg_t *args)
{
  tx_count = sl_cli_get_argument_uint32(args, 0);
  // command tx 0 will continuously transmit packet
  if (tx_count == 0) {
    radio_flags.infinite_tx = true;
  }else {
    radio_flags.infinite_tx = false;
  }
  radio_flags.tx_request = true;
  rftest_set_state(RFTEST_IDLE);
  app_log_info("{%s}{TX mode:%s}{Time:%u}\r\n",
               sl_cli_get_command_string(args, 1),
               (radio_flags.infinite_tx) ? "Infinite TX" : "Packet mode",
               RAIL_GetTime());
}

/******************************************************************************
 * CLI - Set to specific fixed length
 *****************************************************************************/
void rftest_cli_set_fixed_length(sl_cli_command_arg_t *args)
{
  if (!rftest_radio_is_idle()) {
    rftest_error_state_print();
    return;
  }
  uint16_t fixedLength = sl_cli_get_argument_uint16(args, 0);
  fixedLength = RAIL_SetFixedLength(rftest_handle, fixedLength);

  // Print configured length
  app_log_info("{%s}{FixedLength:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               fixedLength);
}

/******************************************************************************
 * CLI - Set the packet length
 *****************************************************************************/
void rftest_cli_set_tx_length(sl_cli_command_arg_t *args)
{
  uint16_t fixedLength;
  uint32_t length = sl_cli_get_argument_uint32(args, 0);

  if (length > RFTEST_MAX_PACKET_LENGTH) {
    app_log_info("{%s}{Invalid length %d}{MaxLength:%d}\r\n",
                 sl_cli_get_command_string(args, 1),
                 length,
                 RFTEST_MAX_PACKET_LENGTH);
    return;
  }

  tx_length = length;
  fixedLength = RAIL_SetFixedLength(rftest_handle, tx_length);
  app_log_info("{%s}{TxLength %d}{Written:%d}\r\n",
                   sl_cli_get_command_string(args, 1),
                   tx_length,
                   fixedLength);
}

/******************************************************************************
 * CLI - Print the packet content
 *****************************************************************************/
void rftest_cli_print_tx_payload(sl_cli_command_arg_t *args)
{
  app_log_info("{%s}", sl_cli_get_command_string(args, 1));
  rftest_print_tx_packet();
}

/******************************************************************************
 * CLI - Set the packet content
 *****************************************************************************/
void rftest_cli_set_tx_payload(sl_cli_command_arg_t *args)
{
  uint16_t offset = sl_cli_get_argument_uint16(args, 0);

  // Read as many bytes as have been supplied and set them
  for (int i = 2; i < sl_cli_get_argument_count(args) + 1; i++) {
    uint32_t index = offset + i - 2;
    uint8_t value = sl_cli_get_argument_uint8(args, i - 1);

    // Make sure this fits in the txData buffer
    if (index >= RFTEST_MAX_PACKET_LENGTH) {
      app_log_info("{%s}{%s}\r\n",
                   sl_cli_get_command_string(args, 1),
                   "Data overflows txData buffer");
      return;
    }
    rftest_set_tx_paylaod(index, value);
  }
  // Print the Tx payload
  rftest_cli_print_tx_payload(args);
}

/******************************************************************************
 * BER test, reset the statistic counters
 *****************************************************************************/
void berResetStats(uint32_t numBytes)
{
  // Reset test statistics
  memset(&berStats, 0, sizeof(BerStatus_t));

  // 0x1FFFFFFF bytes (0xFFFFFFF8 bits) is max number of bytes that can be
  // tested without uint32_t math rollover; numBytes = 0 is same as max
  if ((0 == numBytes) || (numBytes > 0x1FFFFFFF)) {
    numBytes = 0x1FFFFFFF;
  }
  berStats.bytesTotal = numBytes;
}

/******************************************************************************
 * CLI - BER test, configure how many bytes should be test
 *****************************************************************************/
void rftest_cli_config_ber(sl_cli_command_arg_t *args)
{
  RAIL_Status_t status;
  uint16_t rxThreshold, packetLength;

  RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(rftest_handle, true, true);

  // Switch to BER test PHY
  config_index = RFTEST_BERTEST_INDEX;
  rf_channel = RAIL_ConfigChannels(rftest_handle,
                                   channelConfigs[config_index],
                                   &sli_rail_util_on_channel_config_change);

  RAIL_DataConfig_t railDataConfig;
  // configure radio
  railDataConfig.txSource = TX_PACKET_DATA;
  railDataConfig.rxSource = RX_PACKET_DATA;
  railDataConfig.txMethod = FIFO_MODE;
  railDataConfig.rxMethod = FIFO_MODE;
  status = RAIL_ConfigData(rftest_handle, &railDataConfig);
  if (status) {
    app_log_info("{%s}"
                 "{%s}"
                 "{status:%x}",
                 sl_cli_get_command_string(args, 1),
                 "Error calling RAIL_ConfigData().",
                 status);
  }

  // Configure RX FIFO
  rxThreshold = 100;
  rxThreshold = RAIL_SetRxFifoThreshold(rftest_handle, rxThreshold);
  // Configure Event
  RAIL_ConfigEvents(rftest_handle,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL,
                    RAIL_EVENT_RX_FIFO_ALMOST_FULL);

  // specify overall packet length info (infinite)
  packetLength = 0;
  packetLength = RAIL_SetFixedLength(rftest_handle, packetLength);

  RAIL_EnablePti(rftest_handle, false);

  ber_bytes = sl_cli_get_argument_uint32(args, 0);
  berResetStats(ber_bytes);

  app_log_info("{%s}"
               "{NumBytes:%d}\r\n",
               sl_cli_get_command_string(args, 1),
               ber_bytes);
}

/******************************************************************************
 * CLI - Start BER test
 *****************************************************************************/
void rftest_cli_ber_rx(sl_cli_command_arg_t *args)
{
  bool enable = !!sl_cli_get_argument_uint8(args, 0);


  args->argc = sl_cli_get_command_count(args); /* only reference cmd str */
//  resetCounters(args);

  RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);
  RAIL_ResetFifo(rftest_handle, true, true);
  if (enable) {
    RAIL_EnablePti(rftest_handle, false);
    berResetStats(ber_bytes);
    RAIL_StartRx(rftest_handle, rf_channel, NULL);
    rftest_set_state(RFTEST_BER_TEST);
    radio_flags.ber_enabled = true;
  }
  app_log_info("{%s}"
               "{BER:%s}\r\n",
               sl_cli_get_command_string(args, 1),
               enable ? "Enabled" : "Disabled");
}

/******************************************************************************
 * CLI - Get the result of BER test
 *****************************************************************************/
void rftest_cli_get_ber_status(sl_cli_command_arg_t *args)
{
  float percentDone;
  float percentBitError;
  uint32_t bytesTotal; /**< Number of bytes to receive */
  uint32_t bytesTested; /**< Number of bytes currently tested */
  uint32_t bitErrors; /**< Number of bits errors detected */
  int8_t rssi; /**< Current RSSI value during pattern acquisition */
  CORE_DECLARE_IRQ_STATE;

  // be sure we don't get half new, half stale data
  CORE_ENTER_CRITICAL();
  bytesTotal = berStats.bytesTotal;
  bytesTested = berStats.bytesTested;
  bitErrors = berStats.bitErrors;
  rssi = berStats.rssi;
  CORE_EXIT_CRITICAL();

  // don't divide by 0
  if (0 != bytesTotal) {
    percentDone = (float)((((double)bytesTested) / bytesTotal) * 100);
  } else {
    percentDone = 0.0;
  }
  // don't divide by 0
  if (0 != bytesTested) {
    percentBitError = (float)((((double)bitErrors) / (bytesTested * 8)) * 100);
  } else {
    percentBitError = 0.0;
  }

  // If rxOfEvent is > 0, then we're overflowing the incoming RX buffer
  // probably because the BER test isn't processing incoming bits fast
  // enough. The test will automatically try to re-synchronize and read in bits
  // from the stream, but the bits under test will not be continuous. Abort
  // testing and notify the user if this is the case.
  if (radio_flags.rx_of) {
    app_log_info("{%s}"
                "{BitsToTest:%u}"
                "{BitsTested:0}"
                "{PercentDone:0.00}"
                "{RSSI:%d}"
                "{BitErrors:0}"
                "{PercentBitError:0.00}"
                "{Status:TestAbortedRxOverflow}\r\n",
                 sl_cli_get_command_string(args, 1),
                 bytesTotal * 8,
                 rssi);

  } else {
    char bufPercentDone[10];
    char bufPercentBitError[10];

    sprintfFloat(bufPercentDone, sizeof(bufPercentDone),
                 percentDone, 2);
    sprintfFloat(bufPercentBitError, sizeof(bufPercentBitError),
                 percentBitError, 2);

    app_log_info("{%s}"
                 "{BitsToTest:%u}"
                 "{BitsTested:%u}"
                 "{PercentDone:%s}"
                 "{RSSI:%d}"
                 "{BitErrors:%u}"
                  "{PercentBitError:%s}\r\n",
                  sl_cli_get_command_string(args, 1),
                  (bytesTotal * 8),
                  (bytesTested * 8),
                  bufPercentDone,
                  rssi,
                  bitErrors,
                  bufPercentBitError);
  }
}

/******************************************************************************
 * Timer callback, PER trigger timer.
 *****************************************************************************/
void per_test_timer_callback(RAIL_Handle_t arg)
{
   (void)arg;
#if defined (SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
  GPIO_PinOutToggle(SL_RAIL_TEST_PER_PORT, SL_RAIL_TEST_PER_PIN);
  counters.per_triggers += GPIO_PinOutGet(SL_RAIL_TEST_PER_PORT,
                                          SL_RAIL_TEST_PER_PIN);
  per_count -= GPIO_PinOutGet(SL_RAIL_TEST_PER_PORT, SL_RAIL_TEST_PER_PIN);
#endif // SL_RAIL_TEST_PER_PORT && SL_RAIL_TEST_PER_PIN
  if (per_count < 1) {
#if defined (SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
    GPIO_PinOutClear(SL_RAIL_TEST_PER_PORT, SL_RAIL_TEST_PER_PIN);
    radio_flags.per_end = true;
#endif // SL_RAIL_TEST_PER_PORT && SL_RAIL_TEST_PER_PIN
  } else {
    RAIL_SetTimer(rftest_handle,
                  per_delay,
                  RAIL_TIME_DELAY,
                  &per_test_timer_callback);
  }
}

/******************************************************************************
 * CLI - Start PER test
 *****************************************************************************/
void rftest_cli_start_per(sl_cli_command_arg_t *args)
{
#if defined(SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
  uint32_t packets = sl_cli_get_argument_uint32(args, 0);
  uint32_t delayUs = sl_cli_get_argument_uint32(args, 1);
  per_count = packets;
  per_delay = delayUs / 2;
  // Reset counters
   rftest_reset_counters();
  if (packets > 0) {
    // Start the PER test
    radio_flags.per_end = false;
    RAIL_SetTimer(rftest_handle,
                  per_delay,
                  RAIL_TIME_DELAY,
                  &per_test_timer_callback);

  }else {
    // Stop the PER
    RAIL_CancelTimer(rftest_handle);
    GPIO_PinOutClear(SL_RAIL_TEST_PER_PORT, SL_RAIL_TEST_PER_PIN);
    radio_flags.per_end = false;
  }
  app_log_info("{%s}{PER:%s}{Packets:%u}{DelayMs:%u}\r\n",
               sl_cli_get_command_string(args, 1),
               (packets == 0) ? "DISABLE" : "ENABLE",
               packets,
               delayUs/1000);

#else
  app_log_warning("{%s}"
                    "{To run PER commands, SL_RAIL_TEST_PER_PORT "
                    "and SL_RAIL_TEST_PER_PIN must be configured for use.}\r\n",
                    sl_cli_get_command_string(args, 1));
#endif //defined(SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
}

/******************************************************************************
 * CLI - Get PER test
 *****************************************************************************/
void rftest_cli_get_per_stats(sl_cli_command_arg_t *args)
{
#if defined(SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
  char bufRssiMean[10];
  char bufRssiMin[10];
  char bufRssiMax[10];
  char bufRssiVariance[10];

  sprintfFloat(bufRssiMean, sizeof(bufRssiMean),
               counters.rssi.mean / 4, 0);

  sprintfFloat(bufRssiMin, sizeof(bufRssiMin),
               ((float) counters.rssi.min) / 4, 2);

  sprintfFloat(bufRssiMax, sizeof(bufRssiMax),
               ((float) counters.rssi.max) / 4, 2);

  sprintfFloat(bufRssiVariance, sizeof(bufRssiVariance),
               per_variance(counters.rssi) / 16, 0);

  app_log_info("{%s}"
                "{PerTriggers:%u}"
                "{RssiMean:%s}"
                "{RssiMin:%s}"
                "{RssiMax:%s}"
                "{RssiVariance:%s}\r\n",
                sl_cli_get_command_string(args, 1),
                counters.per_triggers,
                bufRssiMean,
                bufRssiMin,
                bufRssiMax,
                bufRssiVariance);
#else
  app_log_warning("{%s}"
                   "{To run PER commands, SL_RAIL_TEST_PER_PORT "
                   "and SL_RAIL_TEST_PER_PIN must be configured for use.}\r\n",
                   sl_cli_get_command_string(args, 1));
#endif //defined(SL_RAIL_TEST_PER_PORT) && defined(SL_RAIL_TEST_PER_PIN)
}

/******************************************************************************
 * CLI - Get Current RSSI value
 *****************************************************************************/
void rftest_cli_get_rssi(sl_cli_command_arg_t *args)
{

  uint32_t waitTimeout = RAIL_GET_RSSI_NO_WAIT;

  if (sl_cli_get_argument_count(args) == 1) {
    waitTimeout = sl_cli_get_argument_uint8(args, 0);
    // For backwards compatibility, map the value 1 to a no timeout wait
    if (waitTimeout == 1) {
      waitTimeout = RAIL_GET_RSSI_WAIT_WITHOUT_TIMEOUT;
    }
  }

  int16_t rssi = RAIL_GetRssiAlt(rftest_handle, waitTimeout) / 4;

  // The lowest negative value is used to indicate an error reading the RSSI
  if (rssi == RAIL_RSSI_INVALID) {
    app_log_info("{%s}Could not read RSSI. Ensure Rx is enabled",
                   sl_cli_get_command_string(args, 1));
    return;
  }

  app_log_info("{%s}{rssi:%d}\r\n", sl_cli_get_command_string(args, 1), rssi);
}

/******************************************************************************
 * CLI - Get all counters and status.
 *****************************************************************************/
void rftest_cli_get_status(sl_cli_command_arg_t *args)
{
  app_log_info("{%s}"
                "{TxCount:%u}"
                "{TxError:%u}"
                "{TxAbort:%u}"
                "{TxBlock:%u}"
                "{TxUnderFlow:%u}\r\n"
                "{RxCount:%u}"
                "{RxError:%u}"
                "{RxCrcError:%u}"
                "{RXOverflow:%u}"
                "{RxAbort:%u}"
                "{RxFrameError:%u}"
                "{TimingDetect:%u}"
                "{TimingLost:%u}"
                "{PreambleDetect:%u}"
                "{PreambleLost:%u}"
                "{SyncDetect:%u}"
                "{CalibrationError:%u}\r\n",
               sl_cli_get_command_string(args, 1),
               counters.packet_sent,
               counters.tx_error,
               counters.tx_abort,
               counters.tx_block,
               counters.tx_underflow,
               counters.packet_recevied,
               counters.rx_error,
               counters.crc_error,
               counters.rx_overflow,
               counters.rx_abort,
               counters.frame_error,
               counters.timing_detect,
               counters.timing_lost,
               counters.preamble_detect,
               counters.preamble_lost,
               counters.sync_detect,
               counters.cal_error);
}

/******************************************************************************
 * CLI - Reset counters
 *****************************************************************************/
void rftest_cli_reset_counters(sl_cli_command_arg_t *args)
{
  memset(&counters, 0, sizeof(counters));
  args->argc = sl_cli_get_command_count(args); /* only reference cmd str */
  rftest_cli_get_status(args);
}

void rftest_cli_get_config_index(sl_cli_command_arg_t *args)
{
#if SL_RAIL_UTIL_INIT_RADIO_CONFIG_SUPPORT_INST0_ENABLE
  app_log_info("{%s}"
               "{configIndex:%d}{%s}\r\n",
               sl_cli_get_command_string(args, 1),
               config_index,
               config_index_name[config_index]);

#else
  app_log_error("{%s},{%s}",sl_cli_get_command_string(args, 1),
                 "External radio config support not enabled"));
#endif // SL_RAIL_UTIL_INIT_RADIO_CONFIG_SUPPORT_INST0_ENABLE
}

/******************************************************************************
 * CLI - Set channel configuration index and switch to the desired PHY.
 *****************************************************************************/
void rftest_cli_set_config_index(sl_cli_command_arg_t *args)
{
#if SL_RAIL_UTIL_INIT_RADIO_CONFIG_SUPPORT_INST0_ENABLE
  uint8_t proposedIndex = sl_cli_get_argument_uint8(args, 0);

  // Be sure that the proposed index is valid. Scan through all possible
  // indexes and check for the last NULL parameter since you can't
  // use sizeof on an extern-ed array without an explicit index.
  for (uint8_t i = 0; i <= proposedIndex; i++) {
    if (channelConfigs[i] == NULL) {
      app_log_info("{%s}"
                    "{Invalid radio config index %d}\r\n",
                   sl_cli_get_command_string(args, 1),
                   proposedIndex);
      return;
    }
  }

  config_index = proposedIndex;
  RAIL_Idle(rftest_handle, RAIL_IDLE_ABORT, true);
  // Load the channel configuration for the specified index.
  rf_channel = RAIL_ConfigChannels(rftest_handle,
                                   channelConfigs[config_index],
                                   &sli_rail_util_on_channel_config_change);
  app_log_info("{%s},"
               "{configIndex:%d}"
               "{firstAvailableChannel:%d}\r\n",
                sl_cli_get_command_string(args, 1),
                config_index,
                rf_channel);


  (void) RAIL_StartRx(rftest_handle, rf_channel, NULL);

#else // !SL_RAIL_UTIL_INIT_RADIO_CONFIG_SUPPORT_INST0_ENABLE
  app_log_error("{%s},"
                "{%s}\r\n",
                sl_cli_get_command_string(args, 1),
                "External radio config support not enabled");
#endif // SL_RAIL_UTIL_INIT_RADIO_CONFIG_SUPPORT_INST0_ENABLE
}


/******************************************************************************
 * CLI - Set RF path
 *****************************************************************************/
void rftest_cli_set_rf_path(sl_cli_command_arg_t *args)
{
#ifdef  _SILICON_LABS_32B_SERIES_2
  RAIL_AntennaConfig_t halAntennaConfig = {0};
  uint32_t rf_path = RAIL_ANTENNA_0;

  if (sl_cli_get_argument_count(args) == 1) {
    rf_path = sl_cli_get_argument_uint32(args, 0);
  }

  if ((rf_path == RAIL_ANTENNA_0)
      || (rf_path == RAIL_ANTENNA_1)
      || (rf_path == RAIL_ANTENNA_AUTO)) {

    halAntennaConfig.defaultPath = rf_path;
    RAIL_ConfigAntenna(rftest_handle, &halAntennaConfig);
    app_log_info("{%s}"
                "{RfPath:%d}",
                sl_cli_get_command_string(args, 1),
                rf_path);
  }else {
    app_log_info("{%s}"
                 "{Invalid argument, should be one of %d,%d %d}",
                sl_cli_get_command_string(args, 1),
                RAIL_ANTENNA_0,
                RAIL_ANTENNA_1,
                RAIL_ANTENNA_AUTO);
  }
#endif
}

/******************************************************************************
 * Covert float to string.
 *****************************************************************************/
int sprintfFloat(char *buffer, int8_t len, float f, uint8_t precision)
{
  int8_t isNegative = (f < 0) ? 1 : 0;

  // Buffer needs to be big enough to hold sign (if negative), 1 integral digit,
  // precision fractional digits, decimal point (if precision > 0), and \0.
  if (buffer == NULL || len < (isNegative + 1 + precision + (precision > 0) + 1)) {
    return 0;
  }

  int8_t writeIndex = len - 1;
  buffer[writeIndex] = '\0';

  for (uint8_t exp = 0; exp < precision; exp++) {
    f *= 10;
  }

  int a;
  if (isNegative != 0) {
    a = -(int)(f - 0.5); // Round toward negative infinity
  } else {
    a = (int)(f + 0.5); // Round toward positive infinity
  }
  if (a < 0) { // Sign changed, float too large!
    return 0;
  }

  buffer[writeIndex--] = '\0'; // terminate string

  int8_t digit;
  do {
    digit = a % 10;
    a = a / 10;
    buffer[writeIndex--] = '0' + digit;
    if (precision && len == writeIndex + 2 + precision) {
      buffer[writeIndex--] = '.';
    }
  } while ((a != 0 || (precision && writeIndex >= (len - precision - 3)))
           && writeIndex >= isNegative);
  if (a != 0) {
    return 0; // Number too large to represent in buffer!
  }
  if (isNegative != 0) {
    buffer[writeIndex--] = '-';
  }

  // shift up
  if (writeIndex != -1 ) {
    memmove(buffer, &buffer[writeIndex + 1], len - writeIndex - 1);
  }
  return len - writeIndex - 1;
}
