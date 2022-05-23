/***************************************************************************//**
 * @file
 * @brief rftest_command.c
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
#include "rftest_app.h"


// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//                          Static Function Declarations
// -----------------------------------------------------------------------------
void rftest_cli_enable(sl_cli_command_arg_t *args);
void rftest_cli_get_rail_version(sl_cli_command_arg_t *args);
void rftest_cli_get_data_rate(sl_cli_command_arg_t *args);
void rftest_cli_set_RxOptions(sl_cli_command_arg_t *args);
void rftest_cli_rx(sl_cli_command_arg_t *args);
void rftest_cli_config_TxOptions(sl_cli_command_arg_t *args);
void rftest_cli_tx_stream(sl_cli_command_arg_t *args);
void rftest_cli_tx_tone(sl_cli_command_arg_t *args);
void rftest_cli_get_power(sl_cli_command_arg_t *args);
void rftest_cli_get_power_limits(sl_cli_command_arg_t *args);
void rftest_cli_set_tx_power(sl_cli_command_arg_t *args);
void rftest_cli_get_channel(sl_cli_command_arg_t *args);
void rftest_cli_set_channel(sl_cli_command_arg_t *args);
void rftest_cli_reset_chip(sl_cli_command_arg_t *args);
void rftest_cli_get_ctune(sl_cli_command_arg_t *args);
void rftest_cli_set_ctune(sl_cli_command_arg_t *args);
void rftest_cli_get_ctune_delta(sl_cli_command_arg_t *args);
void rftest_cli_set_ctune_delta(sl_cli_command_arg_t *args);
void rftest_cli_tx(sl_cli_command_arg_t *args);
void rftest_cli_set_fixed_length(sl_cli_command_arg_t *args);
void rftest_cli_set_tx_length(sl_cli_command_arg_t *args);
void rftest_cli_set_tx_payload(sl_cli_command_arg_t *args);
void rftest_cli_print_tx_payload(sl_cli_command_arg_t *args);
void rftest_cli_start_per(sl_cli_command_arg_t *args);
void rftest_cli_get_per_stats(sl_cli_command_arg_t *args);
void rftest_cli_config_ber(sl_cli_command_arg_t *args);
void rftest_cli_ber_rx(sl_cli_command_arg_t *args);
void rftest_cli_get_ber_status(sl_cli_command_arg_t *args);
void rftest_cli_get_rssi(sl_cli_command_arg_t *args);
void rftest_cli_get_status(sl_cli_command_arg_t *args);
void rftest_cli_reset_counters(sl_cli_command_arg_t *args);
void rftest_cli_get_config_index(sl_cli_command_arg_t *args);
void rftest_cli_set_config_index(sl_cli_command_arg_t *args);
void rftest_cli_set_rf_path(sl_cli_command_arg_t *args);



// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------
extern const sl_cli_handle_t sl_cli_handles[];
// -----------------------------------------------------------------------------
//                                Static Variables
// -----------------------------------------------------------------------------

static const sl_cli_command_info_t cli_cmd__rftestEnable = \
    SL_CLI_COMMAND(rftest_cli_enable,
                   "Enable RF test and switch to rftest PHY.",
                   "",
                   {SL_CLI_ARG_UINT8, SL_CLI_ARG_END,});


static const sl_cli_command_info_t cli_cmd__getVersion = \
    SL_CLI_COMMAND(rftest_cli_get_rail_version,
                   "Get RAIL version information.",
                   "[uint8opt] enable verbose: 1=verbose version info.",
                   {SL_CLI_ARG_UINT8OPT, SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__printDataRates = \
    SL_CLI_COMMAND(rftest_cli_get_data_rate,
                   "Print the data rates of the current PHY.",
                   "",
                   {SL_CLI_ARG_END,});

static const sl_cli_command_info_t cli_cmd__setRxOptions = \
  SL_CLI_COMMAND(rftest_cli_set_RxOptions,
                 "Show/Configure receive options (RAIL_RX_OPTIONs).",
                 "rxOptionsValues: bitmask of enabled options"
                 SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__rx = \
    SL_CLI_COMMAND(rftest_cli_rx,
                   "Control receive mode.",
                    "0=Disable [1=Enable]" SL_CLI_UNIT_SEPARATOR,
                   {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setTxStream = \
  SL_CLI_COMMAND(rftest_cli_tx_stream,
                 "Control stream transmission.",
                  "0=Disable 1=Enable" SL_CLI_UNIT_SEPARATOR\
                  "streamMode: [1=PN9] 2=1010 3=phaseNoise 0=tone\n"\
                  "                    [uint32opt] antenna: [0]/1"\
                  SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32, SL_CLI_ARG_UINT32OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setTxTone = \
  SL_CLI_COMMAND(rftest_cli_tx_tone,
                 "Control tone transmission.",
                  "0=Disable 1=Enable" SL_CLI_UNIT_SEPARATOR
                  "antenna: [0]/1\n                    "
                  "[uint32opt] mode: [0]/1=phaseNoise"
                  SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32, SL_CLI_ARG_UINT32OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getPower = \
  SL_CLI_COMMAND(rftest_cli_get_power,
                 "Get the transmit power in deci-dBm.",
                  "",
                 {SL_CLI_ARG_END, });


static const sl_cli_command_info_t cli_cmd__setPower = \
  SL_CLI_COMMAND(rftest_cli_set_tx_power,
                 "Set the transmit power. The radio must be IDLE.",
                  "power: deci-dBm unless 'raw' is added"
                 SL_CLI_UNIT_SEPARATOR
                 "'raw'=units are raw power level"
                 SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_INT32, SL_CLI_ARG_STRINGOPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getPowerLimits = \
  SL_CLI_COMMAND(rftest_cli_get_power_limits,
                 "Get min and max powerLevel for a power mode.",
                  "powerMode" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getChannel = \
  SL_CLI_COMMAND(rftest_cli_get_channel,
                 "Get the current radio channel.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setChannel = \
  SL_CLI_COMMAND(rftest_cli_set_channel,
                 "Set the radio channel.",
                  "channel" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__reset = \
  SL_CLI_COMMAND(rftest_cli_reset_chip,
                 "Perform a reboot of the chip.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setCtune = \
  SL_CLI_COMMAND(rftest_cli_set_ctune,
                 "Set the value of HFXO CTUNE. The radio must be IDLE.",
                  "ctune" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getCtune = \
  SL_CLI_COMMAND(rftest_cli_get_ctune,
                 "Get the value of HFXO CTUNE",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setCtuneDelta = \
  SL_CLI_COMMAND(rftest_cli_set_ctune_delta,
                 "Set the value of HFXO CTUNE delta",
                  "delta" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getCtuneDelta = \
  SL_CLI_COMMAND(rftest_cli_get_ctune_delta,
                 "Get the value of HFXO CTUNE delta",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__configTxOptions = \
  SL_CLI_COMMAND(rftest_cli_config_TxOptions,
                 "Show/Configure transmit options (RAIL_TX_OPTIONs).",
                 "txOptionsValues: bitmask of enabled options"
                 SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setFixedLength = \
  SL_CLI_COMMAND(rftest_cli_set_fixed_length,
                 "Configure fixed length packet operation.",
                  "fixedLength: payload bytes" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__tx = \
  SL_CLI_COMMAND(rftest_cli_tx,
                 "Transmit packets with current TX options.",
                  "number of packets, 0=continuous until next 'tx 0'"
                  SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32, SL_CLI_ARG_END, });


static const sl_cli_command_info_t cli_cmd__setTxPayload = \
  SL_CLI_COMMAND(rftest_cli_set_tx_payload,
                 "Set TX packet payload bytes for future transmits.",
                  "offset" SL_CLI_UNIT_SEPARATOR "byte0 byte1 ..."
                  SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_UINT8OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setTxLength = \
  SL_CLI_COMMAND(rftest_cli_set_tx_length,
                 "Set how much data to load into the TX FIFO for transmitting."
                 "\n                    Actual packet length may vary based on"
                 " radio configuration.",
                  "lengthBytes" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT16, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__printTxPacket = \
  SL_CLI_COMMAND(rftest_cli_print_tx_payload,
                 "Print the current TX payload data and byte length.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__perRx = \
  SL_CLI_COMMAND(rftest_cli_start_per,
                 "Start a Packet Error Rate test. 'perRx 0 0' stops test.",
                  "number of packets" SL_CLI_UNIT_SEPARATOR "delayUs"
                  SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32, SL_CLI_ARG_UINT32, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__perStatus = \
  SL_CLI_COMMAND(rftest_cli_get_per_stats,
                 "Get the PER test results. Also see status command.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setBerConfig = \
  SL_CLI_COMMAND(rftest_cli_config_ber,
                 "Set number of bytes to receive in BER mode.",
                  "number of bytes: 0=maximum (536870911)"
                  SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__berRx = \
  SL_CLI_COMMAND(rftest_cli_ber_rx,
                 "Control BER receive mode.",
                  "0=Disable 1=Enable" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__berStatus = \
  SL_CLI_COMMAND(rftest_cli_get_ber_status,
                 "Get status of current or last BER test.\n"
                 "                    Status is reset by setBerConfig"
                 " and berRx enable.",
                 "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getRssi = \
  SL_CLI_COMMAND(rftest_cli_get_rssi,
                 "Get RSSI in dBm. It'll be invalid if receiver isn't ready.",
                 "[0=don't wait] 1=wait for valid RSSI if possible"
                 SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32OPT, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__resetCounters = \
  SL_CLI_COMMAND(rftest_cli_reset_counters,
                 "Resets the TX and RX counters.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__status = \
  SL_CLI_COMMAND(rftest_cli_get_status,
                 "Print the current status counters.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__getConfigIndex = \
  SL_CLI_COMMAND(rftest_cli_get_config_index,
                 "Get the index of the current multi-PHY radio config."
                 "\n                    See the entries in *channelConfigs[]."
                 " Start with index 0.",
                  "",
                 {SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setConfigIndex = \
  SL_CLI_COMMAND(rftest_cli_set_config_index,
                 "Activate a multi-PHY radio configuration."
                 "\n                    See the entries in *channelConfigs[]."
                 " Start with index 0.",
                  "multiPhyIndex" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT8, SL_CLI_ARG_END, });

static const sl_cli_command_info_t cli_cmd__setRfPath = \
  SL_CLI_COMMAND(rftest_cli_set_rf_path,
                 "Set the RF path.",
                  "0=Path0 1=Path1" SL_CLI_UNIT_SEPARATOR,
                 {SL_CLI_ARG_UINT32, SL_CLI_ARG_END, });


// Create group command
const sl_cli_command_entry_t sl_cli_rftest_group_table[] = {
  { "getVersion", &cli_cmd__getVersion, false },
  { "printDataRates", &cli_cmd__printDataRates, false },
  { "setRfPath", &cli_cmd__setRfPath, false },
  { "getConfigIndex", &cli_cmd__getConfigIndex, false },
  { "setConfigIndex", &cli_cmd__setConfigIndex, false },
  { "setRxOptions", &cli_cmd__setRxOptions, false },
  { "rx", &cli_cmd__rx, false },
  { "setTxOptions", &cli_cmd__configTxOptions, false },
  { "setFixedLength", &cli_cmd__setFixedLength, false },
  { "tx", &cli_cmd__tx, false },
  { "setTxPayload", &cli_cmd__setTxPayload, false },
  { "setTxLength", &cli_cmd__setTxLength, false },
  { "printTxPacket", &cli_cmd__printTxPacket, false },
  { "setPower", &cli_cmd__setPower, false },
  { "getPower", &cli_cmd__getPower, false },
  { "getPowerLimits", &cli_cmd__getPowerLimits, false },
  { "getRssi", &cli_cmd__getRssi, false },
  { "setTxTone", &cli_cmd__setTxTone, false },
  { "setTxStream", &cli_cmd__setTxStream, false },
  { "status", &cli_cmd__status, false },
  { "setCtune", &cli_cmd__setCtune, false },
  { "getCtune", &cli_cmd__getCtune, false },
  { "setCtuneDelta", &cli_cmd__setCtuneDelta, false },
  { "getCtuneDelta", &cli_cmd__getCtuneDelta, false },
  { "getChannel", &cli_cmd__getChannel, false },
  { "setChannel", &cli_cmd__setChannel, false },
  { "perRx", &cli_cmd__perRx, false },
  { "perStatus", &cli_cmd__perStatus, false },
  { "setBerConfig", &cli_cmd__setBerConfig, false },
  { "berRx", &cli_cmd__berRx, false },
  { "berStatus", &cli_cmd__berStatus, false },
  { "reset", &cli_cmd__reset, false },
  { "resetCounters", &cli_cmd__resetCounters, false },
  { NULL, NULL, false },
};

static const sl_cli_command_info_t cli_cmd_group_rftest = \
  SL_CLI_COMMAND_GROUP(sl_cli_rftest_group_table, "RFtest command group");

// Create command table
const sl_cli_command_entry_t sl_cli_rftest_command_table[] = {
  { "rftestEnable", &cli_cmd__rftestEnable, false },
  { "rftest", &cli_cmd_group_rftest, false },
  { NULL, NULL, false },
};

sl_cli_command_group_t sl_cli_rftest_command_group =
{
  { NULL },
  false,
  sl_cli_rftest_command_table
};
// -----------------------------------------------------------------------------
//                          Public Function Definitions
// -----------------------------------------------------------------------------

/******************************************************************************
 * BER FIFO mode, almost full event process
 *****************************************************************************/
void rftest_add_cli_cmd_group(void)
{
  sl_cli_command_add_command_group(sl_cli_handles[0],
                                   &sl_cli_rftest_command_group);
}

// -----------------------------------------------------------------------------
//                          Static Function Definitions
// -----------------------------------------------------------------------------


