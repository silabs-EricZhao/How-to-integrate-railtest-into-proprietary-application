/***************************************************************************//**
 * @file
 * @brief rftest_app.h
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
#ifndef RFTEST_APP_H
#define RFTEST_APP_H

// -----------------------------------------------------------------------------
//                                   Includes
// -----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "rail.h"

// -----------------------------------------------------------------------------
//                              Macros and Typedefs
// -----------------------------------------------------------------------------



// -----------------------------------------------------------------------------
//                                Global Variables
// -----------------------------------------------------------------------------

extern RAIL_FrequencyOffset_t rxFreqOffset;
extern RAIL_TxOptions_t txOptions;
extern RAIL_RxOptions_t rxOptions;
extern uint8_t          tx_length;
// -----------------------------------------------------------------------------
//                          Public Function Declarations
// -----------------------------------------------------------------------------
extern void rftest_set_state(rftest_status_t state);
extern rftest_status_t rftest_get_state(void);
extern void rftest_init(void);
extern void rftest_reset_counters(void);
extern void rftest_print_tx_packet(void);
extern void rftest_set_tx_paylaod(uint8_t offset, uint8_t value);
extern float per_variance(const Stats_t stats);
extern void rftest_add_cli_cmd_group(void);
#endif
