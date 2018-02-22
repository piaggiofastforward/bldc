/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "conf_general.h"

// Functions
void encoder_deinit(void);
void encoder_init_abi(uint32_t counts);
void encoder_init_as5047p_spi(void);
bool encoder_is_configured(void);
float encoder_read_deg(void);
void encoder_reset(void);
void encoder_tim_isr(void);
void encoder_set_counts(uint32_t counts);

// the number of encoder counts per revolution 
uint32_t encoder_counts(void);

// has the encoder index pin triggered an interrupt since startup??
bool encoder_index_found(void);

/**
 *  Update the internal absolute encoder count by examining the current counts in the hardware
 *  timer that keeps track of encoder pulses.
 */
void encoder_update_abs_count(void);
 
/**
 *  Keep track of absolute encoder ticks since we probably want to use that instead of rotor angle.
 */
typedef uint32_t enc_continuous_count_t;
enc_continuous_count_t encoder_continuous_count(void);

#endif /* ENCODER_H_ */
