/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file registers.c
 *
 * Implementation of the PX4IO register space.
 */

#include <px4_config.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

/*#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <systemlib/systemlib.h>
#include <stm32_pwr.h>
#include <rc/dsm.h>

#include "px4io.h"*/
#include "virtual_register.h"


/**
 * PAGE 0
 *
 * Static configuration parameters.
 */
/*static const uint16_t	r_page_config[] = {
	[PX4IO_P_CONFIG_PROTOCOL_VERSION]	= PX4IO_PROTOCOL_VERSION,
	[PX4IO_P_CONFIG_HARDWARE_VERSION]	= 2,
	[PX4IO_P_CONFIG_BOOTLOADER_VERSION]	= 3,
	[PX4IO_P_CONFIG_MAX_TRANSFER]		= 64,
	[PX4IO_P_CONFIG_CONTROL_COUNT]		= PX4IO_CONTROL_CHANNELS,
	[PX4IO_P_CONFIG_ACTUATOR_COUNT]		= PX4IO_SERVO_COUNT,
	[PX4IO_P_CONFIG_RC_INPUT_COUNT]		= PX4IO_RC_INPUT_CHANNELS,
	[PX4IO_P_CONFIG_ADC_INPUT_COUNT]	= PX4IO_ADC_CHANNEL_COUNT,
	[PX4IO_P_CONFIG_RELAY_COUNT]		= PX4IO_RELAY_CHANNELS,
};*/

static const uint16_t	r_page_config[] = {
	PX4IO_PROTOCOL_VERSION,
	2,
	3,
	64,
	PX4IO_CONTROL_CHANNELS,
	PX4IO_SERVO_COUNT,
	PX4IO_RC_INPUT_CHANNELS,
	PX4IO_ADC_CHANNEL_COUNT,
	PX4IO_RELAY_CHANNELS
};
/**
 * PAGE 1
 *
 * Status values.
 */
/*uint16_t		r_page_status[] = {
	[PX4IO_P_STATUS_FREEMEM]		= 0,
	[PX4IO_P_STATUS_CPULOAD]		= 0,
	[PX4IO_P_STATUS_FLAGS]			= 0,
	[PX4IO_P_STATUS_ALARMS]			= 0,
	[PX4IO_P_STATUS_VBATT]			= 0,
	[PX4IO_P_STATUS_IBATT]			= 0,
	[PX4IO_P_STATUS_VSERVO]			= 0,
	[PX4IO_P_STATUS_VRSSI]			= 0,
	[PX4IO_P_STATUS_PRSSI]			= 0,
	[PX4IO_P_STATUS_MIXER]			= 0,
};*/

uint16_t		r_page_status[] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

/**
 * PAGE 2
 *
 * Post-mixed actuator values.
 */
uint16_t 		r_page_actuators[PX4IO_SERVO_COUNT];

/**
 * PAGE 3
 *
 * Servo PWM values
 */
uint16_t		r_page_servos[PX4IO_SERVO_COUNT];

/**
 * PAGE 4
 *
 * Raw RC input
 */
/*uint16_t		r_page_raw_rc_input[] = {
	[PX4IO_P_RAW_RC_COUNT]			= 0,
	[PX4IO_P_RAW_RC_FLAGS]			= 0,
	[PX4IO_P_RAW_RC_NRSSI]			= 0,
	[PX4IO_P_RAW_RC_DATA]			= 0,
	[PX4IO_P_RAW_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_LOST_FRAME_COUNT]		= 0,
	[PX4IO_P_RAW_RC_BASE ...(PX4IO_P_RAW_RC_BASE + PX4IO_RC_INPUT_CHANNELS)] = 0
};*/

uint16_t		r_page_raw_rc_input[] = {
	0,
	0,
	0,
	0,
	0,
	0,
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * PAGE 5
 *
 * Scaled/routed RC input
 */
/*uint16_t		r_page_rc_input[] = {
	[PX4IO_P_RC_VALID]			= 0,
	[PX4IO_P_RC_BASE ...(PX4IO_P_RC_BASE + PX4IO_RC_MAPPED_CONTROL_CHANNELS)] = 0
};*/

uint16_t		r_page_rc_input[] = {
	0,
	0, 0, 0, 0, 0, 0, 0, 0
};

/**
 * Scratch page; used for registers that are constructed as-read.
 *
 * PAGE 6 Raw ADC input.
 * PAGE 7 PWM rate maps.
 */
uint16_t		r_page_scratch[32];

/**
 * PAGE 100
 *
 * Setup registers
 */
/*volatile uint16_t	r_page_setup[] = {
	[PX4IO_P_SETUP_FEATURES]		= 0,
	[PX4IO_P_SETUP_ARMING]			= (PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE),
	[PX4IO_P_SETUP_PWM_RATES]		= 0,
	[PX4IO_P_SETUP_PWM_DEFAULTRATE]		= 50,
	[PX4IO_P_SETUP_PWM_ALTRATE]		= 200,
	// this is unused, but we will pad it for readability (the compiler pads it automatically)
	[PX4IO_P_SETUP_RELAYS_PAD]		= 0,
	[PX4IO_P_SETUP_VBATT_SCALE]		= 10000,
	[PX4IO_P_SETUP_SET_DEBUG]		= 0,
	[PX4IO_P_SETUP_REBOOT_BL]		= 0,
	[PX4IO_P_SETUP_CRC ...(PX4IO_P_SETUP_CRC + 1)] = 0,
	[PX4IO_P_SETUP_RC_THR_FAILSAFE_US] = 0,
	[PX4IO_P_SETUP_PWM_REVERSE] = 0,
	[PX4IO_P_SETUP_TRIM_ROLL] = 0,
	[PX4IO_P_SETUP_TRIM_PITCH] = 0,
	[PX4IO_P_SETUP_TRIM_YAW] = 0
};*/

volatile uint16_t	r_page_setup[] = {
	0,
	PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE,
	0,
	50,
	200,
	0,
	10000,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0,
	0
};

#ifdef CONFIG_ARCH_BOARD_PX4IO_V2
#define PX4IO_P_SETUP_FEATURES_VALID	(PX4IO_P_SETUP_FEATURES_SBUS1_OUT | \
		PX4IO_P_SETUP_FEATURES_SBUS2_OUT | \
		PX4IO_P_SETUP_FEATURES_ADC_RSSI | \
		PX4IO_P_SETUP_FEATURES_PWM_RSSI)
#else
#define PX4IO_P_SETUP_FEATURES_VALID	0
#endif
#define PX4IO_P_SETUP_ARMING_VALID	(PX4IO_P_SETUP_ARMING_FMU_ARMED | \
		PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK | \
		PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK | \
		PX4IO_P_SETUP_ARMING_IO_ARM_OK | \
		PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM | \
		PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE | \
		PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED | \
		PX4IO_P_SETUP_ARMING_LOCKDOWN | \
		PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE | \
		PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE | \
		PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE)
#define PX4IO_P_SETUP_RATES_VALID	((1 << PX4IO_SERVO_COUNT) - 1)
#define PX4IO_P_SETUP_RELAYS_VALID	((1 << PX4IO_RELAY_CHANNELS) - 1)

/**
 * PAGE 101
 *
 * Control values from the FMU.
 */
volatile uint16_t	r_page_controls[PX4IO_CONTROL_GROUPS * PX4IO_CONTROL_CHANNELS];

/*
 * PAGE 102 does not have a buffer.
 */

/**
 * PAGE 103
 *
 * R/C channel input configuration.
 */
uint16_t		r_page_rc_input_config[PX4IO_RC_INPUT_CHANNELS * PX4IO_P_RC_CONFIG_STRIDE];

/* valid options */
#define PX4IO_P_RC_CONFIG_OPTIONS_VALID	(PX4IO_P_RC_CONFIG_OPTIONS_REVERSE | PX4IO_P_RC_CONFIG_OPTIONS_ENABLED)

/*
 * PAGE 104 uses r_page_servos.
 */

/**
 * PAGE 105
 *
 * Failsafe servo PWM values
 *
 * Disable pulses as default.
 */
uint16_t		r_page_servo_failsafe[PX4IO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };

/**
 * PAGE 106
 *
 * minimum PWM values when armed
 *
 */
uint16_t		r_page_servo_control_min[PX4IO_SERVO_COUNT] = { PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN, PWM_DEFAULT_MIN };

/**
 * PAGE 107
 *
 * maximum PWM values when armed
 *
 */
uint16_t		r_page_servo_control_max[PX4IO_SERVO_COUNT] = { PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX, PWM_DEFAULT_MAX };

/**
 * PAGE 108
 *
 * disarmed PWM values for difficult ESCs
 *
 */
uint16_t		r_page_servo_disarmed[PX4IO_SERVO_COUNT] = { 0, 0, 0, 0, 0, 0, 0, 0 };


uint16_t get_virtual_reg(uint8_t page, uint8_t offset)
{
	switch(page)
	{
		case PX4IO_PAGE_CONFIG : return r_page_config[offset]; break;
		case PX4IO_PAGE_STATUS : return r_page_status[offset]; break;
		case PX4IO_PAGE_ACTUATORS : return r_page_actuators[offset]; break;
		case PX4IO_PAGE_SERVOS : return r_page_servos[offset]; break;
		case PX4IO_PAGE_RAW_RC_INPUT : return r_page_raw_rc_input[offset]; break;
		case PX4IO_PAGE_RC_INPUT : return r_page_rc_input[offset]; break;
		case PX4IO_PAGE_RAW_ADC_INPUT : return r_page_scratch[offset]; break;
		case PX4IO_PAGE_PWM_INFO : return r_page_scratch[offset]; break;
		case PX4IO_PAGE_SETUP : return r_page_setup[offset]; break;
		case PX4IO_PAGE_CONTROLS : return r_page_controls[offset]; break;
		case PX4IO_PAGE_RC_CONFIG : return r_page_rc_input_config[offset]; break;
		case PX4IO_PAGE_DIRECT_PWM : return r_page_servos[offset]; break;
		case PX4IO_PAGE_FAILSAFE_PWM : return r_page_servo_failsafe[offset]; break;
		case PX4IO_PAGE_CONTROL_MAX_PWM : return r_page_servo_control_max[offset]; break;
		case PX4IO_PAGE_DISARMED_PWM : return r_page_servo_disarmed[offset]; break;
		default : printf("not defined page : %d\n",page); break;
	}

	return -1;
}

void set_virtual_reg(uint8_t page, uint8_t offset, uint16_t dat)
{
	switch(page)
	{
		case PX4IO_PAGE_CONFIG : printf("PX4IO_PAGE_CONFIG is static area\n"); break;
		case PX4IO_PAGE_STATUS :  r_page_status[offset] = dat; break;
		case PX4IO_PAGE_ACTUATORS :  r_page_actuators[offset] = dat; break;
		case PX4IO_PAGE_SERVOS :  r_page_servos[offset] = dat; break;
		case PX4IO_PAGE_RAW_RC_INPUT :  r_page_raw_rc_input[offset] = dat; break;
		case PX4IO_PAGE_RC_INPUT :  r_page_rc_input[offset] = dat; break;
		case PX4IO_PAGE_RAW_ADC_INPUT :  r_page_scratch[offset] = dat; break;
		case PX4IO_PAGE_PWM_INFO :  r_page_scratch[offset] = dat; break;
		case PX4IO_PAGE_SETUP :  r_page_setup[offset] = dat; break;
		case PX4IO_PAGE_CONTROLS :  r_page_controls[offset] = dat; break;
		case PX4IO_PAGE_RC_CONFIG :  r_page_rc_input_config[offset] = dat; break;
		case PX4IO_PAGE_DIRECT_PWM :  r_page_servos[offset] = dat; break;
		case PX4IO_PAGE_FAILSAFE_PWM :  r_page_servo_failsafe[offset] = dat; break;
		case PX4IO_PAGE_CONTROL_MAX_PWM :  r_page_servo_control_max[offset] = dat; break;
		case PX4IO_PAGE_DISARMED_PWM :  r_page_servo_disarmed[offset] = dat; break;
		default : printf("not defined page : %d\n",page); break;
	}
}

