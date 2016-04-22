/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4io.cpp
 * Driver for the PX4IO board.
 *
 * PX4IO is connected via I2C or DMA enabled high-speed UART.
 */

#ifndef __STDC_LIMIT_MACROS
#define __STDC_LIMIT_MACROS
#endif

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <ctype.h>

#include <px4_tasks.h>
#include <px4_posix.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_sbus.h>
#include <drivers/drv_mixer.h>
#include <drivers/ts7500io_virtual_firmware/virtual_register.h>

#include <systemlib/param/param.h>
#include <systemlib/circuit_breaker.h>
//#include <systemlib/perf_counter.h>

#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/safety.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/servorail_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/input_rc.h>

#include <mavlink/mavlink_log.h>
#include "modules/dataman/dataman.h"

/*#include <crc32.h>

#include <systemlib/mixer/mixer.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>

#include <modules/px4iofirmware/protocol.h>

#include "uploader.h"


#include "px4io_driver.h"
*/

#define PX4IO_SET_DEBUG			_IO(0xff00, 0)
#define PX4IO_INAIR_RESTART_ENABLE	_IO(0xff00, 1)
#define PX4IO_REBOOT_BOOTLOADER		_IO(0xff00, 2)
#define PX4IO_CHECK_CRC			_IO(0xff00, 3)

#define UPDATE_INTERVAL_MIN		2			// 2 ms	-> 500 Hz
#define ORB_CHECK_INTERVAL		200000		// 200 ms -> 5 Hz
#define IO_POLL_INTERVAL		20000		// 20 ms -> 50 Hz

#define TS7500IO_DEVICE_PATH	"/dev/ts7500io"
#define SCHED_PRIORITY_ACTUATOR_OUTPUTS     (SCHED_PRIORITY_MAX - 15)

class RC_Driver : public device::VDev
{
public:

	RC_Driver();

	~RC_Driver();

	virtual int init();

	int init(bool disable_rc_handling);

	/**
	 * Disable RC input handling
	 */
	int			disable_rc_handling();

	inline uint16_t		system_status() const {return _status;}

private:

	//device::VDev			*PWM_dev;
	unsigned		_hardware;		///< Hardware revision
	unsigned		_max_actuators;		///< Maximum # of actuators supported by PX4IO
	unsigned		_max_controls;		///< Maximum # of controls supported by PX4IO
	unsigned		_max_rc_input;		///< Maximum receiver channels supported by PX4IO
	unsigned		_max_relays;		///< Maximum relays supported by PX4IO
	unsigned		_max_transfer;		///< Maximum number of I2C transfers supported by PX4IO

	unsigned 		_update_interval;	///< Subscription interval limiting send rate
	bool			_rc_handling_disabled;	///< If set, IO does not evaluate, but only forward the RC values
	unsigned		_rc_chan_count;		///< Internal copy of the last seen number of RC channels
	uint64_t		_rc_last_valid;		///< last valid timestamp

	volatile int _task;
	volatile bool _task_should_exit;

	int			_ts7500io_fd;
	int			_mavlink_fd;		///< mavlink file descriptor.

	uint16_t		_status;		///< Various IO status flags
	uint16_t		_alarms;		///< Various IO alarms
	uint16_t		_last_written_arming_s;	///< the last written arming state reg
	uint16_t		_last_written_arming_c;	///< the last written arming state reg

	/* subscribed topics */
	int			_t_actuator_controls_0;	///< actuator controls group 0 topic
	int			_t_actuator_controls_1;	///< actuator controls group 1 topic
	int			_t_actuator_controls_2;	///< actuator controls group 2 topic
	int			_t_actuator_controls_3;	///< actuator controls group 3 topic
	int			_t_actuator_armed;	///< system armed control topic
	int 			_t_vehicle_control_mode;///< vehicle control mode topic
	int			_t_param;		///< parameter update topic
	bool			_param_update_force;	///< force a parameter update
	int			_t_vehicle_command;	///< vehicle command topic

	orb_advert_t 		_to_input_rc;		///< rc inputs from io
	orb_advert_t		_to_outputs;		///< mixed servo outputs topic
	orb_advert_t		_to_servorail;		///< servorail status
	orb_advert_t		_to_safety;		///< status of safety
	orb_advert_t 		_to_mixer_status; 	///< mixer status flags

	actuator_outputs_s	_outputs;		///< mixed outputs

	bool			_primary_pwm_device;	///< true if we are the default PWM output
	bool			_lockdown_override;	///< allow to override the safety lockdown

	bool			_cb_flighttermination;	///< true if the flight termination circuit breaker is enabled
	bool 			_in_esc_calibration_mode;	///< do not send control outputs to IO (used for esc calibration)

	int32_t			_rssi_pwm_chan; ///< RSSI PWM input channel
	int32_t			_rssi_pwm_max; ///< max RSSI input on PWM channel
	int32_t			_rssi_pwm_min; ///< min RSSI input on PWM channel

	/**
	 * IO Control handler.
	 *
	 * Handle all IOCTL calls to the PX4IO file descriptor.
	 *
	 * @param[in] filp file handle (not used). This function is always called directly through object reference
	 * @param[in] cmd the IOCTL command
	 * @param[in] the IOCTL command parameter (optional)
	 */
	virtual int		ioctl(device::file_t *filep, int cmd, unsigned long arg);


	/**
	 * read register(s)
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @param values	Pointer to array where values should be stored.
	 * @param num_values	The number of values to read.
	 * @return		OK if all values were successfully read.
	 */
	int			io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values);

	/**
	 * read a register
	 *
	 * @param page		Register page to read from.
	 * @param offset	Register offset to start reading from.
	 * @return		Register value that was read, or _io_reg_get_error on error.
	 */
	uint32_t		io_reg_get(uint8_t page, uint8_t offset);
	static const uint32_t	_io_reg_get_error = 0x80000000;

	/**
	 * write register(s)
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to start writing at.
	 * @param values	Pointer to array of values to write.
	 * @param num_values	The number of values to write.
	 * @return		OK if all values were successfully written.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values);

	/**
	 * write a register
	 *
	 * @param page		Register page to write to.
	 * @param offset	Register offset to write to.
	 * @param value		Value to write.
	 * @return		OK if the value was written successfully.
	 */
	int			io_reg_set(uint8_t page, uint8_t offset, const uint16_t value);

	/**
	 * modify a register
	 *
	 * @param page		Register page to modify.
	 * @param offset	Register offset to modify.
	 * @param clearbits	Bits to clear in the register.
	 * @param setbits	Bits to set in the register.
	 */
	int			io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);


	/**
	 * Send controls for one group to IO
	 */
	int			io_set_control_state(unsigned group);

	/**
	 * Send all controls to IO
	 */
	int			io_set_control_groups();

	/**
	 * Update IO's arming-related state
	 */
	int			io_set_arming_state();

	/**
	 * Push RC channel configuration to IO.
	 */
	int			io_set_rc_config();

	/**
	 * Handle a status update from IO.
	 *
	 * Publish IO status information if necessary.
	 *
	 * @param status	The status register
	 */
	int			io_handle_status(uint16_t status);

	/**
	 * Handle an alarm update from IO.
	 *
	 * Publish IO alarm information if necessary.
	 *
	 * @param alarm		The status register
	 */
	int			io_handle_alarms(uint16_t alarms);



	int io_get_status();

	/**
	 * Disable RC input handling
	 */
	int			io_disable_rc_handling();

	/**
	 * Fetch RC inputs from IO.
	 *
	 * @param input_rc	Input structure to populate.
	 * @return		OK if data was returned.
	 */
	int			io_get_raw_rc_input(rc_input_values &input_rc);

	/**
	 * Fetch and publish raw RC input data.
	 */
	int			io_publish_raw_rc();

	/**
	 * Fetch and publish the PWM servo outputs.
	 */
	int			io_publish_pwm_outputs();

	int			pwm_init();

	/**
	 * Send mixer definition text to IO
	 */
	int			mixer_send(const char *buf, unsigned buflen, unsigned retries = 3);

	int 		load_mixer_file(const char *fname, char *buf, unsigned maxlen);

	int			mixer_load(const char *fname);

	static void task_main_trampoline(int argc, char *argv[]);

	int task_main();

};

RC_Driver *rc_dev = NULL;

RC_Driver::RC_Driver() :
	VDev("ts7500io", TS7500IO_DEVICE_PATH)
{
	//PWM_dev = new device::VDev("pwm-dev", PWM_OUTPUT0_DEVICE_PATH);
	_hardware = 0;
	_max_actuators = 0;
	_max_controls = 0;
	_max_rc_input = 0;
	_max_relays = 0;
	_max_transfer = 16;
	_update_interval = 0;
	_rc_handling_disabled = false;
	_rc_chan_count = 0;
	_rc_last_valid = 0;
	_task = -1;
	_task_should_exit = false;
	_ts7500io_fd = -1;
	_mavlink_fd = -1;
	_status = 0;
	_alarms = 0;
	_t_actuator_controls_0 = -1;
	_t_actuator_controls_1 = -1;
	_t_actuator_controls_2 = -1;
	_t_actuator_controls_3 = -1;
	_t_actuator_armed = -1;
	_t_vehicle_control_mode = -1;
	_t_param = -1;
	_param_update_force = false;
	_t_vehicle_command = -1;
	_to_input_rc = NULL;
	_to_safety = NULL;
	_primary_pwm_device = false;
	_cb_flighttermination = true;
	_rssi_pwm_chan = 0;
	_rssi_pwm_max = 0;
	_rssi_pwm_min = 0;

	rc_dev = this;
}

RC_Driver::~RC_Driver()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		px4_task_delete(_task);
	}

	rc_dev = NULL;
}

int RC_Driver::ioctl(device::file_t *filep, int cmd, unsigned long arg)
{
	int ret = OK;

	/* regula::ioctl? */
	switch (cmd) {
	case PWM_SERVO_ARM:
		/* set the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FMU_ARMED);
		break;

	case PWM_SERVO_SET_ARM_OK:
		/* set the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_IO_ARM_OK);
		break;

	case PWM_SERVO_CLEAR_ARM_OK:
		/* clear the 'OK to arm' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_IO_ARM_OK, 0);
		break;

	case PWM_SERVO_DISARM:
		/* clear the 'armed' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FMU_ARMED, 0);
		break;

	case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
		/* get the default update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_DEFAULTRATE);
		break;

	case PWM_SERVO_SET_UPDATE_RATE:
		/* set the requested alternate rate */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE, arg);
		break;

	case PWM_SERVO_GET_UPDATE_RATE:
		/* get the alternative update rate */
		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE);
		break;

	case PWM_SERVO_SET_SELECT_UPDATE_RATE: {

			/* blindly clear the PWM update alarm - might be set for some other reason */
			io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);

			/* attempt to set the rate map */
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, arg);

			/* check that the changes took */
			uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);

			if (alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR) {
				ret = -EINVAL;
				io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);
			}

			break;
		}

	case PWM_SERVO_GET_SELECT_UPDATE_RATE:

		*(unsigned *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES);
		break;

	case PWM_SERVO_SET_FAILSAFE_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_FAILSAFE_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_FAILSAFE_PWM:

		ret = io_reg_get(PX4IO_PAGE_FAILSAFE_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_SET_DISARMED_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_DISARMED_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_DISARMED_PWM:

		ret = io_reg_get(PX4IO_PAGE_DISARMED_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_SET_MIN_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MIN_PWM:

		ret = io_reg_get(PX4IO_PAGE_CONTROL_MIN_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_SET_MAX_PWM: {
			struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

			if (pwm->channel_count > _max_actuators)
				/* fail with error */
			{
				return -E2BIG;
			}

			/* copy values to registers in IO */
			ret = io_reg_set(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm->values, pwm->channel_count);
			break;
		}

	case PWM_SERVO_GET_MAX_PWM:

		ret = io_reg_get(PX4IO_PAGE_CONTROL_MAX_PWM, 0, (uint16_t *)arg, _max_actuators);

		if (ret != OK) {
			ret = -EIO;
		}

		break;

	case PWM_SERVO_GET_COUNT:
		*(unsigned *)arg = _max_actuators;
		break;

	case PWM_SERVO_SET_DISABLE_LOCKDOWN:
		_lockdown_override = arg;
		break;

	case PWM_SERVO_GET_DISABLE_LOCKDOWN:
		*(unsigned *)arg = _lockdown_override;
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_OFF:
		/* force safety swith off */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_SAFETY_ON:
		/* force safety switch on */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_ON, PX4IO_FORCE_SAFETY_MAGIC);
		break;

	case PWM_SERVO_SET_FORCE_FAILSAFE:

		/* force failsafe mode instantly */
		if (arg == 0) {
			/* clear force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE, 0);

		} else {
			/* set force failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_TERMINATION_FAILSAFE:

		/* if failsafe occurs, do not allow the system to recover */
		if (arg == 0) {
			/* clear termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE, 0);

		} else {
			/* set termination failsafe flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE);
		}

		break;

	case PWM_SERVO_SET_OVERRIDE_IMMEDIATE:

		/* control whether override on FMU failure is
		   immediate or waits for override threshold on mode
		   switch */
		if (arg == 0) {
			/* clear override immediate flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE, 0);

		} else {
			/* set override immediate flag */
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE);
		}

		break;

	case DSM_BIND_START:

		/* only allow DSM2, DSM-X and DSM-X with more than 7 channels */
		if (arg == DSM2_BIND_PULSES ||
		    arg == DSMX_BIND_PULSES ||
		    arg == DSMX8_BIND_PULSES) {
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_down);
			usleep(500000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_set_rx_out);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
			usleep(72000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_send_pulses | (arg << 4));
			usleep(50000);
			io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_reinit_uart);

			ret = OK;

		} else {
			ret = -EINVAL;
		}

		break;

	case DSM_BIND_POWER_UP:
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_DSM, dsm_bind_power_up);
		break;

	case PWM_SERVO_SET(0) ... PWM_SERVO_SET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			/* TODO: we could go lower for e.g. TurboPWM */
			unsigned channel = cmd - PWM_SERVO_SET(0);

			if ((channel >= _max_actuators) || (arg < PWM_LOWEST_MIN) || (arg > PWM_HIGHEST_MAX)) {
				ret = -EINVAL;

			} else {
				/* send a direct PWM value */
				ret = io_reg_set(PX4IO_PAGE_DIRECT_PWM, channel, arg);
			}

			break;
		}

	case PWM_SERVO_GET(0) ... PWM_SERVO_GET(PWM_OUTPUT_MAX_CHANNELS - 1): {

			unsigned channel = cmd - PWM_SERVO_GET(0);

			if (channel >= _max_actuators) {
				ret = -EINVAL;

			} else {
				/* fetch a current PWM value */
				uint32_t value = io_reg_get(PX4IO_PAGE_SERVOS, channel);

				if (value == _io_reg_get_error) {
					ret = -EIO;

				} else {
					*(servo_position_t *)arg = value;
				}
			}

			break;
		}

	case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
			// jjh deprecated
			/*unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

			*(uint32_t *)arg = io_reg_get(PX4IO_PAGE_PWM_INFO, PX4IO_RATE_MAP_BASE + channel);

			if (*(uint32_t *)arg == _io_reg_get_error) {
				ret = -EIO;
			}*/

			break;
		}

	case GPIO_RESET: {
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
			uint32_t bits = (1 << _max_relays) - 1;

			/* don't touch relay1 if it's controlling RX vcc */
			if (_dsm_vcc_ctl) {
				bits &= ~PX4IO_P_SETUP_RELAYS_POWER1;
			}

			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS, bits, 0);
#endif
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
			ret = -EINVAL;
#endif
			break;
		}

	case GPIO_SET:
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
		arg &= ((1 << _max_relays) - 1);

		/* don't touch relay1 if it's controlling RX vcc */
		if (_dsm_vcc_ctl & (arg & PX4IO_P_SETUP_RELAYS_POWER1)) {
			ret = -EINVAL;
			break;
		}

		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS, 0, arg);
#endif
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
		ret = -EINVAL;
#endif
		break;

	case GPIO_CLEAR:
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
		arg &= ((1 << _max_relays) - 1);

		/* don't touch relay1 if it's controlling RX vcc */
		if (_dsm_vcc_ctl & (arg & PX4IO_P_SETUP_RELAYS_POWER1)) {
			ret = -EINVAL;
			break;
		}

		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS, arg, 0);
#endif
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
		ret = -EINVAL;
#endif
		break;

	case GPIO_GET:
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
		*(uint32_t *)arg = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RELAYS);

		if (*(uint32_t *)arg == _io_reg_get_error) {
			ret = -EIO;
		}

#endif
#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
		ret = -EINVAL;
#endif
		break;

	case MIXERIOCGETOUTPUTCOUNT:
		*(unsigned *)arg = _max_actuators;
		break;

	case MIXERIOCRESET:
		ret = 0;	/* load always resets */
		break;

	case MIXERIOCLOADBUF: {
			const char *buf = (const char *)arg;
			ret = mixer_send(buf, strnlen(buf, 2048));
			break;
		}

	case RC_INPUT_GET: {
			uint16_t status;
			rc_input_values *rc_val = (rc_input_values *)arg;

			ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &status, 1);

			if (ret != OK) {
				break;
			}

			/* if no R/C input, don't try to fetch anything */
			if (!(status & PX4IO_P_STATUS_FLAGS_RC_OK)) {
				ret = -ENOTCONN;
				break;
			}

			/* sort out the source of the values */
			if (status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

			} else if (status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

			} else if (status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

			} else if (status & PX4IO_P_STATUS_FLAGS_RC_ST24) {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24;

			} else {
				rc_val->input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;
			}

			/* read raw R/C input values */
			ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE, &(rc_val->values[0]), _max_rc_input);
			break;
		}

	case PX4IO_SET_DEBUG:
		/* set the debug level */
		ret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG, arg);
		break;

	case PX4IO_REBOOT_BOOTLOADER:
		if (system_status() & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
			return -EINVAL;
		}

		// jjh deprecated
		/* reboot into bootloader - arg must be PX4IO_REBOOT_BL_MAGIC */
		//io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_REBOOT_BL, arg);
		// we don't expect a reply from this operation
		ret = OK;
		break;

	case PX4IO_CHECK_CRC: {
			/* check IO firmware CRC against passed value */
			uint32_t io_crc = 0;
			ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_CRC, (uint16_t *)&io_crc, 2);

			if (ret != OK) {
				return ret;
			}

			// jjh deprecated - there is virtual firmware, no need to check crc
			/*if (io_crc != arg) {
				DEVICE_DEBUG("crc mismatch 0x%08x 0x%08x", (unsigned)io_crc, arg);
				return -EINVAL;
			}*/

			break;
		}

	case PX4IO_INAIR_RESTART_ENABLE:

		/* set/clear the 'in-air restart' bit */
		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK, 0);
		}

		break;

	case RC_INPUT_ENABLE_RSSI_ANALOG:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_ADC_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_ADC_RSSI, 0);
		}

		break;

	case RC_INPUT_ENABLE_RSSI_PWM:

		if (arg) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_PWM_RSSI);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, PX4IO_P_SETUP_FEATURES_PWM_RSSI, 0);
		}

		break;

	case SBUS_SET_PROTO_VERSION:

		if (arg == 1) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS1_OUT);

		} else if (arg == 2) {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES, 0, PX4IO_P_SETUP_FEATURES_SBUS2_OUT);

		} else {
			ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FEATURES,
					    (PX4IO_P_SETUP_FEATURES_SBUS1_OUT | PX4IO_P_SETUP_FEATURES_SBUS2_OUT), 0);
		}

		break;

	case PWM_SERVO_SET_RC_CONFIG: {
			/* enable setting of RC configuration without relying
			   on param_get()
			*/
			struct pwm_output_rc_config *config = (struct pwm_output_rc_config *)arg;

			if (config->channel >= input_rc_s::RC_INPUT_MAX_CHANNELS) {
				/* fail with error */
				return -E2BIG;
			}

			/* copy values to registers in IO */
			uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
			uint16_t offset = config->channel * PX4IO_P_RC_CONFIG_STRIDE;
			regs[PX4IO_P_RC_CONFIG_MIN]        = config->rc_min;
			regs[PX4IO_P_RC_CONFIG_CENTER]     = config->rc_trim;
			regs[PX4IO_P_RC_CONFIG_MAX]        = config->rc_max;
			regs[PX4IO_P_RC_CONFIG_DEADZONE]   = config->rc_dz;
			regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = config->rc_assignment;
			regs[PX4IO_P_RC_CONFIG_OPTIONS]    = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;

			if (config->rc_reverse) {
				regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
			}

			ret = io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);
			break;
		}

	case PWM_SERVO_SET_OVERRIDE_OK:
		/* set the 'OVERRIDE OK' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, 0, PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK);
		break;

	case PWM_SERVO_CLEAR_OVERRIDE_OK:
		/* clear the 'OVERRIDE OK' bit */
		ret = io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK, 0);
		break;

	default:
		/* see if the parent class can make any use of it */
		ret = VDev::ioctl(filep, cmd, arg);
		break;
	}

	return ret;
}



int RC_Driver::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		printf("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	//int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);

	get_virtual_register(page, offset, values, num_values);
	/*int i;
	for( i = 0; i < num_values; i++ )
		printf("get - page : %d, offset : %d, values : 0x%x\n", page, offset+i, *(values+i));*/

	/*if (ret != (int)num_values) {
		printf("io_reg_get(%u,%u,%u): data error %d", page, offset, num_values, ret);
		return -1;
	}*/

	return OK;
}

uint32_t RC_Driver::io_reg_get(uint8_t page, uint8_t offset)
{
	uint16_t value;

	if (io_reg_get(page, offset, &value, 1) != OK) {
		return _io_reg_get_error;
	}

	return value;
}

int RC_Driver::io_reg_set(uint8_t page, uint8_t offset, const uint16_t *values, unsigned num_values)
{
/*	int i;
	for( i = 0; i < num_values; i++ )
		printf("set - page : %d, offset : %d, values : 0x%x\n", page, offset+i, *(values+i));*/
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		printf("io_reg_set: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	//int ret =  _interface->write((page << 8) | offset, (void *)values, num_values);

	
	// jjh
	/*for( int i = 0; i < num_values; i++ )
		printf("temp[%d] = %d;\n",i, values[i]);
	printf("io_reg_set(%d, %d, temp, %d);\n",page,offset,num_values);*/
	set_virtual_register(page, offset, values, num_values);

	/*if (ret != (int)num_values) {
		printf("io_reg_set(%u,%u,%u): error %d", page, offset, num_values, ret);
		return -1;
	}*/

	return OK;
}

int RC_Driver::io_reg_set(uint8_t page, uint8_t offset, uint16_t value)
{
	return io_reg_set(page, offset, &value, 1);
}

int RC_Driver::io_reg_modify(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
	int ret;
	uint16_t value;

	ret = io_reg_get(page, offset, &value, 1);

	if (ret != OK) {
		return ret;
	}

	value &= ~clearbits;
	value |= setbits;

	return io_reg_set(page, offset, value);
}

int RC_Driver::io_handle_status(uint16_t status)
{
	int ret = 1;
	/**
	 * WARNING: This section handles in-air resets.
	 */

	/* check for IO reset - force it back to armed if necessary */
	if (_status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF && !(status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF)
	    && !(status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {
		/* set the arming flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0,
				    PX4IO_P_STATUS_FLAGS_SAFETY_OFF | PX4IO_P_STATUS_FLAGS_ARM_SYNC);

		/* set new status */
		_status = status;
		_status &= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

	} else if (!(_status & PX4IO_P_STATUS_FLAGS_ARM_SYNC)) {

		/* set the sync flag */
		ret = io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_ARM_SYNC);
		/* set new status */
		_status = status;

	} else {
		ret = 0;

		/* set new status */
		_status = status;
	}

	/**
	 * Get and handle the safety status
	 */
	struct safety_s safety;
	safety.timestamp = hrt_absolute_time();

	if (status & PX4IO_P_STATUS_FLAGS_SAFETY_OFF) {
		safety.safety_off = true;
		safety.safety_switch_available = true;

	} else {
		safety.safety_off = false;
		safety.safety_switch_available = true;
	}

	/* lazily publish the safety status */
	if (_to_safety != NULL) {
		orb_publish(ORB_ID(safety), _to_safety, &safety);

	} else {
		_to_safety = orb_advertise(ORB_ID(safety), &safety);
	}

	return ret;
}

int RC_Driver::io_handle_alarms(uint16_t alarms)
{

	/* XXX handle alarms */


	/* set new alarms state */
	_alarms = alarms;

	return 0;
}

int RC_Driver::io_get_status()
{
	uint16_t	regs[6];
	int		ret;

	/* get
	 * STATUS_FLAGS, STATUS_ALARMS, STATUS_VBATT, STATUS_IBATT,
	 * STATUS_VSERVO, STATUS_VRSSI, STATUS_PRSSI
	 * in that order */
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, &regs[0], sizeof(regs) / sizeof(regs[0]));

	if (ret != OK) {
		return ret;
	}

	io_handle_status(regs[0]);
	io_handle_alarms(regs[1]);

	return ret;
}

int RC_Driver::io_disable_rc_handling()
{
	uint16_t set = PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED;
	uint16_t clear = 0;

	return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
}

int RC_Driver::io_set_rc_config()
{
	unsigned offset = 0;
	int input_map[_max_rc_input];
	int32_t ichan;
	int ret = OK;

	/*
	 * Generate the input channel -> control channel mapping table;
	 * assign RC_MAP_ROLL/PITCH/YAW/THROTTLE to the canonical
	 * controls.
	 */

	/* fill the mapping with an error condition triggering value */
	for (unsigned i = 0; i < _max_rc_input; i++) {
		input_map[i] = UINT8_MAX;
	}

	/*
	 * NOTE: The indices for mapped channels are 1-based
	 *       for compatibility reasons with existing
	 *       autopilots / GCS'.
	 */

	/* ROLL */
	param_get(param_find("RC_MAP_ROLL"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 0;
	}

	/* PITCH */
	param_get(param_find("RC_MAP_PITCH"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 1;
	}

	/* YAW */
	param_get(param_find("RC_MAP_YAW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 2;
	}

	/* THROTTLE */
	param_get(param_find("RC_MAP_THROTTLE"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 3;
	}

	/* FLAPS */
	param_get(param_find("RC_MAP_FLAPS"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 4;
	}

	/* AUX 1*/
	param_get(param_find("RC_MAP_AUX1"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 5;
	}

	/* AUX 2*/
	param_get(param_find("RC_MAP_AUX2"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 6;
	}

	/* AUX 3*/
	param_get(param_find("RC_MAP_AUX3"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		input_map[ichan - 1] = 7;
	}

	/* MAIN MODE SWITCH */
	param_get(param_find("RC_MAP_MODE_SW"), &ichan);

	if ((ichan > 0) && (ichan <= (int)_max_rc_input)) {
		/* use out of normal bounds index to indicate special channel */
		input_map[ichan - 1] = PX4IO_P_RC_CONFIG_ASSIGNMENT_MODESWITCH;
	}

	/*
	 * Iterate all possible RC inputs.
	 */
	for (unsigned i = 0; i < _max_rc_input; i++) {
		uint16_t regs[PX4IO_P_RC_CONFIG_STRIDE];
		char pname[16];
		float fval;

		/*
		 * RC params are floats, but do only
		 * contain integer values. Do not scale
		 * or cast them, let the auto-typeconversion
		 * do its job here.
		 * Channels: 500 - 2500
		 * Inverted flag: -1 (inverted) or 1 (normal)
		 */

		sprintf(pname, "RC%d_MIN", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MIN] = fval;

		sprintf(pname, "RC%d_TRIM", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_CENTER] = fval;

		sprintf(pname, "RC%d_MAX", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_MAX] = fval;

		sprintf(pname, "RC%d_DZ", i + 1);
		param_get(param_find(pname), &fval);
		regs[PX4IO_P_RC_CONFIG_DEADZONE] = fval;

		regs[PX4IO_P_RC_CONFIG_ASSIGNMENT] = input_map[i];

		regs[PX4IO_P_RC_CONFIG_OPTIONS] = PX4IO_P_RC_CONFIG_OPTIONS_ENABLED;
		sprintf(pname, "RC%d_REV", i + 1);
		param_get(param_find(pname), &fval);

		/*
		 * This has been taken for the sake of compatibility
		 * with APM's setup / mission planner: normal: 1,
		 * inverted: -1
		 */
		if (fval < 0) {
			regs[PX4IO_P_RC_CONFIG_OPTIONS] |= PX4IO_P_RC_CONFIG_OPTIONS_REVERSE;
		}

		/* send channel config to IO */
		ret = io_reg_set(PX4IO_PAGE_RC_CONFIG, offset, regs, PX4IO_P_RC_CONFIG_STRIDE);

		if (ret != OK) {
			printf("rc config upload failed");
			break;
		}

		/* check the IO initialisation flag */
		if (!(io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS) & PX4IO_P_STATUS_FLAGS_INIT_OK)) {
			printf("config for RC%d rejected by IO\n", i+1);
			//mavlink_and_console_log_critical(_mavlink_fd, "config for RC%d rejected by IO", i + 1);
			break;
		}

		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

int RC_Driver::io_set_control_groups()
{
	int ret = io_set_control_state(0);

	/* send auxiliary control groups */
	(void)io_set_control_state(1);
	(void)io_set_control_state(2);
	(void)io_set_control_state(3);

	return ret;
}

int RC_Driver::io_set_control_state(unsigned group)
{
	actuator_controls_s	controls;	///< actuator outputs
	uint16_t 		regs[_max_actuators];

	/* get controls */
	bool changed = false;

	switch (group) {
	case 0: {
			orb_check(_t_actuator_controls_0, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_0), _t_actuator_controls_0, &controls);
				//perf_set(_perf_sample_latency, hrt_elapsed_time(&controls.timestamp_sample));
			}
		}
		break;

	case 1: {
			orb_check(_t_actuator_controls_1, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_1), _t_actuator_controls_1, &controls);
			}
		}
		break;

	case 2: {
			orb_check(_t_actuator_controls_2, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_2), _t_actuator_controls_2, &controls);
			}
		}
		break;

	case 3: {
			orb_check(_t_actuator_controls_3, &changed);

			if (changed) {
				orb_copy(ORB_ID(actuator_controls_3), _t_actuator_controls_3, &controls);
			}
		}
		break;
	}

	if (!changed && (!_in_esc_calibration_mode || group != 0)) {
		return -1;

	} else if (_in_esc_calibration_mode && group == 0) {
		/* modify controls to get max pwm (full thrust) on every esc */
		memset(&controls, 0, sizeof(controls));

		/* set maximum thrust */
		controls.control[3] = 1.0f;
	}

	for (unsigned i = 0; i < _max_controls; i++) {

		/* ensure FLOAT_TO_REG does not produce an integer overflow */
		float ctrl = controls.control[i];

		if (ctrl < -1.0f) {
			ctrl = -1.0f;

		} else if (ctrl > 1.0f) {
			ctrl = 1.0f;
		}

		regs[i] = FLOAT_TO_REG(ctrl);
	}

	/* copy values to registers in IO */
	return io_reg_set(PX4IO_PAGE_CONTROLS, group * PX4IO_PROTOCOL_MAX_CONTROL_COUNT, regs, _max_controls);
}

int RC_Driver::io_get_raw_rc_input(rc_input_values &input_rc)
{
	uint32_t channel_count;
	int	ret;

	/* we don't have the status bits, so input_source has to be set elsewhere */
	input_rc.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

	const unsigned prolog = (PX4IO_P_RAW_RC_BASE - PX4IO_P_RAW_RC_COUNT);
	uint16_t regs[input_rc_s::RC_INPUT_MAX_CHANNELS + prolog];

	/*
	 * Read the channel count and the first 9 channels.
	 *
	 * This should be the common case (9 channel R/C control being a reasonable upper bound).
	 */
	ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_COUNT, &regs[0], prolog + 9);

	if (ret != OK) {
		return ret;
	}

	/*
	 * Get the channel count any any extra channels. This is no more expensive than reading the
	 * channel count once.
	 */
	channel_count = regs[PX4IO_P_RAW_RC_COUNT];
	//printf("channel_count : %d\n", channel_count);

	/* limit the channel count */
	if (channel_count > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		channel_count = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	_rc_chan_count = channel_count;

	input_rc.timestamp_publication = hrt_absolute_time();

	input_rc.rc_ppm_frame_length = regs[PX4IO_P_RAW_RC_DATA];
	input_rc.rssi = regs[PX4IO_P_RAW_RC_NRSSI];
	input_rc.rc_failsafe = (regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_FAILSAFE);
	input_rc.rc_lost = !(regs[PX4IO_P_RAW_RC_FLAGS] & PX4IO_P_RAW_RC_FLAGS_RC_OK);
	input_rc.rc_lost_frame_count = regs[PX4IO_P_RAW_LOST_FRAME_COUNT];
	input_rc.rc_total_frame_count = regs[PX4IO_P_RAW_FRAME_COUNT];
	input_rc.channel_count = channel_count;

	

	/* rc_lost has to be set before the call to this function */
	if (!input_rc.rc_lost && !input_rc.rc_failsafe) {
		_rc_last_valid = input_rc.timestamp_publication;
	}

	input_rc.timestamp_last_signal = _rc_last_valid;

	/* FIELDS NOT SET HERE */
	/* input_rc.input_source is set after this call XXX we might want to mirror the flags in the RC struct */

	if (channel_count > 9) {
		ret = io_reg_get(PX4IO_PAGE_RAW_RC_INPUT, PX4IO_P_RAW_RC_BASE + 9, &regs[prolog + 9], channel_count - 9);

		if (ret != OK) {
			return ret;
		}
	}

	/* last thing set are the actual channel values as 16 bit values */
	for (unsigned i = 0; i < channel_count; i++) {
		input_rc.values[i] = regs[prolog + i];
	}

	/* get RSSI from input channel */
	if (_rssi_pwm_chan > 0 && _rssi_pwm_chan <= input_rc_s::RC_INPUT_MAX_CHANNELS && _rssi_pwm_max - _rssi_pwm_min != 0) {
		int rssi = ((input_rc.values[_rssi_pwm_chan - 1] - _rssi_pwm_min) * 100) /
			   (_rssi_pwm_max - _rssi_pwm_min);
		rssi = rssi > 100 ? 100 : rssi;
		rssi = rssi < 0 ? 0 : rssi;
		input_rc.rssi = rssi;
	}

	return ret;
}

int RC_Driver::io_publish_raw_rc()
{

	/* fetch values from IO */
	rc_input_values	rc_val;

	/* set the RC status flag ORDER MATTERS! */
	rc_val.rc_lost = !(_status & PX4IO_P_STATUS_FLAGS_RC_OK);

	int ret = io_get_raw_rc_input(rc_val);

	if (ret != OK) {
		return ret;
	}

	/* sort out the source of the values */
	if (_status & PX4IO_P_STATUS_FLAGS_RC_PPM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_PPM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_DSM) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SPEKTRUM;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_SBUS) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_SBUS;

	} else if (_status & PX4IO_P_STATUS_FLAGS_RC_ST24) {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_PX4IO_ST24;

	} else {
		rc_val.input_source = input_rc_s::RC_INPUT_SOURCE_UNKNOWN;

		/* only keep publishing RC input if we ever got a valid input */
		if (_rc_last_valid == 0) {
			/* we have never seen valid RC signals, abort */
			return OK;
		}
	}

	/* lazily advertise on first publication */
	if (_to_input_rc == NULL) {
		_to_input_rc = orb_advertise(ORB_ID(input_rc), &rc_val);

	} else {
		orb_publish(ORB_ID(input_rc), _to_input_rc, &rc_val);
	}

	return OK;
}

int RC_Driver::io_publish_pwm_outputs()
{
	/* data we are going to fetch */
	actuator_outputs_s outputs = {};
	multirotor_motor_limits_s motor_limits;

	outputs.timestamp = hrt_absolute_time();

	/* get servo values from IO */
	uint16_t ctl[_max_actuators];
	int ret = io_reg_get(PX4IO_PAGE_SERVOS, 0, ctl, _max_actuators);

	if (ret != OK) {
		return ret;
	}

	/* convert from register format to float */
	for (unsigned i = 0; i < _max_actuators; i++) {
		outputs.output[i] = ctl[i];
	}

	outputs.noutputs = _max_actuators;

	/* lazily advertise on first publication */
	if (_to_outputs == NULL) {
		int instance;
		_to_outputs = orb_advertise_multi(ORB_ID(actuator_outputs),
						  &outputs, &instance, ORB_PRIO_MAX);

	} else {
		orb_publish(ORB_ID(actuator_outputs), _to_outputs, &outputs);
	}

	/* get mixer status flags from IO */
	uint16_t mixer_status;
	ret = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_MIXER, &mixer_status, sizeof(mixer_status) / sizeof(uint16_t));
	memcpy(&motor_limits, &mixer_status, sizeof(motor_limits));

	if (ret != OK) {
		return ret;
	}

	/* publish mixer status */
	if (_to_mixer_status == NULL) {
		_to_mixer_status = orb_advertise(ORB_ID(multirotor_motor_limits), &motor_limits);

	} else {
		orb_publish(ORB_ID(multirotor_motor_limits), _to_mixer_status, &motor_limits);
	}

	return OK;
}

int RC_Driver::io_set_arming_state()
{
	actuator_armed_s	armed;		///< system armed state
	vehicle_control_mode_s	control_mode;	///< vehicle_control_mode

	int have_armed = orb_copy(ORB_ID(actuator_armed), _t_actuator_armed, &armed);
	int have_control_mode = orb_copy(ORB_ID(vehicle_control_mode), _t_vehicle_control_mode, &control_mode);
	_in_esc_calibration_mode = armed.in_esc_calibration_mode;

	uint16_t set = 0;
	uint16_t clear = 0;

	if (have_armed == OK) {
		_in_esc_calibration_mode = armed.in_esc_calibration_mode;

		if (armed.armed || _in_esc_calibration_mode) {
			set |= PX4IO_P_SETUP_ARMING_FMU_ARMED;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FMU_ARMED;
		}

		if (armed.lockdown && !_lockdown_override) {
			set |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = true;

		} else if (!armed.lockdown && _lockdown_override) {
			clear |= PX4IO_P_SETUP_ARMING_LOCKDOWN;
			_lockdown_override = false;
		}

		/* Do not set failsafe if circuit breaker is enabled */
		if (armed.force_failsafe && !_cb_flighttermination) {
			set |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE;
		}

		// XXX this is for future support in the commander
		// but can be removed if unneeded
		// if (armed.termination_failsafe) {
		// 	set |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// } else {
		// 	clear |= PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE;
		// }

		if (armed.ready_to_arm) {
			set |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_IO_ARM_OK;
		}
	}

	if (have_control_mode == OK) {
		if (control_mode.flag_external_manual_override_ok) {
			set |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;

		} else {
			clear |= PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK;
		}
	}

	if (_last_written_arming_s != set || _last_written_arming_c != clear) {
		_last_written_arming_s = set;
		_last_written_arming_c = clear;
		return io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, clear, set);
	}

	return 0;
}

int RC_Driver::pwm_init()
{
	int ret = 0;
	unsigned alt_rate = 0;
	//uint32_t alt_channel_groups = 0;
	//bool alt_channels_set = false;
	uint32_t set_mask = 0;
	//unsigned group = 0;
	unsigned long channels = 0;
	unsigned single_ch = 0;
	//unsigned pwm_value = 0;

	// [jjh] param value [channels] [alt_rate] [group]
	channels = 1234;
	while ((single_ch = channels % 10)) {

		set_mask |= 1 << (single_ch - 1);
		channels /= 10;
	}
	alt_rate = 400;

	if( alt_rate > 0 )
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_ALTRATE, alt_rate);

	if (set_mask > 0)
	{
		/* blindly clear the PWM update alarm - might be set for some other reason */
		io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);

		/* attempt to set the rate map */
		io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_RATES, set_mask);

		/* check that the changes took */
		uint16_t alarms = io_reg_get(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS);

		if (alarms & PX4IO_P_STATUS_ALARMS_PWM_ERROR) {
			ret = -EINVAL;
			io_reg_set(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_ALARMS, PX4IO_P_STATUS_ALARMS_PWM_ERROR);
		}
	}

	/* assign alternate rate to channel groups */
	/*if (alt_channels_set) {
		uint32_t mask = 0;

		for (group = 0; group < 32; group++) {
			if ((1 << group) & alt_channel_groups) {
				uint32_t group_mask;

				ret = ioctl(fd, PWM_SERVO_GET_RATEGROUP(group), (unsigned long)&group_mask);

				if (ret != OK) {
					err(1, "PWM_SERVO_GET_RATEGROUP(%u)", group);
				}

				mask |= group_mask;
			}
		}
		ioctl(fd, PWM_SERVO_SET_SELECT_UPDATE_RATE, mask);
	}*/

	// [jjh] param value [disarm]
	struct pwm_output_values pwm_values;
	memset(&pwm_values, 0, sizeof(pwm_values));
	pwm_values.channel_count = _max_actuators;
	for (unsigned i = 0; i < _max_actuators; i++) {
		if (set_mask & 1 << i)
			pwm_values.values[i] = 900;
	}
	io_reg_set(PX4IO_PAGE_DISARMED_PWM, 0, pwm_values.values, pwm_values.channel_count);

	memset(&pwm_values, 0, sizeof(pwm_values));
	pwm_values.channel_count = _max_actuators;
	for (unsigned i = 0; i < _max_actuators; i++) {
		if (set_mask & 1 << i)
			pwm_values.values[i] = 1100;
	}
	io_reg_set(PX4IO_PAGE_CONTROL_MIN_PWM, 0, pwm_values.values, pwm_values.channel_count);

	memset(&pwm_values, 0, sizeof(pwm_values));
	pwm_values.channel_count = _max_actuators;
	for (unsigned i = 0; i < _max_actuators; i++) {
		if (set_mask & 1 << i)
			pwm_values.values[i] = 1950;
	}
	io_reg_set(PX4IO_PAGE_CONTROL_MAX_PWM, 0, pwm_values.values, pwm_values.channel_count);

	return ret;
}

int RC_Driver::mixer_send(const char *buf, unsigned buflen, unsigned retries)
{
	/* get debug level */
	int debuglevel = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_SET_DEBUG);

	uint8_t	frame[_max_transfer];

	do {

		px4io_mixdata *msg = (px4io_mixdata *)&frame[0];
		unsigned max_len = _max_transfer - sizeof(px4io_mixdata);

		msg->f2i_mixer_magic = F2I_MIXER_MAGIC;
		msg->action = F2I_MIXER_ACTION_RESET;

		do {
			unsigned count = buflen;

			if (count > max_len) {
				count = max_len;
			}

			if (count > 0) {
				memcpy(&msg->text[0], buf, count);
				buf += count;
				buflen -= count;

			} else {
				continue;
			}

			/*
			 * We have to send an even number of bytes.  This
			 * will only happen on the very last transfer of a
			 * mixer, and we are guaranteed that there will be
			 * space left to round up as _max_transfer will be
			 * even.
			 */
			unsigned total_len = sizeof(px4io_mixdata) + count;

			if (total_len % 2) {
				msg->text[count] = '\0';
				total_len++;
			}

			int ret;

			for (int i = 0; i < 30; i++) {
				/* failed, but give it a 2nd shot */
				ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, total_len / 2);

				if (ret) {
					usleep(333);

				} else {
					break;
				}
			}

			/* print mixer chunk */
			if (debuglevel > 5 || ret) {

				warnx("fmu sent: \"%s\"", msg->text);

				/* read IO's output */
				//print_debug();
			}

			if (ret) {
				DEVICE_LOG("mixer send error %d", ret);
				return ret;
			}

			msg->action = F2I_MIXER_ACTION_APPEND;

		} while (buflen > 0);

		int ret;

		/* send the closing newline */
		msg->text[0] = '\n';
		msg->text[1] = '\0';

		for (int i = 0; i < 30; i++) {
			/* failed, but give it a 2nd shot */
			ret = io_reg_set(PX4IO_PAGE_MIXERLOAD, 0, (uint16_t *)frame, (sizeof(px4io_mixdata) + 2) / 2);

			if (ret) {
				usleep(333);

			} else {
				break;
			}
		}

		if (ret == 0) {
			printf("mixer load succeed\n");
			/* success, exit */
			break;
		}

		retries--;

	} while (retries > 0);

	if (retries == 0) {
		printf("mixer load failed\n");
		//mavlink_and_console_log_info(_mavlink_fd, "[IO] mixer upload fail");
		/* load must have failed for some reason */
		return -EINVAL;

	} else {
		/* all went well, set the mixer ok flag */
		return io_reg_modify(PX4IO_PAGE_STATUS, PX4IO_P_STATUS_FLAGS, 0, PX4IO_P_STATUS_FLAGS_MIXER_OK);
	}
}

int RC_Driver::load_mixer_file(const char *fname, char *buf, unsigned maxlen)
{
	FILE		*fp;
	char		line[120];

	/* open the mixer definition file */
	fp = fopen(fname, "r");

	if (fp == NULL) {
		warnx("file not found");
		return -1;
	}

	/* read valid lines from the file into a buffer */
	buf[0] = '\0';

	for (;;) {

		/* get a line, bail on error/EOF */
		line[0] = '\0';

		if (fgets(line, sizeof(line), fp) == NULL) {
			break;
		}

		/* if the line doesn't look like a mixer definition line, skip it */
		if ((strlen(line) < 2) || !isupper(line[0]) || (line[1] != ':')) {
			continue;
		}

		/* compact whitespace in the buffer */
		char *t, *f;

		for (f = line; *f != '\0'; f++) {
			/* scan for space characters */
			if (*f == ' ') {
				/* look for additional spaces */
				t = f + 1;

				while (*t == ' ') {
					t++;
				}

				if (*t == '\0') {
					/* strip trailing whitespace */
					*f = '\0';

				} else if (t > (f + 1)) {
					memmove(f + 1, t, strlen(t) + 1);
				}
			}
		}

		/* if the line is too long to fit in the buffer, bail */
		if ((strlen(line) + strlen(buf) + 1) >= maxlen) {
			warnx("line too long");
			fclose(fp);
			return -1;
		}

		/* add the line to the buffer */
		strcat(buf, line);
	}

	fclose(fp);
	return 0;
}

int RC_Driver::mixer_load(const char *fname)
{
	char buf[2048];

	if (load_mixer_file(fname, &buf[0], sizeof(buf)) < 0) {
		printf("can't load mixer: %s", fname);
		return 1;
	}

	int ret = mixer_send(buf, strnlen(buf, 2048));
	/* Pass the buffer to the device */

	if (ret < 0) {
		printf("error loading mixers from %s", fname);
		return 1;
	}

	return 0;
}


int RC_Driver::task_main()
{

	int ret;
	ret = VDev::init();
	if (ret != OK) {
		warnx("VDev setup for ts7500 io device failed!\n");
	}
	_ts7500io_fd = px4_open(TS7500IO_DEVICE_PATH, 0);

	hrt_abstime poll_last = 0;
	hrt_abstime orb_check_last = 0;

	//_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);

	/*
	 * Subscribe to the appropriate PWM output topic based on whether we are the
	 * primary PWM output or not.
	 */
	_t_actuator_controls_0 = orb_subscribe(ORB_ID(actuator_controls_0));
	orb_set_interval(_t_actuator_controls_0, 20);		/* default to 50Hz */
	_t_actuator_controls_1 = orb_subscribe(ORB_ID(actuator_controls_1));
	orb_set_interval(_t_actuator_controls_1, 33);		/* default to 30Hz */
	_t_actuator_controls_2 = orb_subscribe(ORB_ID(actuator_controls_2));
	orb_set_interval(_t_actuator_controls_2, 33);		/* default to 30Hz */
	_t_actuator_controls_3 = orb_subscribe(ORB_ID(actuator_controls_3));
	orb_set_interval(_t_actuator_controls_3, 33);		/* default to 30Hz */
	_t_actuator_armed = orb_subscribe(ORB_ID(actuator_armed));
	_t_vehicle_control_mode = orb_subscribe(ORB_ID(vehicle_control_mode));
	_t_param = orb_subscribe(ORB_ID(parameter_update));
	_t_vehicle_command = orb_subscribe(ORB_ID(vehicle_command));

	if ((_t_actuator_controls_0 < 0) ||
	    (_t_actuator_armed < 0) ||
	    (_t_vehicle_control_mode < 0) ||
	    (_t_param < 0) ||
	    (_t_vehicle_command < 0)) {
		warnx("subscription(s) failed");
		warnx("exiting");

		_task = -1;
		return -1;
	}

	/* Fetch initial flight termination circuit breaker state */
	_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);

	printf("initializing pwm...\n");
	pwm_init();

	printf("loading mixer...\n");
	mixer_load("quad_x.main.mix");

	/* poll descriptor */
	px4_pollfd_struct_t fds[1] = {};
	fds[0].fd = _t_actuator_controls_0;
	fds[0].events = POLLIN;

	_param_update_force = true;

	while(!_task_should_exit)
	{
		/* adjust update interval */
		if (_update_interval != 0) {
			if (_update_interval < UPDATE_INTERVAL_MIN) {
				_update_interval = UPDATE_INTERVAL_MIN;
			}

			if (_update_interval > 100) {
				_update_interval = 100;
			}

			orb_set_interval(_t_actuator_controls_0, _update_interval);
			/*
			 * NOT changing the rate of groups 1-3 here, because only attitude
			 * really needs to run fast.
			 */
			_update_interval = 0;
		}

		int ret = px4_poll(fds, 1, 20);

		if (ret < 0) {
			warnx("poll error %d", errno);
			continue;
		}

		hrt_abstime now = hrt_absolute_time();

		if (fds[0].revents & POLLIN) {

			(void)io_set_control_groups();
		}

		if (now >= poll_last + IO_POLL_INTERVAL) {
			poll_last = now;

			io_get_status();

			io_publish_raw_rc();

			io_publish_pwm_outputs();

			bool updated = false;

			orb_check(_t_actuator_armed, &updated);

			if (!updated) {
				orb_check(_t_vehicle_control_mode, &updated);
			}

			if (updated) {
				io_set_arming_state();
			}
		}

		if (now >= orb_check_last + ORB_CHECK_INTERVAL) {
			orb_check_last = now;

			if (_mavlink_fd < 0) {
				_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
			}

			bool updated = false;

			orb_check(_t_vehicle_command, &updated);

			if (updated) {
				struct vehicle_command_s cmd;
				orb_copy(ORB_ID(vehicle_command), _t_vehicle_command, &cmd);
			}

			orb_check(_t_param, &updated);

			if (updated || _param_update_force) {
				_param_update_force = false;
				parameter_update_s pupdate;
				orb_copy(ORB_ID(parameter_update), _t_param, &pupdate);

				io_set_rc_config();

				int pret;
				int32_t failsafe_param_val;
				param_t failsafe_param = param_find("RC_FAILS_THR");

				if (failsafe_param != PARAM_INVALID) {

					param_get(failsafe_param, &failsafe_param_val);

					if (failsafe_param_val > 0) {

						uint16_t failsafe_thr = failsafe_param_val;
						pret = io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_RC_THR_FAILSAFE_US, &failsafe_thr, 1);

						if (pret != OK) {
							mavlink_and_console_log_critical(_mavlink_fd, "failsafe upload failed, FS: %d us", (int)failsafe_thr);
						}
					}
				}

				int32_t safety_param_val;
				param_t safety_param = param_find("RC_FAILS_THR");

				if (safety_param != PARAM_INVALID) {

					param_get(safety_param, &safety_param_val);

					if (safety_param_val == PX4IO_FORCE_SAFETY_MAGIC) {
						(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, safety_param_val);
					}
				}

				_cb_flighttermination = circuit_breaker_enabled("CBRK_FLIGHTTERM", CBRK_FLIGHTTERM_KEY);

				param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
				param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
				param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);

				int16_t pwm_invert_mask = 0;

				for (unsigned i = 0; i < _max_actuators; i++) {
					char pname[16];
					int32_t ival;

					sprintf(pname, "PWM_MAIN_REV%d", i + 1);
					param_t param_h = param_find(pname);

					if (param_h != PARAM_INVALID) {
						param_get(param_h, &ival);
						pwm_invert_mask |= ((int16_t)(ival != 0)) << i;
					}
				}

				(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_PWM_REVERSE, pwm_invert_mask);

				float trim_val;
				param_t parm_handle;

				parm_handle = param_find("TRIM_ROLL");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &trim_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_ROLL, FLOAT_TO_REG(trim_val));
				}

				parm_handle = param_find("TRIM_PITCH");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &trim_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_PITCH, FLOAT_TO_REG(trim_val));
				}

				parm_handle = param_find("TRIM_YAW");

				if (parm_handle != PARAM_INVALID) {
					param_get(parm_handle, &trim_val);
					(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_TRIM_YAW, FLOAT_TO_REG(trim_val));
				}

			}

		}

	}

	warnx("exiting");

	/* clean up the alternate device node */
	/*if (_primary_pwm_device) {
		delete PWM_dev;
	}*/

	/* tell the dtor that we are exiting */
	_task = -1;
	return -1;

}

void RC_Driver::task_main_trampoline(int argc, char *argv[])
{
	rc_dev->task_main();
}

int RC_Driver::init(bool rc_handling_disabled)
{
	_rc_handling_disabled = rc_handling_disabled;
	return init();
}

int RC_Driver::init()
{

	int ret;
	param_t sys_restart_param;
	int32_t sys_restart_val = DM_INIT_REASON_VOLATILE;

	ASSERT(_task == -1);

	sys_restart_param = param_find("SYS_RESTART_TYPE");

	if (sys_restart_param != PARAM_INVALID) {
		/* Indicate restart type is unknown */
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != DM_INIT_REASON_POWER_ON) {
			param_set_no_notification(sys_restart_param, &sys_restart_val);
		}
	}


	_mavlink_fd = px4_open(MAVLINK_LOG_DEVICE, 0);
	/* get some parameters */
	unsigned protocol;
	hrt_abstime start_try_time = hrt_absolute_time();

	do {
		usleep(2000);
		protocol = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_PROTOCOL_VERSION);
	} while (protocol == _io_reg_get_error && (hrt_elapsed_time(&start_try_time) < 700U * 1000U));

	/* if the error still persists after timing out, we give up */
	if (protocol == _io_reg_get_error) {
		mavlink_and_console_log_emergency(_mavlink_fd, "Failed to communicate with IO, abort.");
		return -1;
	}

	if (protocol != PX4IO_PROTOCOL_VERSION) {
		mavlink_and_console_log_emergency(_mavlink_fd, "IO protocol/firmware mismatch, abort.");
		return -1;
	}


	_hardware      = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_HARDWARE_VERSION);
	_max_actuators = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_ACTUATOR_COUNT);
	_max_controls  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_CONTROL_COUNT);
	_max_relays    = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RELAY_COUNT);
	_max_transfer  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_MAX_TRANSFER) - 2;
	_max_rc_input  = io_reg_get(PX4IO_PAGE_CONFIG, PX4IO_P_CONFIG_RC_INPUT_COUNT);

	if ((_max_actuators < 1) || (_max_actuators > 255) ||
	    (_max_relays > 32)   ||
	    (_max_transfer < 16) || (_max_transfer > 255)  ||
	    (_max_rc_input < 1)  || (_max_rc_input > 255)) {

		mavlink_log_emergency(_mavlink_fd, "[IO] config read fail, abort.");
		return -1;
	}

	if (_max_rc_input > input_rc_s::RC_INPUT_MAX_CHANNELS) {
		_max_rc_input = input_rc_s::RC_INPUT_MAX_CHANNELS;
	}

	param_get(param_find("RC_RSSI_PWM_CHAN"), &_rssi_pwm_chan);
	param_get(param_find("RC_RSSI_PWM_MAX"), &_rssi_pwm_max);
	param_get(param_find("RC_RSSI_PWM_MIN"), &_rssi_pwm_min);


	/*
	 * Check for IO flight state - if FMU was flagged to be in
	 * armed state, FMU is recovering from an in-air reset.
	 * Read back status and request the commander to arm
	 * in this case.
	 */

	uint16_t reg;

	/* get IO's last seen FMU state */
	ret = io_reg_get(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING, &reg, sizeof(reg));

	if (ret != OK) {
		return ret;
	}

	/*
	 * in-air restart is only tried if the IO board reports it is
	 * already armed, and has been configured for in-air restart
	 */
	if ((reg & PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK) &&
	    (reg & PX4IO_P_SETUP_ARMING_FMU_ARMED)) {

		/* get a status update from IO */
		io_get_status();

		mavlink_and_console_log_emergency(_mavlink_fd, "RECOVERING FROM FMU IN-AIR RESTART");

		/* WARNING: COMMANDER app/vehicle status must be initialized.
		 * If this fails (or the app is not started), worst-case IO
		 * remains untouched (so manual override is still available).
		 */

		int safety_sub = orb_subscribe(ORB_ID(actuator_armed));
		/* fill with initial values, clear updated flag */
		struct actuator_armed_s safety;
		uint64_t try_start_time = hrt_absolute_time();
		bool updated = false;

		/* keep checking for an update, ensure we got a arming information,
		   not something that was published a long time ago. */
		do {
			orb_check(safety_sub, &updated);

			if (updated) {
				/* got data, copy and exit loop */
				orb_copy(ORB_ID(actuator_armed), safety_sub, &safety);
				break;
			}

			/* wait 10 ms */
			usleep(10000);

			/* abort after 5s */
			if ((hrt_absolute_time() - try_start_time) / 1000 > 3000) {
				mavlink_and_console_log_emergency(_mavlink_fd, "Failed to recover from in-air restart (1), abort");
				return 1;
			}

		} while (true);

		/* send command to arm system via command API */
		vehicle_command_s cmd;
		/* send this to itself */
		param_t sys_id_param = param_find("MAV_SYS_ID");
		param_t comp_id_param = param_find("MAV_COMP_ID");

		int32_t sys_id;
		int32_t comp_id;

		if (param_get(sys_id_param, &sys_id)) {
			errx(1, "PRM SYSID");
		}

		if (param_get(comp_id_param, &comp_id)) {
			errx(1, "PRM CMPID");
		}

		cmd.target_system = sys_id;
		cmd.target_component = comp_id;
		cmd.source_system = sys_id;
		cmd.source_component = comp_id;
		/* request arming */
		cmd.param1 = 1.0f;
		cmd.param2 = 0;
		cmd.param3 = 0;
		cmd.param4 = 0;
		cmd.param5 = 0;
		cmd.param6 = 0;
		cmd.param7 = 0;
		cmd.command = vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM;

		/* ask to confirm command */
		cmd.confirmation =  1;

		/* send command once */
		orb_advert_t pub = orb_advertise(ORB_ID(vehicle_command), &cmd);

		/* spin here until IO's state has propagated into the system */
		do {
			orb_check(safety_sub, &updated);

			if (updated) {
				orb_copy(ORB_ID(actuator_armed), safety_sub, &safety);
			}

			/* wait 50 ms */
			usleep(50000);

			/* abort after 5s */
			if ((hrt_absolute_time() - try_start_time) / 1000 > 2000) {
				mavlink_and_console_log_emergency(_mavlink_fd, "Failed to recover from in-air restart (2), abort");
				return 1;
			}

			/* re-send if necessary */
			if (!safety.armed) {
				orb_publish(ORB_ID(vehicle_command), pub, &cmd);
				printf("re-sending arm cmd");
			}

			/* keep waiting for state change for 2 s */
		} while (!safety.armed);

		/* Indicate restart type is in-flight */
		sys_restart_val = DM_INIT_REASON_IN_FLIGHT;
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);
		}

		/* regular boot, no in-air restart, init IO */

	} else {

		/* dis-arm IO before touching anything */
		io_reg_modify(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_ARMING,
			      PX4IO_P_SETUP_ARMING_FMU_ARMED |
			      PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK |
			      PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK |
			      PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE |
			      PX4IO_P_SETUP_ARMING_LOCKDOWN, 0);

		if (_rc_handling_disabled) {
			ret = io_disable_rc_handling();

			if (ret != OK) {
				printf("failed disabling RC handling");
				return ret;
			}

		} else {
			/* publish RC config to IO */
			ret = io_set_rc_config();

			if (ret != OK) {
				mavlink_and_console_log_critical(_mavlink_fd, "IO RC config upload fail");
				return ret;
			}
		}

		/* Indicate restart type is power on */
		sys_restart_val = DM_INIT_REASON_POWER_ON;
		int32_t prev_val;
		param_get(sys_restart_param, &prev_val);

		if (prev_val != sys_restart_val) {
			param_set(sys_restart_param, &sys_restart_val);
		}

	}

	/* set safety to off if circuit breaker enabled */
	if (circuit_breaker_enabled("CBRK_IO_SAFETY", CBRK_IO_SAFETY_KEY)) {
		(void)io_reg_set(PX4IO_PAGE_SETUP, PX4IO_P_SETUP_FORCE_SAFETY_OFF, PX4IO_FORCE_SAFETY_MAGIC);
	}

	// start the IO interface task
	_task = px4_task_spawn_cmd("ts7500io",
				   SCHED_DEFAULT,
				   SCHED_PRIORITY_ACTUATOR_OUTPUTS,
				   1500,
				   (px4_main_t)&RC_Driver::task_main_trampoline,
				   NULL);

	if (_task < 0) {
		//printf("task start failed: %d", errno);
		printf("task start failed: %d", errno);
		return -errno;
	}

	return OK;
}

extern "C" __EXPORT int ts7500io_main(int argc, char *argv[]);

int ts7500io_main(int argc, char *argv[])
{

	if (argc < 2) {
		warnx("usage: ts7500io {start|stop|status}");
		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (rc_dev != NULL) {
			warnx("already running");
			return 0;
		}

		rc_dev = new RC_Driver;

		if (rc_dev == NULL) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != rc_dev->init()) {
			delete rc_dev;
			rc_dev = NULL;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (rc_dev == NULL) {
			warnx("not running");
			return 1;
		}

		delete rc_dev;
		rc_dev = NULL;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (rc_dev) {
			warnx("is running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}

