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

#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>

#include <systemlib/param/param.h>
#include <systemlib/circuit_breaker.h>

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

#include "sbus.h"
#include "virtual_register.h"

/*#include <crc32.h>

#include <arch/board/board.h>

#include <drivers/device/device.h>
#include <drivers/drv_rc_input.h>
#include <drivers/drv_pwm_output.h>
#include <drivers/drv_sbus.h>
#include <drivers/drv_gpio.h>
#include <drivers/drv_mixer.h>

#include <systemlib/mixer/mixer.h>
#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/scheduling_priorities.h>

#include <modules/px4iofirmware/protocol.h>

#include "uploader.h"


#include "px4io_driver.h"

#define PX4IO_SET_DEBUG			_IOC(0xff00, 0)
#define PX4IO_INAIR_RESTART_ENABLE	_IOC(0xff00, 1)
#define PX4IO_REBOOT_BOOTLOADER		_IOC(0xff00, 2)
#define PX4IO_CHECK_CRC			_IOC(0xff00, 3)

#define UPDATE_INTERVAL_MIN		2			// 2 ms	-> 500 Hz
#define ORB_CHECK_INTERVAL		200000		// 200 ms -> 5 Hz
#define IO_POLL_INTERVAL		20000		// 20 ms -> 50 Hz
*/


#define SCHED_PRIORITY_ACTUATOR_OUTPUTS     (SCHED_PRIORITY_MAX - 15)

class RC_Driver
{
public:

	RC_Driver();

	~RC_Driver();

	virtual int start();

	int start(bool disable_rc_handling);

	/**
	 * Disable RC input handling
	 */
	int			disable_rc_handling();


private:

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

	int			_mavlink_fd;		///< mavlink file descriptor.

	uint16_t		_status;		///< Various IO status flags
	uint16_t		_alarms;		///< Various IO alarms

	orb_advert_t 		_to_input_rc;		///< rc inputs from io
	orb_advert_t		_to_safety;		///< status of safety

	int32_t			_rssi_pwm_chan; ///< RSSI PWM input channel
	int32_t			_rssi_pwm_max; ///< max RSSI input on PWM channel
	int32_t			_rssi_pwm_min; ///< min RSSI input on PWM channel

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



	static void task_main_trampoline(int argc, char *argv[]);

	void task_main();

};

RC_Driver *rc_dev = NULL;

RC_Driver::RC_Driver()
{
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
	_mavlink_fd = -1;
	_status = 0;
	_alarms = 0;
	_to_input_rc = NULL;
	_to_safety = NULL;
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

int RC_Driver::io_reg_get(uint8_t page, uint8_t offset, uint16_t *values, unsigned num_values)
{
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		printf("io_reg_get: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	//int ret = _interface->read((page << 8) | offset, reinterpret_cast<void *>(values), num_values);

	int i;
	if( page == 0 )
	{
		sbuslock();
		for( i = 0; i < num_values; i++ )
			*(values + i) = sbus_peek16((page << 8) | offset + i*sizeof(*values));
		sbusunlock();
	}
	else
	{
		//virtual_lock();
		for( i = 0; i < num_values; i+=sizeof(values) )
			*(values + i) = get_virtual_reg(page, offset + i);
		//virtual_unlock();
	}

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
	/* range check the transfer */
	if (num_values > ((_max_transfer) / sizeof(*values))) {
		printf("io_reg_set: too many registers (%u, max %u)", num_values, _max_transfer / 2);
		return -EINVAL;
	}

	//int ret =  _interface->write((page << 8) | offset, (void *)values, num_values);

	int i;
	if( page == 0 )
	{
		sbuslock();
		for( i = 0; i < num_values; i++ )
			sbus_poke16((page << 8) | offset + i*sizeof(*values), *(values + i));
		sbusunlock();
	}
	else
	{
		//virtual_lock();
		for( i = 0; i < num_values; i+=sizeof(values) )
			set_virtual_reg(page, offset + i, *(values + i));
		//virtual_unlock();
	}

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
			mavlink_and_console_log_critical(_mavlink_fd, "config for RC%d rejected by IO", i + 1);
			break;
		}

		offset += PX4IO_P_RC_CONFIG_STRIDE;
	}

	return ret;
}

void RC_Driver::task_main()
{

	hrt_abstime poll_last = 0;
	hrt_abstime orb_check_last = 0;

	_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);

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
		goto out;
	}

	while(!_task_should_exit)
	{
		/****************************************/
		//io_reg_get();
		sleep(1);
	}

out:
	DEVICE_DEBUG("exiting");

	/* clean up the alternate device node */
	if (_primary_pwm_device) {
		unregister_driver(PWM_OUTPUT0_DEVICE_PATH);
	}

	/* tell the dtor that we are exiting */
	_task = -1;
	_exit(0);

}

void RC_Driver::task_main_trampoline(int argc, char *argv[])
{
	rc_dev->task_main();
}

int RC_Driver::start(bool rc_handling_disabled)
{
	_rc_handling_disabled = rc_handling_disabled;
	return start();
}

int RC_Driver::start()
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


	_mavlink_fd = ::open(MAVLINK_LOG_DEVICE, 0);
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

		printf("config read error");
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

		if (OK != rc_dev->start()) {
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

