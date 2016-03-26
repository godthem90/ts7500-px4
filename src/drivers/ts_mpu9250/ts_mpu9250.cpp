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
 * @file mpu9250.cpp
 *
 * Driver for the Invensense MPU9250 connected via SPI.
 *
 * @author Andrew Tridgell
 *
 * based on the mpu6000 driver
 */

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <getopt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>
#include <systemlib/conversions.h>

#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_gyro.h>
#include <drivers/drv_mag.h>
#include <lib/conversion/rotation.h>
//#include <mathlib/math/filter/LowPassFilter2p.hpp>

#define DIR_READ			0x80
#define DIR_WRITE			0x00

#define MPU_DEVICE_PATH_ACCEL		"/dev/mpu9250_accel"
#define MPU_DEVICE_PATH_GYRO		"/dev/mpu9250_gyro"
#define MPU_DEVICE_PATH_ACCEL_EXT	"/dev/mpu9250_accel_ext"
#define MPU_DEVICE_PATH_GYRO_EXT	"/dev/mpu9250_gyro_ext"

// MPU 9250 registers
#define MPUREG_WHOAMI			0x75
#define MPUREG_SMPLRT_DIV		0x19
#define MPUREG_CONFIG			0x1A
#define MPUREG_GYRO_CONFIG		0x1B
#define MPUREG_ACCEL_CONFIG		0x1C
#define MPUREG_ACCEL_CONFIG2		0x1D
#define MPUREG_LPACCEL_ODR		0x1E
#define MPUREG_WOM_THRESH		0x1F
#define MPUREG_FIFO_EN			0x23
#define MPUREG_I2C_MST_CTRL		0x24
#define MPUREG_I2C_SLV0_ADDR		0x25
#define MPUREG_I2C_SLV0_REG		0x26
#define MPUREG_I2C_SLV0_CTRL		0x27
#define MPUREG_I2C_SLV1_ADDR		0x28
#define MPUREG_I2C_SLV1_REG		0x29
#define MPUREG_I2C_SLV1_CTRL		0x2A
#define MPUREG_I2C_SLV2_ADDR		0x2B
#define MPUREG_I2C_SLV2_REG		0x2C
#define MPUREG_I2C_SLV2_CTRL		0x2D
#define MPUREG_I2C_SLV3_ADDR		0x2E
#define MPUREG_I2C_SLV3_REG		0x2F
#define MPUREG_I2C_SLV3_CTRL		0x30
#define MPUREG_I2C_SLV4_ADDR		0x31
#define MPUREG_I2C_SLV4_REG		0x32
#define MPUREG_I2C_SLV4_DO		0x33
#define MPUREG_I2C_SLV4_CTRL		0x34
#define MPUREG_I2C_SLV4_DI		0x35
#define MPUREG_I2C_MST_STATUS		0x36
#define MPUREG_INT_PIN_CFG		0x37
#define MPUREG_INT_ENABLE		0x38
#define MPUREG_INT_STATUS		0x3A
#define MPUREG_ACCEL_XOUT_H		0x3B
#define MPUREG_ACCEL_XOUT_L		0x3C
#define MPUREG_ACCEL_YOUT_H		0x3D
#define MPUREG_ACCEL_YOUT_L		0x3E
#define MPUREG_ACCEL_ZOUT_H		0x3F
#define MPUREG_ACCEL_ZOUT_L		0x40
#define MPUREG_TEMP_OUT_H		0x41
#define MPUREG_TEMP_OUT_L		0x42
#define MPUREG_GYRO_XOUT_H		0x43
#define MPUREG_GYRO_XOUT_L		0x44
#define MPUREG_GYRO_YOUT_H		0x45
#define MPUREG_GYRO_YOUT_L		0x46
#define MPUREG_GYRO_ZOUT_H		0x47
#define MPUREG_GYRO_ZOUT_L		0x48
#define MPUREG_EXT_SENS_DATA_00		0x49
#define MPUREG_I2C_SLV0_D0		0x63
#define MPUREG_I2C_SLV1_D0		0x64
#define MPUREG_I2C_SLV2_D0		0x65
#define MPUREG_I2C_SLV3_D0		0x66
#define MPUREG_I2C_MST_DELAY_CTRL	0x67
#define MPUREG_SIGNAL_PATH_RESET	0x68
#define MPUREG_MOT_DETECT_CTRL		0x69
#define MPUREG_USER_CTRL		0x6A
#define MPUREG_PWR_MGMT_1		0x6B
#define MPUREG_PWR_MGMT_2		0x6C
#define MPUREG_FIFO_COUNTH		0x72
#define MPUREG_FIFO_COUNTL		0x73
#define MPUREG_FIFO_R_W			0x74

// Configuration bits MPU 9250
#define BIT_SLEEP			0x40
#define BIT_H_RESET			0x80
#define MPU_CLK_SEL_AUTO		0x01

#define BITS_GYRO_ST_X			0x80
#define BITS_GYRO_ST_Y			0x40
#define BITS_GYRO_ST_Z			0x20
#define BITS_FS_250DPS			0x00
#define BITS_FS_500DPS			0x08
#define BITS_FS_1000DPS			0x10
#define BITS_FS_2000DPS			0x18
#define BITS_FS_MASK			0x18

#define BITS_DLPF_CFG_250HZ		0x00
#define BITS_DLPF_CFG_184HZ		0x01
#define BITS_DLPF_CFG_92HZ		0x02
#define BITS_DLPF_CFG_41HZ		0x03
#define BITS_DLPF_CFG_20HZ		0x04
#define BITS_DLPF_CFG_10HZ		0x05
#define BITS_DLPF_CFG_5HZ		0x06
#define BITS_DLPF_CFG_3600HZ		0x07
#define BITS_DLPF_CFG_MASK		0x07

#define BIT_RAW_RDY_EN			0x01
#define BIT_INT_ANYRD_2CLEAR		0x10

#define MPU_WHOAMI_9250			0x71

#define MPU9250_ACCEL_DEFAULT_RATE	1000
#define MPU9250_ACCEL_MAX_OUTPUT_RATE			280
#define MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ 30
#define MPU9250_GYRO_DEFAULT_RATE	1000
/* rates need to be the same between accel and gyro */
#define MPU9250_GYRO_MAX_OUTPUT_RATE			MPU9250_ACCEL_MAX_OUTPUT_RATE
#define MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ 30

#define MPU9250_DEFAULT_ONCHIP_FILTER_FREQ	41

#define MPU9250_ONE_G					9.80665f

#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif

/*
  the MPU9250 can only handle high SPI bus speeds on the sensor and
  interrupt status registers. All other registers have a maximum 1MHz
  SPI speed
 */
#define MPU9250_LOW_BUS_SPEED				1000*1000
#define MPU9250_HIGH_BUS_SPEED				11*1000*1000

/*
  we set the timer interrupt to run a bit faster than the desired
  sample rate and then throw away duplicates by comparing
  accelerometer values. This time reduction is enough to cope with
  worst case timing jitter due to other timers
 */
#define MPU9250_TIMER_REDUCTION				200

#define MPU9250_MAG_DEFAULT_RATE			100

class LowPassFilter2p
{
public:
    LowPassFilter2p(float sample_freq, float cutoff_freq); 
     
    void set_cutoff_frequency(float sample_freq, float cutoff_freq);

    float apply(float sample);

    float get_cutoff_freq(void) const {
        return _cutoff_freq;
    }

    float reset(float sample);

private:
    float           _cutoff_freq; 
    float           _a1;
    float           _a2;
    float           _b0;
    float           _b1;
    float           _b2;
    float           _delay_element_1;        // buffered sample -1
    float           _delay_element_2;        // buffered sample -2
};

LowPassFilter2p::LowPassFilter2p(float sample_freq, float cutoff_freq) : 
	_cutoff_freq(cutoff_freq),
	_a1(0.0f),
	_a2(0.0f),
	_b0(0.0f),
	_b1(0.0f),
	_b2(0.0f),
	_delay_element_1(0.0f),
	_delay_element_2(0.0f)
{
	// set initial parameters
	set_cutoff_frequency(sample_freq, cutoff_freq);
}


void LowPassFilter2p::set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
    _cutoff_freq = cutoff_freq;
    if (_cutoff_freq <= 0.0f) {
        // no filtering
        return;
    }
    float fr = sample_freq/_cutoff_freq;
    float ohm = tanf(M_PI_F/fr);
    float c = 1.0f+2.0f*cosf(M_PI_F/4.0f)*ohm + ohm*ohm;
    _b0 = ohm*ohm/c;
    _b1 = 2.0f*_b0;
    _b2 = _b0;
    _a1 = 2.0f*(ohm*ohm-1.0f)/c;
    _a2 = (1.0f-2.0f*cosf(M_PI_F/4.0f)*ohm+ohm*ohm)/c;
}

float LowPassFilter2p::apply(float sample)
{
    if (_cutoff_freq <= 0.0f) {
        // no filtering
        return sample;
    }

    // do the filtering
    float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;
    if (!PX4_ISFINITE(delay_element_0)) {
        // don't allow bad values to propagate via the filter
        delay_element_0 = sample;
    }
    float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
    
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    // return the value.  Should be no need to check limits
    return output;
}

float LowPassFilter2p::reset(float sample) {
	float dval = sample / (_b0 + _b1 + _b2);
    _delay_element_1 = dval;
    _delay_element_2 = dval;
    return apply(sample);
}


class MPU9250
{
public:
	MPU9250(const char *path_accel, const char *path_gyro, enum Rotation rotation);
	virtual ~MPU9250();

	int		init();

	ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	int		mpu9250_ioctl(int cmd, unsigned long arg);

private:

	struct hrt_call		_call;
	unsigned		_call_interval;

	ringbuffer::RingBuffer	*_accel_reports;
	struct accel_scale	_accel_scale;
	float			_accel_range_scale;
	float			_accel_range_m_s2;
	orb_advert_t		_accel_topic;
	int			_accel_orb_class_instance;
	int			_accel_class_instance;

	ringbuffer::RingBuffer	*_gyro_reports;
	struct gyro_scale	_gyro_scale;
	float			_gyro_range_scale;
	float			_gyro_range_rad_s;
	orb_advert_t		_gyro_topic;
	int			_gyro_orb_class_instance;
	int			_gyro_class_instance;

	ringbuffer::RingBuffer	*_mag_reports;
	struct mag_scale		_mag_scale;
	float 			_mag_range_scale;
	float 			_mag_range_ga;
	bool			_mag_collect_phase;
	orb_advert_t		_mag_topic;
	int			_mag_class_instance;
	int			_mag_orb_class_instance;


	unsigned		_dlpf_freq;
	unsigned		_sample_rate;

	uint8_t			_range_bits;

	perf_counter_t		_accel_reads;
	perf_counter_t		_gyro_reads;
	perf_counter_t		_sample_perf;
	perf_counter_t		_bad_transfers;
	perf_counter_t		_bad_registers;
	perf_counter_t		_good_transfers;
	perf_counter_t		_reset_retries;
	perf_counter_t		_duplicates;
	perf_counter_t		_controller_latency_perf;

	uint8_t			_register_wait;
	uint64_t		_reset_wait;

	LowPassFilter2p	_accel_filter_x;
	LowPassFilter2p	_accel_filter_y;
	LowPassFilter2p	_accel_filter_z;
	LowPassFilter2p	_gyro_filter_x;
	LowPassFilter2p	_gyro_filter_y;
	LowPassFilter2p	_gyro_filter_z;
	LowPassFilter2p	_mag_filter_x;
	LowPassFilter2p	_mag_filter_y;
	LowPassFilter2p	_mag_filter_z;

	Integrator		_accel_int;
	Integrator		_gyro_int;

	enum Rotation		_rotation;

	// this is used to support runtime checking of key
	// configuration registers to detect SPI bus errors and sensor
	// reset
#define MPU9250_NUM_CHECKED_REGISTERS 11
	static const uint8_t	_checked_registers[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_values[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_bad[MPU9250_NUM_CHECKED_REGISTERS];
	uint8_t			_checked_next;

	// last temperature reading for print_info()
	float			_last_temperature;

	// keep last accel reading for duplicate detection
	uint16_t		_last_accel[3];
	bool			_got_duplicate;

	/**
	 * Reset chip.
	 *
	 * Resets the chip and measurements ranges, but not scale and offset.
	 */
	int			reset();

	/*
	  set low pass filter frequency
	 */
	void _set_dlpf_filter(uint16_t frequency_hz);

	/*
	  set sample rate (approximate) - 1kHz to 5Hz
	*/

	void _set_sample_rate(unsigned desired_sample_rate_hz);
	/**
	 * Set the MPU9250 measurement range.
	 *
	 * @param max_g		The maximum G value the range must support.
	 * @return		OK if the value can be supported, -ERANGE otherwise.
	 */
	int			set_accel_range(unsigned max_g);

	int			set_mag_range(unsigned range);

	/**
	 * Write a register in the MPU9250
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_reg(unsigned reg, uint8_t value);

	/**
	 * Modify a register in the MPU9250
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg		The register to modify.
	 * @param clearbits	Bits in the register to clear.
	 * @param setbits	Bits in the register to set.
	 */
	void			modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/**
	 * Write a register in the MPU9250, updating _checked_values
	 *
	 * @param reg		The register to write.
	 * @param value		The new value to write.
	 */
	void			write_checked_reg(unsigned reg, uint8_t value);

	/*
	  check that key registers still have the right value
	 */
	void check_registers(void);

	/**
	 * Start automatic measurement.
	 */
	void			start();

	/**
	 * Stop automatic measurement.
	 */
	void			stop();

	/**
	 * Static trampoline from the hrt_call context; because we don't have a
	 * generic hrt wrapper yet.
	 *
	 * Called by the HRT in interrupt context at the specified rate if
	 * automatic polling is enabled.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void		measure_trampoline(void *arg);

	/**
	 * Fetch measurements from the sensor and update the report buffers.
	 */
	void			measure();

	/**
	 * Swap a 16-bit value read from the MPU9250 to native byte order.
	 */
	uint16_t		swap16(uint16_t val) { return (val >> 8) | (val << 8);	}

	/**
	 * Get the internal / external state
	 *
	 * @return true if the sensor is not on the main MCU board
	 */
	//bool			is_external() { return (_bus == EXTERNAL_BUS); }

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			self_test();

	/**
	 * Accel self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			accel_self_test();

	/**
	 * Gyro self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int 			gyro_self_test();

	/* do not allow to copy this class due to pointer data members */
	MPU9250(const MPU9250 &);
	MPU9250 operator=(const MPU9250 &);

	/**
	 * Report conversation within the MPU9250, including command byte and
	 * interrupt status.
	 */
	struct MPUReport {
		uint8_t		cmd;
		uint8_t		status;
		uint8_t		accel_x[2];
		uint8_t		accel_y[2];
		uint8_t		accel_z[2];
		uint8_t		temp[2];
		uint8_t		gyro_x[2];
		uint8_t		gyro_y[2];
		uint8_t		gyro_z[2];
	};
};

/*
  list of registers that will be checked in check_registers(). Note
  that MPUREG_PRODUCT_ID must be first in the list.
 */
const uint8_t MPU9250::_checked_registers[MPU9250_NUM_CHECKED_REGISTERS] = { MPUREG_WHOAMI,
									     MPUREG_PWR_MGMT_1,
									     MPUREG_PWR_MGMT_2,
									     MPUREG_USER_CTRL,
									     MPUREG_SMPLRT_DIV,
									     MPUREG_CONFIG,
									     MPUREG_GYRO_CONFIG,
									     MPUREG_ACCEL_CONFIG,
									     MPUREG_ACCEL_CONFIG2,
									     MPUREG_INT_ENABLE,
									     MPUREG_INT_PIN_CFG
									   };


/** driver 'main' command */
extern "C" { __EXPORT int ts_mpu9250_main(int argc, char *argv[]); }

MPU9250::MPU9250(const char *path_accel, const char *path_gyro, enum Rotation rotation) :
	_call(),
	_accel_scale(),
	_accel_range_scale(0.0f),
	_accel_range_m_s2(0.0f),
	_gyro_scale(),
	_gyro_range_scale(0.0f),
	_gyro_range_rad_s(0.0f),
	_mag_scale(),
	_mag_range_scale(0.0f),
	_mag_range_ga(1.3f),
	_dlpf_freq(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ),
	_sample_rate(1000),
	_range_bits(0),
	_accel_reads(perf_alloc(PC_COUNT, "mpu9250_accel_read")),
	_gyro_reads(perf_alloc(PC_COUNT, "mpu9250_gyro_read")),
	_sample_perf(perf_alloc(PC_ELAPSED, "mpu9250_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "mpu9250_bad_transfers")),
	_bad_registers(perf_alloc(PC_COUNT, "mpu9250_bad_registers")),
	_good_transfers(perf_alloc(PC_COUNT, "mpu9250_good_transfers")),
	_reset_retries(perf_alloc(PC_COUNT, "mpu9250_reset_retries")),
	_duplicates(perf_alloc(PC_COUNT, "mpu9250_duplicates")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency")),
	/*_accel_filter_x(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_y(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_accel_filter_z(MPU9250_ACCEL_DEFAULT_RATE, MPU9250_ACCEL_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_x(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_y(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),
	_gyro_filter_z(MPU9250_GYRO_DEFAULT_RATE, MPU9250_GYRO_DEFAULT_DRIVER_FILTER_FREQ),*/
	_accel_filter_x(MPU9250_ACCEL_DEFAULT_RATE, 0),
	_accel_filter_y(MPU9250_ACCEL_DEFAULT_RATE, 0),
	_accel_filter_z(MPU9250_ACCEL_DEFAULT_RATE, 0),
	_gyro_filter_x(MPU9250_GYRO_DEFAULT_RATE, 0),
	_gyro_filter_y(MPU9250_GYRO_DEFAULT_RATE, 0),
	_gyro_filter_z(MPU9250_GYRO_DEFAULT_RATE, 0),
	_mag_filter_x(MPU9250_MAG_DEFAULT_RATE, 0),
	_mag_filter_y(MPU9250_MAG_DEFAULT_RATE, 0),
	_mag_filter_z(MPU9250_MAG_DEFAULT_RATE, 0),
	_accel_int(1000000 / MPU9250_ACCEL_MAX_OUTPUT_RATE),
	_gyro_int(1000000 / MPU9250_GYRO_MAX_OUTPUT_RATE, true),
	_last_accel()
{
	_call_interval = 0;
	_accel_reports = NULL;
	_accel_topic = NULL;
	_accel_orb_class_instance = -1;
	_accel_class_instance = -1;
	_gyro_reports = NULL;
	_gyro_topic = NULL;
	_gyro_orb_class_instance = -1;
	_gyro_class_instance = -1;
	_mag_reports = NULL;
	_mag_topic = NULL;
	_mag_orb_class_instance = -1;
	_mag_class_instance = -1;
	_register_wait = 0;
	_reset_wait = 0;
	_rotation = rotation;
	_checked_next = 0;
	_last_temperature = 0;
	_got_duplicate = false;


	/* Prime _gyro with parents devid. */
	//_gyro->_device_id.devid = _device_id.devid;
	//_gyro->_device_id.devid_s.devtype = DRV_GYR_DEVTYPE_MPU9250;

	// default accel scale factors
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	// default gyro scale factors
	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	memset(&_call, 0, sizeof(_call));
}

MPU9250::~MPU9250()
{
	/* make sure we are truly inactive */
	stop();

	/* delete the gyro subdriver */
	//delete _gyro;

	/* free any existing reports */
	if (_accel_reports != NULL) {
		delete _accel_reports;
	}

	if (_gyro_reports != NULL) {
		delete _gyro_reports;
	}

	/*if (_accel_class_instance != -1) {
		unregister_class_devname(ACCEL_BASE_DEVICE_PATH, _accel_class_instance);
	}*/

	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_accel_reads);
	perf_free(_gyro_reads);
	perf_free(_bad_transfers);
	perf_free(_bad_registers);
	perf_free(_good_transfers);
	perf_free(_reset_retries);
	perf_free(_duplicates);
}

/*uint8_t
MPU9250::read_reg(unsigned reg, uint32_t speed)
{
	uint8_t cmd[2] = { (uint8_t)(reg | DIR_READ), 0};

	// general register transfer at low clock speed
	set_frequency(speed);

	transfer(cmd, cmd, sizeof(cmd));

	return cmd[1];
}

uint16_t
MPU9250::read_reg16(unsigned reg)
{
	uint8_t cmd[3] = { (uint8_t)(reg | DIR_READ), 0, 0 };

	// general register transfer at low clock speed
	set_frequency(MPU9250_LOW_BUS_SPEED);

	transfer(cmd, cmd, sizeof(cmd));

	return (uint16_t)(cmd[1] << 8) | cmd[2];
}*/

void
MPU9250::write_reg(unsigned reg, uint8_t value)
{
	return;
	/*uint8_t	cmd[2];

	cmd[0] = reg | DIR_WRITE;
	cmd[1] = value;

	// general register transfer at low clock speed
	set_frequency(MPU9250_LOW_BUS_SPEED);

	transfer(cmd, nullptr, sizeof(cmd));*/
}

void
MPU9250::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t	val;

	//val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
MPU9250::write_checked_reg(unsigned reg, uint8_t value)
{
	write_reg(reg, value);

	for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
		if (reg == _checked_registers[i]) {
			_checked_values[i] = value;
			_checked_bad[i] = value;
		}
	}
}

void
MPU9250::_set_sample_rate(unsigned desired_sample_rate_hz)
{
	if (desired_sample_rate_hz == 0 ||
	    desired_sample_rate_hz == GYRO_SAMPLERATE_DEFAULT ||
	    desired_sample_rate_hz == ACCEL_SAMPLERATE_DEFAULT) {
		desired_sample_rate_hz = MPU9250_GYRO_DEFAULT_RATE;
	}

	uint8_t div = 1000 / desired_sample_rate_hz;

	if (div > 200) { div = 200; }

	if (div < 1) { div = 1; }

	write_checked_reg(MPUREG_SMPLRT_DIV, div - 1);
	_sample_rate = 1000 / div;
}

void
MPU9250::_set_dlpf_filter(uint16_t frequency_hz)
{
	uint8_t filter;

	/*
	   choose next highest filter frequency available
	 */
	if (frequency_hz == 0) {
		_dlpf_freq = 0;
		filter = BITS_DLPF_CFG_3600HZ;

	} else if (frequency_hz <= 5) {
		_dlpf_freq = 5;
		filter = BITS_DLPF_CFG_5HZ;

	} else if (frequency_hz <= 10) {
		_dlpf_freq = 10;
		filter = BITS_DLPF_CFG_10HZ;

	} else if (frequency_hz <= 20) {
		_dlpf_freq = 20;
		filter = BITS_DLPF_CFG_20HZ;

	} else if (frequency_hz <= 41) {
		_dlpf_freq = 41;
		filter = BITS_DLPF_CFG_41HZ;

	} else if (frequency_hz <= 92) {
		_dlpf_freq = 92;
		filter = BITS_DLPF_CFG_92HZ;

	} else if (frequency_hz <= 184) {
		_dlpf_freq = 184;
		filter = BITS_DLPF_CFG_184HZ;

	} else if (frequency_hz <= 250) {
		_dlpf_freq = 250;
		filter = BITS_DLPF_CFG_250HZ;

	} else {
		_dlpf_freq = 0;
		filter = BITS_DLPF_CFG_3600HZ;
	}

	write_checked_reg(MPUREG_CONFIG, filter);
}

int
MPU9250::set_accel_range(unsigned max_g_in)
{
	uint8_t afs_sel;
	float lsb_per_g;
	float max_accel_g;

	if (max_g_in > 8) { // 16g - AFS_SEL = 3
		afs_sel = 3;
		lsb_per_g = 2048;
		max_accel_g = 16;

	} else if (max_g_in > 4) { //  8g - AFS_SEL = 2
		afs_sel = 2;
		lsb_per_g = 4096;
		max_accel_g = 8;

	} else if (max_g_in > 2) { //  4g - AFS_SEL = 1
		afs_sel = 1;
		lsb_per_g = 8192;
		max_accel_g = 4;

	} else {                //  2g - AFS_SEL = 0
		afs_sel = 0;
		lsb_per_g = 16384;
		max_accel_g = 2;
	}

	write_checked_reg(MPUREG_ACCEL_CONFIG, afs_sel << 3);
	_accel_range_scale = (MPU9250_ONE_G / lsb_per_g);
	_accel_range_m_s2 = max_accel_g * MPU9250_ONE_G;

	return OK;
}

int MPU9250::set_mag_range(unsigned range)
{
	if (range < 1) {
		_range_bits = 0x00;
		_mag_range_scale = 1.0f / 1370.0f;
		_mag_range_ga = 0.88f;

	} else if (range <= 1) {
		_range_bits = 0x01;
		_mag_range_scale = 1.0f / 1090.0f;
		_mag_range_ga = 1.3f;

	} else if (range <= 2) {
		_range_bits = 0x02;
		_mag_range_scale = 1.0f / 820.0f;
		_mag_range_ga = 1.9f;

	} else if (range <= 3) {
		_range_bits = 0x03;
		_mag_range_scale = 1.0f / 660.0f;
		_mag_range_ga = 2.5f;

	} else if (range <= 4) {
		_range_bits = 0x04;
		_mag_range_scale = 1.0f / 440.0f;
		_mag_range_ga = 4.0f;

	} else if (range <= 4.7f) {
		_range_bits = 0x05;
		_mag_range_scale = 1.0f / 390.0f;
		_mag_range_ga = 4.7f;

	} else if (range <= 5.6f) {
		_range_bits = 0x06;
		_mag_range_scale = 1.0f / 330.0f;
		_mag_range_ga = 5.6f;

	} else {
		_range_bits = 0x07;
		_mag_range_scale = 1.0f / 230.0f;
		_mag_range_ga = 8.1f;
	}

	int ret;

	/*
	 * Send the command to set the range
	 */
	//ret = write_reg(ADDR_CONF_B, (_range_bits << 5));

	/*uint8_t range_bits_in = 0;
	ret = read_reg(ADDR_CONF_B, range_bits_in);

	if (OK != ret) {
		perf_count(_comms_errors);
	}

	return !(range_bits_in == (_range_bits << 5));*/

	return ret;
}


int MPU9250::reset()
{
	write_reg(MPUREG_PWR_MGMT_1, BIT_H_RESET);
	usleep(10000);

	write_checked_reg(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_AUTO);
	usleep(1000);

	//write_checked_reg(MPUREG_PWR_MGMT_2, 0);
	usleep(1000);

	// SAMPLE RATE
	_set_sample_rate(_sample_rate);
	usleep(1000);

	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this ruins quality and does not improve the
	// system response
	_set_dlpf_filter(MPU9250_DEFAULT_ONCHIP_FILTER_FREQ);
	usleep(1000);

	// Gyro scale 2000 deg/s ()
	write_checked_reg(MPUREG_GYRO_CONFIG, BITS_FS_2000DPS);
	usleep(1000);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI
	_gyro_range_scale = (0.0174532 / 16.4);//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	_gyro_range_rad_s = (2000.0f / 180.0f) * M_PI_F;

	set_accel_range(16);
	set_mag_range(_mag_range_ga);

	usleep(1000);

	// INT CFG => Interrupt on Data Ready
	write_checked_reg(MPUREG_INT_ENABLE, BIT_RAW_RDY_EN);        // INT: Raw data ready
	usleep(1000);
	write_checked_reg(MPUREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	usleep(1000);

	/*uint8_t retries = 10;

	while (retries--) {
		bool all_ok = true;

		for (uint8_t i = 0; i < MPU9250_NUM_CHECKED_REGISTERS; i++) {
			if (read_reg(_checked_registers[i]) != _checked_values[i]) {
				write_reg(_checked_registers[i], _checked_values[i]);
				all_ok = false;
			}
		}

		if (all_ok) {
			break;
		}
	}*/

	return OK;
}

int
MPU9250::init()
{

	/* allocate basic report buffers */
	_accel_reports = new ringbuffer::RingBuffer(2, sizeof(accel_report));

	if (_accel_reports == NULL) {
		return !OK;
	}

	_gyro_reports = new ringbuffer::RingBuffer(2, sizeof(gyro_report));

	if (_gyro_reports == NULL) {
		return !OK;
	}

	_mag_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_mag_reports == NULL) {
		return !OK;
	}

	if (reset() != OK) {
		return !OK;
	}

	/* Initialize offsets and scales */
	_accel_scale.x_offset = 0;
	_accel_scale.x_scale  = 1.0f;
	_accel_scale.y_offset = 0;
	_accel_scale.y_scale  = 1.0f;
	_accel_scale.z_offset = 0;
	_accel_scale.z_scale  = 1.0f;

	_gyro_scale.x_offset = 0;
	_gyro_scale.x_scale  = 1.0f;
	_gyro_scale.y_offset = 0;
	_gyro_scale.y_scale  = 1.0f;
	_gyro_scale.z_offset = 0;
	_gyro_scale.z_scale  = 1.0f;

	//_accel_class_instance = register_class_devname(ACCEL_BASE_DEVICE_PATH);

	measure();

	/* advertise sensor topic, measure manually to initialize valid report */
	struct accel_report arp;
	_accel_reports->get(&arp);

	/* measurement will have generated a report, publish */
	_accel_topic = orb_advertise_multi(ORB_ID(sensor_accel), &arp,
					   &_accel_orb_class_instance, ORB_PRIO_MAX - 1 );

	if (_accel_topic == NULL) {
		warnx("ADVERT FAIL");
		return !OK;
	}


	/* advertise sensor topic, measure manually to initialize valid report */
	struct gyro_report grp;
	_gyro_reports->get(&grp);

	_gyro_topic = orb_advertise_multi(ORB_ID(sensor_gyro), &grp,
			     &_gyro_orb_class_instance, ORB_PRIO_MAX - 1);

	if (_gyro_topic == NULL) {
		warnx("ADVERT FAIL");
		return !OK;
	}

	struct mag_report mrp;
	_mag_reports->get(&mrp);
	_mag_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrp,
			&_mag_orb_class_instance, ORB_PRIO_MAX - 1);

	if (_mag_topic == NULL) {
		warnx("ADVERT FAIL");
		return !OK;
	}

	return OK;
}

ssize_t
MPU9250::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(accel_report);

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is not enabled, get a fresh measurement into the buffer */
	if (_call_interval == 0) {
		_accel_reports->flush();
		measure();
	}

	/* if no data, error (we could block here) */
	if (_accel_reports->empty()) {
		return -EAGAIN;
	}

	perf_count(_accel_reads);

	/* copy reports out of our buffer to the caller */
	accel_report *arp = reinterpret_cast<accel_report *>(buffer);
	int transferred = 0;

	while (count--) {
		if (!_accel_reports->get(arp)) {
			break;
		}

		transferred++;
		arp++;
	}

	/* return the number of bytes transferred */
	return (transferred * sizeof(accel_report));
}

int
MPU9250::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		measure();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

int
MPU9250::accel_self_test()
{
	if (self_test()) {
		return 1;
	}

	/* inspect accel offsets */
	if (fabsf(_accel_scale.x_offset) < 0.000001f) {
		return 2;
	}

	if (fabsf(_accel_scale.x_scale - 1.0f) > 0.4f || fabsf(_accel_scale.x_scale - 1.0f) < 0.000001f) {
		return 3;
	}

	if (fabsf(_accel_scale.y_offset) < 0.000001f) {
		return 4;
	}

	if (fabsf(_accel_scale.y_scale - 1.0f) > 0.4f || fabsf(_accel_scale.y_scale - 1.0f) < 0.000001f) {
		return 5;
	}

	if (fabsf(_accel_scale.z_offset) < 0.000001f) {
		return 6;
	}

	if (fabsf(_accel_scale.z_scale - 1.0f) > 0.4f || fabsf(_accel_scale.z_scale - 1.0f) < 0.000001f) {
		return 7;
	}

	return 0;
}

int
MPU9250::gyro_self_test()
{
	if (self_test()) {
		return 1;
	}

	/*
	 * Maximum deviation of 20 degrees, according to
	 * http://www.invensense.com/mems/gyro/documents/PS-MPU-9250A-00v3.4.pdf
	 * Section 6.1, initial ZRO tolerance
	 */
	const float max_offset = 0.34f;
	/* 30% scale error is chosen to catch completely faulty units but
	 * to let some slight scale error pass. Requires a rate table or correlation
	 * with mag rotations + data fit to
	 * calibrate properly and is not done by default.
	 */
	const float max_scale = 0.3f;

	/* evaluate gyro offsets, complain if offset -> zero or larger than 20 dps. */
	if (fabsf(_gyro_scale.x_offset) > max_offset) {
		return 1;
	}

	/* evaluate gyro scale, complain if off by more than 30% */
	if (fabsf(_gyro_scale.x_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.y_scale - 1.0f) > max_scale) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_offset) > max_offset) {
		return 1;
	}

	if (fabsf(_gyro_scale.z_scale - 1.0f) > max_scale) {
		return 1;
	}

	/* check if all scales are zero */
	if ((fabsf(_gyro_scale.x_offset) < 0.000001f) &&
	    (fabsf(_gyro_scale.y_offset) < 0.000001f) &&
	    (fabsf(_gyro_scale.z_offset) < 0.000001f)) {
		/* if all are zero, this device is not calibrated */
		return 1;
	}

	return 0;
}

int
MPU9250::mpu9250_ioctl(int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCRESET:
		return reset();

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_call_interval = 0;
				return OK;

			/* external signalling not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
				return mpu9250_ioctl(SENSORIOCSPOLLRATE, 1000);

			case SENSOR_POLLRATE_DEFAULT:
				return mpu9250_ioctl(SENSORIOCSPOLLRATE, MPU9250_ACCEL_DEFAULT_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to hrt interval via microseconds */
					unsigned ticks = 1000000 / arg;

					/* check against maximum sane rate */
					if (ticks < 1000) {
						return -EINVAL;
					}

					// adjust filters
					float cutoff_freq_hz = _accel_filter_x.get_cutoff_freq();
					float sample_rate = 1.0e6f / ticks;
					_set_dlpf_filter(cutoff_freq_hz);
					_accel_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz);
					_accel_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz);


					float cutoff_freq_hz_gyro = _gyro_filter_x.get_cutoff_freq();
					_set_dlpf_filter(cutoff_freq_hz_gyro);
					_gyro_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_gyro_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					float cutoff_freq_hz_mag = _mag_filter_x.get_cutoff_freq();
					_set_dlpf_filter(cutoff_freq_hz_mag);
					_mag_filter_x.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_mag_filter_y.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);
					_mag_filter_z.set_cutoff_frequency(sample_rate, cutoff_freq_hz_gyro);

					/* update interval for next measurement */
					/* XXX this is a bit shady, but no other way to adjust... */
					_call_interval = ticks;

					/*
					  set call interval faster then the sample time. We
					  then detect when we have duplicate samples and reject
					  them. This prevents aliasing due to a beat between the
					  stm32 clock and the mpu9250 clock
					 */
					_call.period = _call_interval - MPU9250_TIMER_REDUCTION;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_call_interval == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return 1000000 / _call_interval;

	case SENSORIOCSQUEUEDEPTH: {
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			/*irqstate_t flags = irqsave();

			if (!_accel_reports->resize(arg)) {
				irqrestore(flags);
				return -ENOMEM;
			}

			irqrestore(flags);*/

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _accel_reports->size();

	case ACCELIOCGSAMPLERATE:
		return _sample_rate;

	case ACCELIOCSSAMPLERATE:
		_set_sample_rate(arg);
		return OK;

	case ACCELIOCGLOWPASS:
		return _accel_filter_x.get_cutoff_freq();

	case ACCELIOCSLOWPASS:
		// set software filtering
		_accel_filter_x.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_y.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		_accel_filter_z.set_cutoff_frequency(1.0e6f / _call_interval, arg);
		return OK;

	case ACCELIOCSSCALE: {
			/* copy scale, but only if off by a few percent */
			struct accel_scale *s = (struct accel_scale *) arg;
			float sum = s->x_scale + s->y_scale + s->z_scale;

			if (sum > 2.0f && sum < 4.0f) {
				memcpy(&_accel_scale, s, sizeof(_accel_scale));
				return OK;

			} else {
				return -EINVAL;
			}
		}

	case ACCELIOCGSCALE:
		/* copy scale out */
		memcpy((struct accel_scale *) arg, &_accel_scale, sizeof(_accel_scale));
		return OK;

	case ACCELIOCSRANGE:
		return set_accel_range(arg);

	case ACCELIOCGRANGE:
		return (unsigned long)((_accel_range_m_s2) / MPU9250_ONE_G + 0.5f);

#ifdef ACCELIOCSHWLOWPASS

	case ACCELIOCSHWLOWPASS:
		_set_dlpf_filter(arg);
		return OK;
#endif

#ifdef ACCELIOCGHWLOWPASS

	case ACCELIOCGHWLOWPASS:
		return _dlpf_freq;
#endif

	default:
		/* give it to the superclass */
		return OK;
	}
}

void
MPU9250::start()
{
	/* make sure we are stopped first */
	stop();

	/* discard any stale data in the buffers */
	_accel_reports->flush();
	_gyro_reports->flush();
	_mag_reports->flush();

	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       1000,
		       _call_interval - MPU9250_TIMER_REDUCTION,
		       (hrt_callout)&MPU9250::measure_trampoline, this);
}

void
MPU9250::stop()
{
	hrt_cancel(&_call);
}

void
MPU9250::measure_trampoline(void *arg)
{
	MPU9250 *dev = reinterpret_cast<MPU9250 *>(arg);

	/* make another measurement */
	dev->measure();
}

void
MPU9250::measure()
{
	if (hrt_absolute_time() < _reset_wait) {
		// we're waiting for a reset to complete
		return;
	}

	struct MPUReport mpu_report;

	struct Report {
		int16_t		accel_x;
		int16_t		accel_y;
		int16_t		accel_z;
		int16_t		temp;
		int16_t		gyro_x;
		int16_t		gyro_y;
		int16_t		gyro_z;
		int16_t		mag_x;
		int16_t		mag_y;
		int16_t		mag_z;
	} report;

	/* start measuring */
	perf_begin(_sample_perf);

	/*
	 * Fetch the full set of measurements from the MPU9250 in one pass.
	 */
	mpu_report.cmd = DIR_READ | MPUREG_INT_STATUS;

	// sensor transfer at high clock speed
	//set_frequency(MPU9250_HIGH_BUS_SPEED);

	/*if (OK != transfer((uint8_t *)&mpu_report, ((uint8_t *)&mpu_report), sizeof(mpu_report))) {
		return;
	}*/

	//check_registers();

	/*
	   see if this is duplicate accelerometer data. Note that we
	   can't use the data ready interrupt status bit in the status
	   register as that also goes high on new gyro data, and when
	   we run with BITS_DLPF_CFG_256HZ_NOLPF2 the gyro is being
	   sampled at 8kHz, so we would incorrectly think we have new
	   data when we are in fact getting duplicate accelerometer data.
	*/
	/*if (!_got_duplicate && memcmp(&mpu_report.accel_x[0], &_last_accel[0], 6) == 0) {
		// it isn't new data - wait for next timer
		perf_end(_sample_perf);
		perf_count(_duplicates);
		_got_duplicate = true;
		return;
	}

	memcpy(&_last_accel[0], &mpu_report.accel_x[0], 6);*/
	_got_duplicate = false;

	/*
	 * Convert from big to little endian
	 */

	/*report.accel_x = int16_t_from_bytes(mpu_report.accel_x);
	report.accel_y = int16_t_from_bytes(mpu_report.accel_y);
	report.accel_z = int16_t_from_bytes(mpu_report.accel_z);

	report.temp = int16_t_from_bytes(mpu_report.temp);

	report.gyro_x = int16_t_from_bytes(mpu_report.gyro_x);
	report.gyro_y = int16_t_from_bytes(mpu_report.gyro_y);
	report.gyro_z = int16_t_from_bytes(mpu_report.gyro_z);*/

	report.accel_x = 0;
	report.accel_y = 0;
	report.accel_z = 100;

	report.temp = 0;

	report.gyro_x = 0;
	report.gyro_y = 0;
	report.gyro_z = 0;

	report.mag_x = 1;
	report.mag_y = 1;
	report.mag_z = 0;

	if (report.accel_x == 0 &&
	    report.accel_y == 0 &&
	    report.accel_z == 0 &&
	    report.temp == 0 &&
	    report.gyro_x == 0 &&
	    report.gyro_y == 0 &&
	    report.gyro_z == 0) {
		// all zero data - probably a SPI bus error
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		// note that we don't call reset() here as a reset()
		// costs 20ms with interrupts disabled. That means if
		// the mpu6k does go bad it would cause a FMU failure,
		// regardless of whether another sensor is available,
		return;
	}

	perf_count(_good_transfers);

	if (_register_wait != 0) {
		// we are waiting for some good transfers before using
		// the sensor again. We still increment
		// _good_transfers, but don't return any data yet
		_register_wait--;
		return;
	}


	/*
	 * Swap axes and negate y
	 */
	int16_t accel_xt = report.accel_y;
	int16_t accel_yt = ((report.accel_x == -32768) ? 32767 : -report.accel_x);

	int16_t gyro_xt = report.gyro_y;
	int16_t gyro_yt = ((report.gyro_x == -32768) ? 32767 : -report.gyro_x);

	/*
	 * Apply the swap
	 */
	report.accel_x = accel_xt;
	report.accel_y = accel_yt;
	report.gyro_x = gyro_xt;
	report.gyro_y = gyro_yt;

	/*
	 * Report buffers.
	 */
	accel_report		arb;
	gyro_report		grb;
	mag_report		mrb;

	/*
	 * Adjust and scale results to m/s^2.
	 */
	grb.timestamp = arb.timestamp = hrt_absolute_time();

	// report the error count as the sum of the number of bad
	// transfers and bad register reads. This allows the higher
	// level code to decide if it should use this sensor based on
	// whether it has had failures
	grb.error_count = arb.error_count = perf_event_count(_bad_transfers) + perf_event_count(_bad_registers);

	/*
	 * 1) Scale raw value to SI units using scaling from datasheet.
	 * 2) Subtract static offset (in SI units)
	 * 3) Scale the statically calibrated values with a linear
	 *    dynamically obtained factor
	 *
	 * Note: the static sensor offset is the number the sensor outputs
	 * 	 at a nominally 'zero' input. Therefore the offset has to
	 * 	 be subtracted.
	 *
	 *	 Example: A gyro outputs a value of 74 at zero angular rate
	 *	 	  the offset is 74 from the origin and subtracting
	 *		  74 from all measurements centers them around zero.
	 */

	/* NOTE: Axes have been swapped to match the board a few lines above. */

	arb.x_raw = report.accel_x;
	arb.y_raw = report.accel_y;
	arb.z_raw = report.accel_z;

	float xraw_f = report.accel_x;
	float yraw_f = report.accel_y;
	float zraw_f = report.accel_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_in_new = ((xraw_f * _accel_range_scale) - _accel_scale.x_offset) * _accel_scale.x_scale;
	float y_in_new = ((yraw_f * _accel_range_scale) - _accel_scale.y_offset) * _accel_scale.y_scale;
	float z_in_new = ((zraw_f * _accel_range_scale) - _accel_scale.z_offset) * _accel_scale.z_scale;

	arb.x = _accel_filter_x.apply(x_in_new);
	arb.y = _accel_filter_y.apply(y_in_new);
	arb.z = _accel_filter_z.apply(z_in_new);

	math::Vector<3> aval(x_in_new, y_in_new, z_in_new);
	math::Vector<3> aval_integrated;

	bool accel_notify = _accel_int.put(arb.timestamp, aval, aval_integrated, arb.integral_dt);
	arb.x_integral = aval_integrated(0);
	arb.y_integral = aval_integrated(1);
	arb.z_integral = aval_integrated(2);

	arb.scaling = _accel_range_scale;
	arb.range_m_s2 = _accel_range_m_s2;

	_last_temperature = (report.temp) / 361.0f + 35.0f;

	arb.temperature_raw = report.temp;
	arb.temperature = _last_temperature;

	grb.x_raw = report.gyro_x;
	grb.y_raw = report.gyro_y;
	grb.z_raw = report.gyro_z;

	xraw_f = report.gyro_x;
	yraw_f = report.gyro_y;
	zraw_f = report.gyro_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_gyro_in_new = ((xraw_f * _gyro_range_scale) - _gyro_scale.x_offset) * _gyro_scale.x_scale;
	float y_gyro_in_new = ((yraw_f * _gyro_range_scale) - _gyro_scale.y_offset) * _gyro_scale.y_scale;
	float z_gyro_in_new = ((zraw_f * _gyro_range_scale) - _gyro_scale.z_offset) * _gyro_scale.z_scale;

	grb.x = _gyro_filter_x.apply(x_gyro_in_new);
	grb.y = _gyro_filter_y.apply(y_gyro_in_new);
	grb.z = _gyro_filter_z.apply(z_gyro_in_new);

	math::Vector<3> gval(x_gyro_in_new, y_gyro_in_new, z_gyro_in_new);
	math::Vector<3> gval_integrated;

	bool gyro_notify = _gyro_int.put(arb.timestamp, gval, gval_integrated, grb.integral_dt);
	grb.x_integral = gval_integrated(0);
	grb.y_integral = gval_integrated(1);
	grb.z_integral = gval_integrated(2);

	grb.scaling = _gyro_range_scale;
	grb.range_rad_s = _gyro_range_rad_s;

	grb.temperature_raw = report.temp;
	grb.temperature = _last_temperature;

	_accel_reports->force(&arb);
	_gyro_reports->force(&grb);


	mrb.x_raw = report.mag_x;
	mrb.y_raw = report.mag_y;
	mrb.z_raw = report.mag_z;

	xraw_f = report.mag_x;
	yraw_f = report.mag_y;
	zraw_f = report.mag_z;

	// apply user specified rotation
	rotate_3f(_rotation, xraw_f, yraw_f, zraw_f);

	float x_mag_in_new = ((xraw_f * _mag_range_scale) - _mag_scale.x_offset) * _mag_scale.x_scale;
	float y_mag_in_new = ((yraw_f * _mag_range_scale) - _mag_scale.y_offset) * _mag_scale.y_scale;
	float z_mag_in_new = ((zraw_f * _mag_range_scale) - _mag_scale.z_offset) * _mag_scale.z_scale;

	mrb.x = _mag_filter_x.apply(x_mag_in_new);
	mrb.y = _mag_filter_y.apply(y_mag_in_new);
	mrb.z = _mag_filter_z.apply(z_mag_in_new);

	mrb.x = 0.1;
	mrb.y = 0.1;
	mrb.z = 0.1;

	mrb.temperature = _last_temperature;
	mrb.timestamp = hrt_absolute_time();
	mrb.error_count = 0;

	_mag_reports->force(&mrb);

	if (accel_notify) {
		/* log the time of this report */
		perf_begin(_controller_latency_perf);
		/* publish it */
		orb_publish(ORB_ID(sensor_accel), _accel_topic, &arb);
	}

	if (gyro_notify) {
		/* publish it */
		orb_publish(ORB_ID(sensor_gyro), _gyro_topic, &grb);
	}

	orb_publish(ORB_ID(sensor_mag), _mag_topic, &mrb);

	/* stop measuring */
	perf_end(_sample_perf);
}

/**
 * Local functions in support of the shell command.
 */
namespace mpu9250
{

MPU9250	*g_dev_int; // on internal bus
MPU9250	*g_dev_ext; // on external bus

void	start(bool, enum Rotation);
void	stop(bool);
void	usage();

/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void
start(enum Rotation rotation)
{
	//int fd;
	MPU9250 **g_dev_ptr = &g_dev_int;
	const char *path_accel = MPU_DEVICE_PATH_ACCEL;
	const char *path_gyro  = MPU_DEVICE_PATH_GYRO;

	if (*g_dev_ptr != NULL)
		/* if already started, the still command succeeded */
	{
		printf("already started\n");
		errx(0, "already started");
	}

	/* create the driver */
	*g_dev_ptr = new MPU9250(path_accel, path_gyro, rotation);

	if (*g_dev_ptr == NULL) {
		goto fail;
	}

	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	/*fd = px4_open(path_accel, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}*/

	if ((*g_dev_ptr)->mpu9250_ioctl(SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	//close(fd);

	return;

fail:

	if (*g_dev_ptr != NULL) {
		delete(*g_dev_ptr);
		*g_dev_ptr = NULL;
	}

	printf("driver start failed\n");
	errx(1, "driver start failed");
}

void
stop()
{
	MPU9250 **g_dev_ptr = &g_dev_int;

	if (*g_dev_ptr != NULL) {
		delete *g_dev_ptr;
		*g_dev_ptr = NULL;

	} else {
		/* warn, but not an error */
		warnx("already stopped.");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */

void
usage()
{
	warnx("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump', 'testerror'");
	warnx("options:");
	warnx("    -X    (external bus)");
	warnx("    -R rotation");
}

} // namespace

int
ts_mpu9250_main(int argc, char *argv[])
{
	enum Rotation rotation = ROTATION_NONE;

	const char *verb = argv[optind];

	/*
	 * Start/load the driver.

	 */
	if (!strcmp(verb, "start")) {
		mpu9250::start(rotation);
	}

	if (!strcmp(verb, "stop")) {
		mpu9250::stop();
	}

	//mpu9250::usage();
	return 0;
}
