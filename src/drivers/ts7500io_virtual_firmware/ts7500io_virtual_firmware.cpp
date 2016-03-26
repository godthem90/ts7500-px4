#include <px4_config.h>

#include <stdio.h>	// required for task_create
#include <stdbool.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <signal.h>
#include <crc32.h>

#include <drivers/drv_pwm_output.h>
#include <drivers/drv_hrt.h>

#include <systemlib/perf_counter.h>
#include <systemlib/pwm_limit/pwm_limit.h>

#include "ts7500io_virtual_firmware.h"

struct sys_state_s system_state;

pwm_limit_t pwm_limit;

int safety_off_countdown = 3000;

class Virtual_Firmware
{
public:

	Virtual_Firmware();

	~Virtual_Firmware();

	int init();

private:

	struct hrt_call		_call;

	perf_counter_t mixer_perf;
	perf_counter_t controls_perf;
	perf_counter_t loop_perf;

	void task_main();

	static void task_main_trampoline(int argc, char *argv[]);

};

Virtual_Firmware *firmware = NULL;

Virtual_Firmware::Virtual_Firmware()
{
	memset(&_call, 0, sizeof(_call));
	firmware = this;
}

Virtual_Firmware::~Virtual_Firmware()
{
	hrt_cancel(&_call);

	perf_free(mixer_perf);
	perf_free(controls_perf);
	perf_free(loop_perf);
	firmware = NULL;
}

void Virtual_Firmware::task_main()
{
	/* track the rate at which the loop is running */
	perf_count(loop_perf);

	/* kick the mixer */
	perf_begin(mixer_perf);
	mixer_tick();
	perf_end(mixer_perf);

	/* kick the control inputs */
	perf_begin(controls_perf);
	controls_tick();
	perf_end(controls_perf);

	if(safety_off_countdown-- == 0)
		r_status_flags |= PX4IO_P_STATUS_FLAGS_SAFETY_OFF;

	/* check for debug activity (default: none) */
	//show_debug_messages();
}

void Virtual_Firmware::task_main_trampoline(int argc, char *argv[])
{
	firmware->task_main();
}

int Virtual_Firmware::init()
{
	/* configure the first 8 PWM outputs (i.e. all of them) */
	//up_pwm_servo_init(0xff);
	printf("servo init!!\n");

	/* run C++ ctors before we go any further */
	//up_cxxinitialize();

	/* reset all to zero */
	memset(&system_state, 0, sizeof(system_state));

	/* initialise the control inputs */
	controls_init();

	/* initialize PWM limit lib */
	pwm_limit_init(&pwm_limit);

	/* add a performance counter for mixing */
	mixer_perf = perf_alloc(PC_ELAPSED, "mix");

	/* add a performance counter for controls */
	controls_perf = perf_alloc(PC_ELAPSED, "controls");

	/* and one for measuring the loop rate */
	loop_perf = perf_alloc(PC_INTERVAL, "loop");


	/* start polling at the specified rate */
	hrt_call_every(&_call,
		       10000,
		       10000 - 200,
		       (hrt_callout)&Virtual_Firmware::task_main_trampoline, this);

	return OK;
}

const char *r_status_flags_table[] = {
	"PX4IO_P_STATUS_FLAGS_OUTPUTS_ARMED",
	"PX4IO_P_STATUS_FLAGS_OVERRIDE",
	"PX4IO_P_STATUS_FLAGS_RC_OK",
	"PX4IO_P_STATUS_FLAGS_RC_PPM",
	"PX4IO_P_STATUS_FLAGS_RC_DSM",
	"PX4IO_P_STATUS_FLAGS_RC_SBUS",
	"PX4IO_P_STATUS_FLAGS_FMU_OK",
	"PX4IO_P_STATUS_FLAGS_RAW_PWM",
	"PX4IO_P_STATUS_FLAGS_MIXER_OK",
	"PX4IO_P_STATUS_FLAGS_ARM_SYNC",
	"PX4IO_P_STATUS_FLAGS_INIT_OK",
	"PX4IO_P_STATUS_FLAGS_FAILSAFE",
	"PX4IO_P_STATUS_FLAGS_SAFETY_OFF",
	"PX4IO_P_STATUS_FLAGS_FMU_INITIALIZED",
	"PX4IO_P_STATUS_FLAGS_RC_ST24",
	"PX4IO_P_STATUS_FLAGS_RC_SUMD"
};

const char *r_raw_rc_flags_table[] = {
	"PX4IO_P_RAW_RC_FLAGS_FRAME_DROP",
	"PX4IO_P_RAW_RC_FLAGS_FAILSAFE",
	"PX4IO_P_RAW_RC_FLAGS_RC_DSM11",
	"PX4IO_P_RAW_RC_FLAGS_MAPPING_OK",
	"PX4IO_P_RAW_RC_FLAGS_RC_OK"
};

const char *r_setup_arming_table[] = {
	"PX4IO_P_SETUP_ARMING_IO_ARM_OK",
	"PX4IO_P_SETUP_ARMING_FMU_ARMED",
	"PX4IO_P_SETUP_ARMING_MANUAL_OVERRIDE_OK",
	"PX4IO_P_SETUP_ARMING_FAILSAFE_CUSTOM",
	"PX4IO_P_SETUP_ARMING_INAIR_RESTART_OK",
	"PX4IO_P_SETUP_ARMING_ALWAYS_PWM_ENABLE",
	"PX4IO_P_SETUP_ARMING_RC_HANDLING_DISABLED",
	"PX4IO_P_SETUP_ARMING_LOCKDOWN",
	"PX4IO_P_SETUP_ARMING_FORCE_FAILSAFE",
	"PX4IO_P_SETUP_ARMING_TERMINATION_FAILSAFE",
	"PX4IO_P_SETUP_ARMING_OVERRIDE_IMMEDIATE"
};

void print_page()
{
	int i;

	printf("r_status_flags : \n");
	for( i = 0; i < 16; i++ )
	{
		if(r_status_flags & (1 << i))
			printf("\t%s SET!\n",r_status_flags_table[i]);
	}
	printf("\n");
	
	printf("r_raw_rc_flags : \n");
	for( i = 0; i < 5; i++ )
	{
		if(r_raw_rc_flags & (1 << i))
			printf("\t%s SET!\n",r_raw_rc_flags_table[i]);
	}
	printf("\n");
	
	printf("r_setup_arming : \n");
	for( i = 0; i < 5; i++ )
	{
		if(r_setup_arming & (1 << i))
			printf("\t%s SET!\n",r_setup_arming_table[i]);
	}
	printf("\n");

}

extern "C" __EXPORT int ts7500io_virtual_firmware_main(int argc, char *argv[]);

int ts7500io_virtual_firmware_main(int argc, char *argv[])
{

	if (argc < 2) {
		warnx("usage: ts7500io_virtual_firmware {start|stop|status}");
		return 0;
	}

	if (!strcmp(argv[1], "start")) {

		if (firmware != NULL) {
			warnx("already running");
			return 0;
		}

		firmware = new Virtual_Firmware;

		if (firmware == NULL) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != firmware->init()) {
			delete firmware;
			firmware = NULL;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (firmware == NULL) {
			warnx("not running");
			return 1;
		}

		delete firmware;
		firmware = NULL;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (firmware) {
			printf("is running\n");
			print_page();
			return 0;

		} else {
			printf("not running\n");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}

