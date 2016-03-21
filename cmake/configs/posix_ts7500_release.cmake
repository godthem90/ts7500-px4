include(posix/px4_impl_posix)

if ("${TS7500_TOOLCHAIN_DIR}" STREQUAL "")
	set(TS7500_TOOLCHAIN_DIR /toolchain/arm-oabi-toolchain/)
endif()

set(CMAKE_PROGRAM_PATH 
	"${TS7500_TOOLCHAIN_DIR}/arm-unknown-linux-gnu/bin"
	${CMAKE_PROGRAM_PATH}
	)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-oabi.cmake)

set(config_module_list
	drivers/device
	drivers/ts7500io
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/perf
	modules/uORB
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/sensors
	modules/mavlink
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/navigator
	modules/vtol_att_control
	modules/mc_pos_control
	modules/mc_att_control
	modules/land_detector
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/dataman
	modules/sdlog2
	modules/commander
	modules/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
)
