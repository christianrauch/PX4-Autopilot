if(DEFINED ENV{AUTOPILOT_HOST})
	set(AUTOPILOT_HOST $ENV{AUTOPILOT_HOST})
else()
	set(AUTOPILOT_HOST "pi")
endif()

if(DEFINED ENV{AUTOPILOT_USER})
	set(AUTOPILOT_USER $ENV{AUTOPILOT_USER})
else()
	set(AUTOPILOT_USER "pi")
endif()

add_custom_target(upload
	COMMAND rsync -arh --progress
			${CMAKE_RUNTIME_OUTPUT_DIRECTORY} ${PX4_SOURCE_DIR}/boards/raspberrypi/flypi/configs/flypi.config ${PX4_BINARY_DIR}/etc # source
			"${AUTOPILOT_USER}@${AUTOPILOT_HOST}:/home/${AUTOPILOT_USER}/px4" # destination
	DEPENDS px4
	COMMENT "uploading px4"
	USES_TERMINAL
)
