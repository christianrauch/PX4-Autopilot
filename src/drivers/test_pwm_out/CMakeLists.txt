px4_add_module(
	MODULE drivers__test_pwm_out
	MAIN test_pwm_out
	COMPILE_FLAGS
	SRCS
		main.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		mixer_module
	)
