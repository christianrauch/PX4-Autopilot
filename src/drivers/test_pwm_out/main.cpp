#include <px4_log.h>
#include <drivers/device/device.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <px4_platform_common/module.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>

#include <px4_platform_common/sem.hpp>

#include <drivers/device/device.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform/pwm_out_base.h>

using namespace time_literals;

class TestPWMOut : public ModuleBase<TestPWMOut>, public OutputModuleInterface
{
public:
	TestPWMOut();
	virtual ~TestPWMOut();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	/** @see ModuleBase::print_status() */
	int print_status() override;

	int init();

	bool updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
			   unsigned num_outputs, unsigned num_control_groups_updated) override;

private:
	static constexpr int MAX_ACTUATORS = 8;

	MixingOutput _mixing_output{"PWM_TEST", MAX_ACTUATORS, *this, MixingOutput::SchedulingPolicy::Auto, false};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// pwm_out::PWMOutBase *_pwm_out{nullptr};

	perf_counter_t	_cycle_perf;
	perf_counter_t	_interval_perf;
};

using namespace pwm_out;

TestPWMOut::TestPWMOut() :
	OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval"))
{
}

TestPWMOut::~TestPWMOut()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	// delete _pwm_out;
}

int TestPWMOut::init()
{
	PX4_WARN("TestPWMOut::init");

	// _pwm_out = new BOARD_PWM_OUT_IMPL(MAX_ACTUATORS);

	// int ret = _pwm_out->init();

	// if (ret != 0) {
	// 	PX4_ERR("PWM output init failed");
	// 	delete _pwm_out;
	// 	_pwm_out = nullptr;
	// 	return ret;
	// }

	ScheduleNow();

	return PX4_OK;
}

int TestPWMOut::task_spawn(int argc, char *argv[])
{
	PX4_WARN("TestPWMOut::task_spawn");

	TestPWMOut *instance = new TestPWMOut();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	PX4_WARN("???");

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

bool TestPWMOut::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
				unsigned num_outputs, unsigned num_control_groups_updated)
{
	// _pwm_out->send_output_pwm(outputs, num_outputs);
	PX4_WARN("#### actuators: %d", num_outputs);
	//convert this to duty_cycle in ns
	for (unsigned i = 0; i < num_outputs; ++i) {
		PX4_INFO("actuator(%d): %hu", i, outputs[i]);
	}
	return true;
}

void TestPWMOut::Run()
{
	if (should_exit()) {
		ScheduleClear();
		_mixing_output.unregister();

		PX4_WARN("exit?");

		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	_mixing_output.update();

	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		updateParams();
	}

	// float schedule_rate = 2;
	_mixing_output.updateSubscriptions(false);

	// ScheduleOnInterval(1000000 / schedule_rate, 1000000 / schedule_rate);

	perf_end(_cycle_perf);
}

int TestPWMOut::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TestPWMOut::print_status()
{
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	_mixing_output.printStatus();
	return 0;
}

int TestPWMOut::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Linux PWM output driver with board-specific backend implementation.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("linux_pwm_out", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int test_pwm_out_main(int argc, char *argv[])
{
	return TestPWMOut::main(argc, argv);
}
