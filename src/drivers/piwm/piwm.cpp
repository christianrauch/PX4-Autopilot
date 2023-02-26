/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include "piwm.h"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <drivers/drv_mixer.h>
#include <drivers/drv_pwm_output.h>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>

int TemplateModule::print_status() {
  PX4_INFO("Running");
  // TODO: print additional runtime information about the state of the module

  return 0;
}

int TemplateModule::custom_command(int argc, char *argv[]) {
  /*
  if (!is_running()) {
          print_usage("not running");
          return 1;
  }

  // additional custom commands can be handled like this:
  if (!strcmp(argv[0], "do-something")) {
          get_instance()->do_something();
          return 0;
  }
   */

  return print_usage("unknown command");
}

int TemplateModule::task_spawn(int argc, char *argv[]) {
//  _task_id =
//      px4_task_spawn_cmd("module", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 1024,
//                         (px4_main_t)&run_trampoline, (char *const *)argv);

//  if (_task_id < 0) {
//    _task_id = -1;
//    return -errno;
//  }

//  return 0;
//}

//TemplateModule *TemplateModule::instantiate(int argc, char *argv[]) {
  int example_param = 0;
  bool example_flag = false;
  bool error_flag = false;

  int myoptind = 1;
  int ch;
  const char *myoptarg = nullptr;

  // parse CLI arguments
  while ((ch = px4_getopt(argc, argv, "p:f", &myoptind, &myoptarg)) != EOF) {
    switch (ch) {
    case 'p':
      example_param = (int)strtol(myoptarg, nullptr, 10);
      break;

    case 'f':
      example_flag = true;
      break;

    case '?':
      error_flag = true;
      break;

    default:
      PX4_WARN("unrecognized flag");
      error_flag = true;
      break;
    }
  }

  if (error_flag) {
    return -1;
  }

  TemplateModule *instance = new TemplateModule(example_param, example_flag);

  if (instance == nullptr) {
    PX4_ERR("alloc failed");
  }

  _object.store(instance);
  _task_id = task_id_is_work_queue;
  instance->ScheduleNow();

  return 0;
}

TemplateModule::TemplateModule(int example_param, bool example_flag)
    : CDev("/dev/pwm_output0"),
      OutputModuleInterface(MODULE_NAME, px4::wq_configurations::hp_default)
{
    CDev::init();
}

//void TemplateModule::run() {
//  // Example: run the loop synchronized to the sensor_combined topic publication
//  int actuator_outputs_sub = orb_subscribe(ORB_ID(actuator_outputs));

//  px4_pollfd_struct_t fds[1];
//  fds[0].fd = actuator_outputs_sub;
//  fds[0].events = POLLIN;

//  // initialize parameters
//  parameters_update(true);

//  while (!should_exit()) {

//    // wait for up to 1000ms for data
//    int pret = px4_poll(fds, (sizeof(fds) / sizeof(fds[0])), 1000);

//    if (pret == 0) {
//      // Timeout: let the loop run anyway, don't do `continue` here

//    } else if (pret < 0) {
//      // this is undesirable but not much we can do
//      PX4_ERR("poll error %d, %d", pret, errno);
//      px4_usleep(50000);
//      continue;

//    } else if (fds[0].revents & POLLIN) {

//      struct actuator_outputs_s actuator_outputs;
//      orb_copy(ORB_ID(actuator_outputs), actuator_outputs_sub,
//               &actuator_outputs);
//      // TODO: do something with the data...
////      PX4_INFO("actuators: %d", actuator_outputs.noutputs);
////      for (uint32_t i = 0; i < actuator_outputs.noutputs; i++) {
////        PX4_INFO("actuator(%d): %f", i, double(actuator_outputs.output[i]));
////      }
//    }

//    parameters_update();
//  }

//  orb_unsubscribe(actuator_outputs_sub);
//}

//void TemplateModule::parameters_update(bool force) {
//  // check for parameter updates
//  if (_parameter_update_sub.updated() || force) {
//    // clear update
//    parameter_update_s update;
//    _parameter_update_sub.copy(&update);

//    // update parameters from storage
//    updateParams();
//  }
//}

bool TemplateModule::updateOutputs(bool stop_motors, uint16_t outputs[MAX_ACTUATORS],
                   unsigned num_outputs, unsigned num_control_groups_updated)
{
    PX4_WARN("TemplateModule::updateOutputs");
    PX4_INFO("actuators: %d", MAX_ACTUATORS);
    for (uint32_t i = 0; i < MAX_ACTUATORS; i++) {
        PX4_INFO("actuator(%d): %f", i, double(outputs[i]));
    }
    return true;
}

void TemplateModule::Run()
{
    PX4_WARN("Run");
    _mixing_output.update();
    _mixing_output.updateSubscriptions(true);
}

int TemplateModule::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
    PX4_WARN("TemplateModule::ioctl, cmd %d", cmd);

    int ret = OK;

    switch (cmd) {
    case PWM_SERVO_ARM:
        break;

    case PWM_SERVO_DISARM:
        break;

    case PWM_SERVO_SET_MIN_PWM: {
            struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

            for (unsigned i = 0; i < pwm->channel_count; i++) {
                if (i < OutputModuleInterface::MAX_ACTUATORS && !_mixing_output.useDynamicMixing()) {
                    _mixing_output.minValue(i) = pwm->values[i];
                }
            }

            break;
        }

    case PWM_SERVO_SET_MAX_PWM: {
            struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

            for (unsigned i = 0; i < pwm->channel_count; i++) {
                if (i < OutputModuleInterface::MAX_ACTUATORS && !_mixing_output.useDynamicMixing()) {
                    _mixing_output.maxValue(i) = pwm->values[i];
                }
            }

            break;
        }

    case PWM_SERVO_SET_UPDATE_RATE:
        // PWMSim does not limit the update rate
        break;

    case PWM_SERVO_SET_SELECT_UPDATE_RATE:
        break;

    case PWM_SERVO_GET_DEFAULT_UPDATE_RATE:
        *(uint32_t *)arg = 9999;
        break;

    case PWM_SERVO_GET_UPDATE_RATE:
        *(uint32_t *)arg = 9999;
        break;

    case PWM_SERVO_GET_SELECT_UPDATE_RATE:
        *(uint32_t *)arg = 0;
        break;

    case PWM_SERVO_GET_FAILSAFE_PWM: {
            struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

            for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
                pwm->values[i] = _mixing_output.failsafeValue(i);
            }

            pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
            break;
        }

    case PWM_SERVO_GET_DISARMED_PWM: {
            struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

            for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
                pwm->values[i] = _mixing_output.disarmedValue(i);
            }

            pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
            break;
        }

    case PWM_SERVO_GET_MIN_PWM: {
            struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

            for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
                pwm->values[i] = _mixing_output.minValue(i);
            }

            pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
            break;
        }

    case PWM_SERVO_GET_MAX_PWM: {
            struct pwm_output_values *pwm = (struct pwm_output_values *)arg;

            for (unsigned i = 0; i < OutputModuleInterface::MAX_ACTUATORS; i++) {
                pwm->values[i] = _mixing_output.maxValue(i);
            }

            pwm->channel_count = OutputModuleInterface::MAX_ACTUATORS;
            break;
        }

    case PWM_SERVO_GET_RATEGROUP(0) ... PWM_SERVO_GET_RATEGROUP(PWM_OUTPUT_MAX_CHANNELS - 1): {
            // no restrictions on output grouping
            unsigned channel = cmd - PWM_SERVO_GET_RATEGROUP(0);

            *(uint32_t *)arg = (1 << channel);
            break;
        }

    case PWM_SERVO_GET_COUNT:
        *(unsigned *)arg = OutputModuleInterface::MAX_ACTUATORS;
        break;

    case MIXERIOCRESET:
        _mixing_output.resetMixer();
        break;

    case MIXERIOCLOADBUF: {
            const char *buf = (const char *)arg;
            unsigned buflen = strlen(buf);
            ret = _mixing_output.loadMixer(buf, buflen);
            break;
        }

    default:
        ret = -ENOTTY;
        break;
    }

//    return 0;
    return ret;
}

int TemplateModule::print_usage(const char *reason) {
  if (reason) {
    PX4_WARN("%s\n", reason);
  }

  PRINT_MODULE_DESCRIPTION(
      R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start -f -p 42

)DESCR_STR");

  PRINT_MODULE_USAGE_NAME("module", "template");
  PRINT_MODULE_USAGE_COMMAND("start");
  PRINT_MODULE_USAGE_PARAM_FLAG('f', "Optional example flag", true);
  PRINT_MODULE_USAGE_PARAM_INT('p', 0, 0, 1000, "Optional example parameter",
                               true);
  PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

  return 0;
}

int piwm_main(int argc, char *argv[]) {
  return TemplateModule::main(argc, argv);
}
