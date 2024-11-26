#include "rtl_custom.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/home_position.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rtl_status.h>
#include <uORB/topics/rtl_time_estimate.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_magnetometer.h>

int RTLCustom::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("rtl_custom",
                     SCHED_DEFAULT,
                     SCHED_PRIORITY_DEFAULT,
                     1024,
                     (px4_main_t)&run_trampoline,
                     (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

RTLCustom *RTLCustom::instantiate(int argc, char *argv[])
{
    int example_param = 0;
    bool example_flag = false;
    bool error_flag = false;

    int myoptind = 1;
    int ch;
    const char *myoptarg = nullptr;

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
        return nullptr;
    }

    RTLCustom *instance = new RTLCustom(example_param, example_flag);

    if (instance == nullptr) {
        PX4_ERR("alloc failed");
    }

    return instance;
}

RTLCustom::RTLCustom(int example_param, bool example_flag)
    : ModuleParams(nullptr)
{
}

void RTLCustom::run()
{
    while (!should_exit()) {
        parameters_update();

        bool DEBUG = false;

        // Locations
        print_home_location(DEBUG);
        print_current_location(DEBUG);

        // Magnetic Bearing
        float mag_heading_deg = calculate_mag_heading();
        float current_heading_deg = calculate_current_heading();
        float true_heading_deg = calculate_true_heading();

        PX4_INFO("GT: %3.1f deg, Attitude: %3.1f deg, Mag Bearing: %3.1f deg",
            (double)true_heading_deg,
            (double)current_heading_deg,
            (double)mag_heading_deg);

        px4_usleep(1_s);
    }
}

int RTLCustom::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int RTLCustom::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Custom RTL module for GPS-independent return-to-launch functionality.
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("rtl_custom", "command");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

int RTLCustom::print_status()
{
    PX4_INFO("Running");
    return 0;
}

void RTLCustom::parameters_update(bool force)
{
    if (_parameter_update_sub.updated() || force) {
        parameter_update_s update;
        _parameter_update_sub.copy(&update);
        updateParams();
    }
}

extern "C" __EXPORT int rtl_custom_main(int argc, char *argv[])
{
    return RTLCustom::main(argc, argv);
}
