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
        //parameters_update();

        // bool DEBUG = false;

        // // Print Home Position
        // if (_home_pos_sub.update() && DEBUG) {
        //     //const home_position_s &home_pos = _home_pos_sub.get();
        //     rtl_position.alt = _home_pos_sub.get().alt;
        //     rtl_position.lat = _home_pos_sub.get().lat;
        //     rtl_position.lon = _home_pos_sub.get().lon;
        //     rtl_position.yaw = _home_pos_sub.get().yaw;
        //     PX4_INFO("Home Position: Alt: %lf, Lat: %lf, Lon: %lf, Yaw: %lf",
        //         rtl_position.alt, rtl_position.lat, rtl_position.lon, rtl_position.yaw);
        // }

        // if (_global_pos_sub.update() && DEBUG){
        //     global_position.alt = _global_pos_sub.get().alt;
        //     global_position.lat = _global_pos_sub.get().lat;
        //     global_position.lon = _global_pos_sub.get().lon;
        //     PX4_INFO("Global Position: Alt: %lf, Lat: %lf, Lon: %lf",
        //         global_position.alt, global_position.lat, global_position.lon);
        // }

        // // Print Attitude Data
        // if (_vehicle_attitude_sub.updated() && DEBUG) {
        //     vehicle_attitude_s attitude;
        //     _vehicle_attitude_sub.copy(&attitude);
        //     PX4_INFO("Attitude Quaternion: W: %.3f, X: %.3f, Y: %.3f, Z: %.3f",
        //         (double)attitude.q[0],  // w component
        //         (double)attitude.q[1],  // x component
        //         (double)attitude.q[2],  // y component
        //         (double)attitude.q[3]); // z component
        // }

        // // Print Magnetometer Data
        // if (_vehicle_magnetometer_sub.updated() && DEBUG) {
        //     vehicle_magnetometer_s magnetometer;
        //     _vehicle_magnetometer_sub.copy(&magnetometer);
        //     PX4_INFO("Magnetometer: X: %f, Y: %f, Z: %f gauss",
        //         (double)magnetometer.magnetometer_ga[0],
        //         (double)magnetometer.magnetometer_ga[1],
        //         (double)magnetometer.magnetometer_ga[2]);
        // }

        //         // Get current attitude
        // vehicle_attitude_s attitude;
        // _vehicle_attitude_sub.copy(&attitude);
        // float current_yaw = (double)atan2f(2.0f * (attitude.q[0] * attitude.q[3] + attitude.q[1] * attitude.q[2]),
        //                                  1.0f - 2.0f * (attitude.q[2] * attitude.q[2] + attitude.q[3] * attitude.q[3]));

        // // Calculate true bearing to home using home and global positions
        // if (_home_pos_sub.update() && _global_pos_sub.update()) {
        //     float dx = _home_pos_sub.get().lat - _global_pos_sub.get().lat;
        //     float dy = _home_pos_sub.get().lon - _global_pos_sub.get().lon;
        //     float true_bearing = atan2f(dy, dx);  // Radians

        //     // Convert to degrees for printing
        //     double true_bearing_deg = true_bearing * 180.0f / M_PIf;
        //     double current_yaw_deg = current_yaw * 180.0f / M_PIf;

        //     PX4_INFO("Bearing Comparison: True: %3.1f deg, Current Yaw: %3.1f deg, Difference: %3.1f deg",
        //         (double)true_bearing_deg,
        //         (double)current_yaw_deg,
        //         (double)(true_bearing_deg - current_yaw_deg));
        // }

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
