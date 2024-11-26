#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
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
using namespace time_literals;

extern "C" __EXPORT int rtl_custom_main(int argc, char *argv[]);

struct PositionYawSetpoint {
    double lat;
    double lon;
    double alt;
    double yaw;
};
class RTLCustom : public ModuleBase<RTLCustom>, public ModuleParams
{
public:
    RTLCustom(int example_param, bool example_flag);

    virtual ~RTLCustom() = default;

    static int task_spawn(int argc, char *argv[]);
    static RTLCustom *instantiate(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);
    void run() override;
    int print_status() override;

private:
    void print_home_location(bool DEBUG) {
        if (_home_pos_sub.update() && DEBUG) {
            rtl_position.alt = _home_pos_sub.get().alt;
            rtl_position.lat = _home_pos_sub.get().lat;
            rtl_position.lon = _home_pos_sub.get().lon;
            rtl_position.yaw = _home_pos_sub.get().yaw;
            PX4_INFO("Home Position: Alt: %lf, Lat: %lf, Lon: %lf, Yaw: %lf",
                rtl_position.alt, rtl_position.lat, rtl_position.lon, rtl_position.yaw);
        }
    }

    void print_current_location(bool DEBUG){
            if (_global_pos_sub.update() && DEBUG){
            global_position.alt = _global_pos_sub.get().alt;
            global_position.lat = _global_pos_sub.get().lat;
            global_position.lon = _global_pos_sub.get().lon;
            PX4_INFO("Global Position: Alt: %lf, Lat: %lf, Lon: %lf",
                global_position.alt, global_position.lat, global_position.lon);
        }
    }


    // Headings --------------------------------------------------
    float calculate_mag_heading(){
        vehicle_magnetometer_s magnetometer;
        _vehicle_magnetometer_sub.copy(&magnetometer);

        float mag_heading = atan2f(magnetometer.magnetometer_ga[1],
                            magnetometer.magnetometer_ga[0]);

        return -mag_heading * 180.0f / M_PIf;
    }


    float calculate_current_heading(){
        vehicle_attitude_s attitude;
        _vehicle_attitude_sub.copy(&attitude);
        float arg1 = 2.0f * (attitude.q[0] * attitude.q[3] - attitude.q[1] * attitude.q[2]);
        float arg2 = 1.0f - 2.0f * (attitude.q[2] * attitude.q[2] + attitude.q[3] * attitude.q[3]);
        float current_yaw = atan2f(arg1,arg2);
        double current_yaw_deg = current_yaw * 180.0f / M_PIf;
        return current_yaw_deg;
    }

    float calculate_true_heading(){

        vehicle_attitude_s attitude;
        _vehicle_attitude_sub.copy(&attitude);

        float dx = _home_pos_sub.get().lat - _global_pos_sub.get().lat;
        float dy = _home_pos_sub.get().lon - _global_pos_sub.get().lon;
        float true_bearing = atan2f(dy, dx);  // Radians
        double true_bearing_deg = true_bearing * 180.0f / M_PIf;
        return true_bearing_deg;
    }




    void parameters_update(bool force = false);
    PositionYawSetpoint rtl_position{}; // Add this line
    PositionYawSetpoint global_position{};  // add this for global position

    // // Subscriptions

    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    uORB::Subscription _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	uORB::SubscriptionData<mission_s> _mission_sub{ORB_ID(mission)};
	uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};

	uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
	uORB::PublicationData<rtl_status_s> _rtl_status_pub{ORB_ID(rtl_status)};  // Subscription for global position
};
