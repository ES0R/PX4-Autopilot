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
    void parameters_update(bool force = false);
    // PositionYawSetpoint rtl_position{}; // Add this line
    // PositionYawSetpoint global_position{};  // add this for global position

    // // Subscriptions

    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

    // uORB::Subscription _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)};
    // uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	// uORB::SubscriptionData<vehicle_global_position_s> _global_pos_sub{ORB_ID(vehicle_global_position)};	/**< global position subscription */
	// uORB::SubscriptionData<mission_s> _mission_sub{ORB_ID(mission)};
	// uORB::SubscriptionData<home_position_s> _home_pos_sub{ORB_ID(home_position)};

	// uORB::Publication<rtl_time_estimate_s> _rtl_time_estimate_pub{ORB_ID(rtl_time_estimate)};
	// uORB::PublicationData<rtl_status_s> _rtl_status_pub{ORB_ID(rtl_status)};  // Subscription for global position
};
