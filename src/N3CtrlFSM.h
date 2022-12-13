#ifndef __N3CTRLFSM_H
#define __N3CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>
#include <quadrotor_msgs/SO3Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <uav_utils/utils.h>
#include <boost/format.hpp>
#include <djiros/DroneArmControl.h>
#include <djiros/SDKControlAuthority.h>
#include "input.h"
#include "hovthrkf.h"
#include "controller.h"
#include "N3CtrlStateVisualization.h"

struct AutoTakeoffLand_t
{
    bool landed{true};
    ros::Time toggle_takeoff_land_time;
    std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
    Eigen::Vector4d start_pose;
    
    static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motors idle running for 3 seconds before takeoff
    static constexpr double DELAY_TRIGGER_TIME = 2.0;  // Time to be delayed when reach at target height
};

class N3CtrlFSM
{
public:
    Parameter_t& param;

    RC_Data_t rc_data;
    Odom_Data_t odom_data;
    Imu_Data_t imu_data;
    Command_Data_t cmd_data;
    Idling_Data_t idling_data;
    Trigger_Data_t trigger_data;
    Takeoff_Land_Data_t takeoff_land_data;

    Controller& controller;
    HovThrKF& hov_thr_kf;
    StateVisualizer stateVisualizer;

    uav_utils::DebugSS_t dbgss;

    ros::Publisher fsm_dbg_pub;
    ros::Publisher des_pose_pub;
    ros::Publisher traj_start_trigger_pub;

    ros::ServiceClient arming_client_srv;
    ros::ServiceClient set_api_mode_srv;

    Eigen::Vector4d hover_pose;
    uint32_t last_command_id;

    ros::Time last_ctrl_time;

    enum State_t
    {
        DIRECT_CTRL=0,
        JS_CTRL,
        JS_NO_CTRL,
        JS_RESET_POS_CTRL,
        CMD_HOVER,
        CMD_CTRL,
        CMD_NO_CTRL,
        CMD_RESET_POS_CTRL,
        AUTO_TAKEOFF,
        AUTO_LAND
    };

    enum AXIS_State_t
    {
        FIX = 0,
        MOVE,
        BREAK
    };

    enum Odom_State_t
    {
        ODOM_INVALID=0,
        ODOM_LC=1,
        ODOM_VALID=2
    };

    enum Work_Mode_t
    {
        REALTIME=0,
        SIMULATION=1,
        SIM_WITHOUT_RC=2
    };

    enum JS_CTRL_Mode_t
    {
        JS_CTRL_MODE_FEEDBACK=0,
        JS_CTRL_MODE_RAW
    };

    enum Idling_State_t
    {
        NOIDLING=0,
        AUTOIDLING=1,
        OUTERIDLING=2
    };

    N3CtrlFSM(Parameter_t &, Controller &, HovThrKF &);
    void process();
    bool rc_is_received(const ros::Time& now_time);
    void set_work_mode(Work_Mode_t mode);
    void set_js_ctrl_mode(JS_CTRL_Mode_t mode);
    bool get_landed() { return takeoff_land.landed; }

private:
    State_t state;
    Desired_State_t des;
    AXIS_State_t axis_states[4]; // x, y, z, yaw
    Work_Mode_t work_mode;
    JS_CTRL_Mode_t js_ctrl_mode;
    Idling_State_t idling_state;
    AutoTakeoffLand_t takeoff_land;
    ros::Time idling_start_time;

    // ---- state process ----
    void determine_idling(const ros::Time& now_time);
    void determine_state(const ros::Time& now_time);
    void go_to_state(State_t ds);
    void leave_state(State_t cs);
    void determine_axis_states();
    void go_to_axis_state(AXIS_State_t ds, int i);
    bool toggle_arm_disarm(bool arm);
    bool toggle_api_mode(bool on_off);
    void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
    void publish_trigger(const nav_msgs::Odometry &odom_msg);
    void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom);

    // ---- control related ----
    void process_control(const ros::Time& now_time);
    void process_no_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_raw_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_idling_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3, double idling_lasting_time);
    void process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_break_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_js_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_takeoff_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void process_land_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3);
    void publish_desire(const Desired_State_t& des);

    // ---- tools ----
    void flush_last_command_id();
    double get_yaw_from_odom();
    void get_des_from_js(Eigen::Vector3d& des_v, double& dyaw);
    void align_with_imu(Controller_Output_t& u);
    void set_hov_with_odom();
    std::string state_to_str();
    std::string axis_state_to_str();
    Desired_State_t get_rotor_speed_up_des(const ros::Time now);
    Desired_State_t get_takeoff_land_des(const double speed);

};

#endif