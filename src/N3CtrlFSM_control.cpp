#include "N3CtrlFSM.h"

using namespace Eigen;
using std::cout;
using std::endl;
using namespace uav_utils;

void N3CtrlFSM::process_control(const ros::Time& now_time)
{
    Controller_Output_t u;
    SO3_Controller_Output_t u_so3;
    des.set_odom(odom_data);
    //ROS_WARN("[n3ctrl] state = %d",state); //zxzxzxzx
    //result: state always equals 2
    //ROS_WARN("[n3ctrl] js_ctrl_mode = %d",js_ctrl_mode); //zxzxzxzx

    if (state == DIRECT_CTRL)
    {
        // process_raw_control(u, u_so3);
        return;
    }	
    else if (state == JS_CTRL)
    {
        if (js_ctrl_mode==JS_CTRL_MODE_RAW)
        {
            ROS_WARN("[n3ctrl] js_ctrl_mode = JS_CTRL_MODE_RAW");//zxzxzxzx
            process_raw_control(u, u_so3);
        }
        else
        {
            // This function is called when it is running normally. zxzxzxzx
            process_js_control(u, u_so3);
        }
    }
    else if (state == JS_NO_CTRL)
    {
        ROS_WARN("[n3ctrl] state = JS_NO_CTRL");//zxzxzxzx
        process_no_control(u, u_so3);
    }
    else if (state == JS_RESET_POS_CTRL)
    {
        process_break_control(u, u_so3);
    }
    else if (state == CMD_HOVER)
    {
        process_hover_control(u, u_so3);
    }
    else if (state == CMD_CTRL)
    {
        //ROS_WARN("[n3ctrl] state = CMD_CTRL");//zxzxzxzx
        process_cmd_control(u, u_so3);
    }
    else if (state == CMD_NO_CTRL)
    {
        process_no_control(u, u_so3);
    }
    else if (state == CMD_RESET_POS_CTRL)
    {
        process_break_control(u, u_so3);
    }
    else if (state == AUTO_TAKEOFF)
    {
        process_takeoff_control(u, u_so3);
    }
    else if (state == AUTO_LAND)
    {
        process_land_control(u, u_so3);
    }
    else
    {
        ROS_ASSERT(false);
    }

    if (idling_state != NOIDLING)
    {
        double idling_lasting_time = (now_time - idling_start_time).toSec();
        process_idling_control(u, u_so3, idling_lasting_time);
    }

    controller.publish_so3_ctrl(u_so3, now_time);
    align_with_imu(u);
    controller.publish_ctrl(u, now_time, odom_data.msg.header.stamp);

    if (idling_state == NOIDLING)
    {
        if ((state==JS_CTRL && axis_states[2]==FIX && js_ctrl_mode==JS_CTRL_MODE_FEEDBACK) // JS_CTRL hovering case
            || (state==CMD_HOVER) // CMD_HOVER case
            || (state==CMD_CTRL   // CMD_CTRL verticle stationary case
                && idling_state==NOIDLING // NOT in idling mode
                && std::fabs(cmd_data.v(2)) < param.hover.vert_velo_limit_for_update
                && cmd_data.p(2) > param.hover.vert_height_limit_for_update
                && odom_data.p(2) > param.hover.vert_height_limit_for_update))
        {
#if 0			
            hov_thr_kf.update(imu_data.a(2));

            // This line may not take effect according to param.hov.use_hov_percent_kf
            param.config_full_thrust(hov_thr_kf.get_hov_thr());

            hov_thr_kf.process(u.thrust);
            hov_thr_kf.publish_thr();
#else			
            hov_thr_kf.simple_update(imu_data.q, u.thrust, imu_data.a);
            // This line may not take effect according to param.hov.use_hov_percent_kf
            param.config_full_thrust(hov_thr_kf.get_hov_thr());
            hov_thr_kf.publish_thr();
#endif			
        }
        
    }
}

void N3CtrlFSM::process_no_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    u.roll = 0.0;
    u.pitch = 0.0;
    u.yaw = get_yaw_from_odom();
    u.thrust = 0.0;
    u.mode = Controller_Output_t::VERT_VELO;

    u_so3.Rdes = rotz(get_yaw_from_odom());
    u_so3.Fdes = Vector3d(0, 0, param.mass * param.gra);
}

void N3CtrlFSM::process_raw_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    double des_yaw = yaw_add( get_yaw_from_odom(), -(rc_data.yaw * param.rc.yaw_scale));
    
    u.roll  =  (rc_data.roll * param.rc.attitude_scale);
    u.pitch =  (rc_data.pitch * param.rc.attitude_scale);
    u.yaw   = des_yaw;
    u.thrust = rc_data.thr * param.rc.vert_velo_scale;
    u.mode = Controller_Output_t::VERT_VELO;
    u_so3.Rdes = ypr_to_R(Vector3d(u.yaw, u.pitch, u.roll));
    u_so3.Fdes = Vector3d(0, 0, param.mass * (param.gra + u.thrust * 0.5));
}

void N3CtrlFSM::process_idling_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3, double idling_lasting_time)
{
    u.roll = 0.0;
    u.pitch = 0.0;
    u.yaw = get_yaw_from_odom();
    if (idling_lasting_time < param.idling.landing_timeout)
    {
        u.thrust = param.mass * param.gra / param.full_thrust * param.idling.landing_thrust_percent;
    }
    else
    {
        u.thrust = param.idling.lowest_thrust;
    }
    u.mode = Controller_Output_t::VERT_THRU;
}


void N3CtrlFSM::process_hover_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    des.p = hover_pose.head<3>();
    des.v = Vector3d::Zero();
    des.yaw = hover_pose(3);
    des.a = Vector3d::Zero();

    controller.update(des, odom_data, u, u_so3);

    publish_desire(des);
}

void N3CtrlFSM::process_break_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    des.p = odom_data.p;
    des.v = Vector3d::Zero();
    des.yaw = get_yaw_from_odom();
    des.a = Vector3d::Zero();

    controller.update(des, odom_data, u, u_so3);

    publish_desire(des);
}

void N3CtrlFSM::process_cmd_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    des.p = cmd_data.p;
    des.v = cmd_data.v;
    des.yaw = cmd_data.yaw;
    des.a = cmd_data.a;

    controller.update(des, odom_data, u, u_so3);

    publish_desire(des);	
}

void N3CtrlFSM::process_js_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    Vector3d des_v;
    double des_dyaw;
    
    get_des_from_js(des_v, des_dyaw);
    des.a = Vector3d::Zero();

    int axis_id;
    for(axis_id=0; axis_id<3; ++axis_id)
    {
        switch(axis_states[axis_id]) // xy axis
        {
            case FIX:
                des.p(axis_id) = hover_pose(axis_id);
                des.v(axis_id) = 0.0;
                break;
            case MOVE:
                des.p(axis_id) = odom_data.p(axis_id);
                des.v(axis_id) = des_v(axis_id);
                break;
            case BREAK:
                des.p(axis_id) = odom_data.p(axis_id);
                des.v(axis_id) = 0.0;
                break;
            default:
                ROS_ASSERT(false);
        }
    }

    axis_id = 3;
    switch(axis_states[axis_id]) // xy axis
    {
        case FIX:
            des.yaw = hover_pose(axis_id);
            break;
        case MOVE:
            des.yaw = yaw_add(get_yaw_from_odom(), des_dyaw);
            break;
        case BREAK:
            ROS_ASSERT(false);
            break;
        default:
            ROS_ASSERT(false);
    }

    controller.update(des, odom_data, u, u_so3); 

    publish_desire(des);
}

void N3CtrlFSM::process_takeoff_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    
    ros::Time now_time = ros::Time::now();
    if ((now_time - takeoff_land.toggle_takeoff_land_time).toSec() < 
        AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) {// Wait for several seconds to warn prople.
        ROS_INFO_ONCE("rotor_speed");
        des = get_rotor_speed_up_des(now_time);
    } else {
        ROS_INFO_ONCE("rotor_up");
        des = get_takeoff_land_des(param.takeoff_land.speed);
    }

    controller.update(des, odom_data, u, u_so3);

    publish_desire(des);	
}

void N3CtrlFSM::process_land_control(Controller_Output_t& u, SO3_Controller_Output_t& u_so3)
{
    des = get_takeoff_land_des(-param.takeoff_land.speed);
    controller.update(des, odom_data, u, u_so3);

    publish_desire(des);
}


Desired_State_t N3CtrlFSM::get_rotor_speed_up_des(const ros::Time now) {

    double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec();
    double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
    if (des_a_z > 0.1) {

        ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
        des_a_z = 0.0;
    }

    Desired_State_t out;
    out.p = takeoff_land.start_pose.head<3>();
    out.v = Eigen::Vector3d::Zero();
    out.a = Eigen::Vector3d(0, 0, des_a_z);
    out.j = Eigen::Vector3d::Zero();
    out.yaw = takeoff_land.start_pose(3);
    out.yaw_rate = 0.0;

    return out;
}

Desired_State_t N3CtrlFSM::get_takeoff_land_des(const double speed) {

    ros::Time now = ros::Time::now();
    double delta_t = (now - takeoff_land.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
    // takeoff_land.last_set_cmd_time = now;

    // takeoff_land.start_pose(2) += speed * delta_t;

    Desired_State_t out;
    out.p = takeoff_land.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
    out.v = Eigen::Vector3d(0, 0, speed);
    out.a = Eigen::Vector3d::Zero();
    out.j = Eigen::Vector3d::Zero();
    out.yaw = takeoff_land.start_pose(3);
    out.yaw_rate = 0.0;

    return out;
}