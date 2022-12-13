#include "N3CtrlFSM.h"

using std::endl;
using std::cout;
using namespace uav_utils;

void N3CtrlFSM::determine_state(const ros::Time& now_time) {
    Odom_State_t odom_state;
    if ((now_time - odom_data.rcv_stamp).toSec() < param.msg_timeout.odom) {
        std::string odom_flag_str = odom_data.msg.child_frame_id;
        if (odom_flag_str.size() > 0) {
            if (odom_flag_str == std::string("V") || odom_flag_str.compare("campose") == 0 ||
                odom_flag_str.compare("/campose") == 0) {
                odom_state = ODOM_VALID;
            } else if (odom_flag_str == std::string("X")) {
                odom_state = ODOM_INVALID;
            } else if (odom_flag_str == std::string("L")) {
                odom_state = ODOM_LC;
            } else {
                ROS_ERROR("<N3CTRL> Invalid Odometry Flag!!!");
                odom_state = ODOM_VALID;
            }
        } else {
            odom_state = ODOM_VALID;
        }
    } else {
        std::string tmpss =
            boost::str(boost::format("[Odom timeout] now: %.3f recvStamp: %.3f msgStamp: %.3f") %
                       now_time % odom_data.rcv_stamp % odom_data.msg.header.stamp);
        dbgss << tmpss << endl;
        ROS_WARN_STREAM_THROTTLE(1.0, "[CTRL]" << tmpss);
        odom_state = ODOM_INVALID;
    }

    bool cmd_flag_valid =
        (cmd_data.trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY ||
         cmd_data.trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED);
    bool cmd_id_renewed = cmd_data.trajectory_id > last_command_id;
    bool cmd_is_timeout = (now_time - cmd_data.rcv_stamp).toSec() < param.msg_timeout.cmd;
    //ROS_WARN("%d %d %d",cmd_flag_valid,cmd_id_renewed, cmd_is_timeout); //zxzxzxzx
    bool cmd_is_valid = cmd_flag_valid & cmd_id_renewed & cmd_is_timeout;

    if (!cmd_is_valid) {
        dbgss << boost::format("[Command not valid] flag[%d] cmd_id[%d] last_id[%d]") %
                     cmd_data.trajectory_flag % cmd_data.trajectory_id % last_command_id << endl;
    }
    dbgss << boost::format("[Command info] now: %.3f recvStamp: %.3f msgStamp: %.3f") % now_time %
                 cmd_data.rcv_stamp % cmd_data.msg.header.stamp << endl;

    bool exit_cmd_js_check[4] = {std::fabs(rc_data.roll) > param.rc.exit_cmd_deadzone,
                                 std::fabs(rc_data.pitch) > param.rc.exit_cmd_deadzone,
                                 std::fabs(rc_data.yaw) > param.rc.exit_cmd_deadzone,
                                 std::fabs(rc_data.thr) > param.rc.exit_cmd_deadzone};
    bool exit_cmd_js_satisfied = (exit_cmd_js_check[0] || exit_cmd_js_check[1] ||
                                  exit_cmd_js_check[2] || exit_cmd_js_check[3]);

    dbgss << boost::format("[Exit cmd js] [%d%d%d%d]") % exit_cmd_js_check[0] %
                 exit_cmd_js_check[1] % exit_cmd_js_check[2] % exit_cmd_js_check[3] << endl;

    dbgss << "state";
    while (1) {
        dbgss << " --> " << state_to_str();
        if (state == DIRECT_CTRL) {
            if (rc_data.enter_api_mode) {
                go_to_state(JS_CTRL);
                flush_last_command_id();
                ROS_INFO("Reset trajectory id to %d", cmd_data.trajectory_id);
                continue;
            } else if (param.takeoff_land.enable && takeoff_land_data.triggered && 
                takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::TAKEOFF) {// Try to jump to AUTO_TAKEOFF

                if (!rc_data.is_api_mode || !rc_data.is_command_mode || !rc_data.check_centered()) {
                    takeoff_land_data.triggered = false;
                    ROS_ERROR("[n3ctrl] Reject AUTO_TAKEOFF. If you have your RC connected, keep its switches at \"auto hover\" and \"command control\" states, and all sticks at the center, then takeoff again.");
                    ROS_ERROR("is_api_mode:%d; is_command_mode:%d", rc_data.is_api_mode, rc_data.is_command_mode);
                    break;
                }
                
                go_to_state(AUTO_TAKEOFF);
                continue;
            }
        } else if (!rc_data.is_api_mode) {
            go_to_state(DIRECT_CTRL);
            continue;
        } else if (state == JS_CTRL) {
            if (rc_data.check_enter_command_mode() || (takeoff_land.delay_trigger.first && now_time > takeoff_land.delay_trigger.second)) {
                if (param.leader) publish_trigger(odom_data.msg);
                go_to_state(CMD_HOVER);
                continue;
            }
            if (odom_state == ODOM_INVALID && js_ctrl_mode == JS_CTRL_MODE_FEEDBACK) {
                go_to_state(JS_NO_CTRL);
                continue;
            } else if (odom_state == ODOM_LC) {
                go_to_state(JS_RESET_POS_CTRL);
                continue;
            }

            if (odom_state == ODOM_VALID && trigger_data.get_enter_js() &&
                js_ctrl_mode == JS_CTRL_MODE_RAW) {
                set_js_ctrl_mode(JS_CTRL_MODE_FEEDBACK);
                go_to_state(JS_CTRL);
                flush_last_command_id();
                ROS_INFO("From JS_RAW to JS_FEEDBACK\nReset trajectory id to %d", cmd_data.trajectory_id);
            }

            determine_axis_states();
        } else if (state == JS_NO_CTRL) {
            if (exit_cmd_js_satisfied) {
                go_to_state(DIRECT_CTRL);
            } else if (odom_state == ODOM_VALID) {
                go_to_state(JS_CTRL);
                continue;
            } else if (odom_state == ODOM_LC) {
                go_to_state(JS_RESET_POS_CTRL);
                continue;
            }
        } else if (state == JS_RESET_POS_CTRL) {
            if (exit_cmd_js_satisfied) {
                go_to_state(DIRECT_CTRL);
            } else if (odom_state == ODOM_VALID) {
                go_to_state(JS_CTRL);
                continue;
            } else if (odom_state == ODOM_INVALID) {
                go_to_state(JS_NO_CTRL);
                continue;
            }
        } else if (state == CMD_HOVER) {
            if (exit_cmd_js_satisfied) {
                go_to_state(DIRECT_CTRL);
            } else if (!rc_data.is_command_mode) {
                go_to_state(JS_CTRL);
                continue;
            } else if (odom_state == ODOM_INVALID) {
                go_to_state(CMD_NO_CTRL);
                continue;
            } else if (odom_state == ODOM_LC) {
                go_to_state(CMD_RESET_POS_CTRL);
                continue;
            } else if (takeoff_land_data.triggered && 
                takeoff_land_data.takeoff_land_cmd == quadrotor_msgs::TakeoffLand::LAND) {
                go_to_state(AUTO_LAND);
                continue;
            }
            if (cmd_is_valid) {
                go_to_state(CMD_CTRL);
                continue;
            }
        } else if (state == CMD_CTRL) {
            if (exit_cmd_js_satisfied) {
                go_to_state(DIRECT_CTRL);
            } else if (!rc_data.is_command_mode) {
                leave_state(CMD_CTRL);
                go_to_state(JS_CTRL);
                continue;
            } else if (!cmd_is_valid) {
                leave_state(CMD_CTRL);
                go_to_state(CMD_HOVER);
                continue;
            } else if (odom_state == ODOM_INVALID) {
                leave_state(CMD_CTRL);
                go_to_state(CMD_NO_CTRL);
                continue;
            } else if (odom_state == ODOM_LC) {
                leave_state(CMD_CTRL);
                go_to_state(CMD_RESET_POS_CTRL);
                continue;
            }
        } else if (state == CMD_RESET_POS_CTRL) {
            if (exit_cmd_js_satisfied) {
                go_to_state(DIRECT_CTRL);
            } else if (odom_state == ODOM_VALID) {
                go_to_state(CMD_HOVER);
                continue;
            } else if (odom_state == ODOM_INVALID) {
                go_to_state(CMD_NO_CTRL);
                continue;
            }
        } else if (state == CMD_NO_CTRL) {
            if (exit_cmd_js_satisfied) {
                go_to_state(DIRECT_CTRL);
            } else if (odom_state == ODOM_VALID) {
                go_to_state(CMD_HOVER);
                continue;
            } else if (odom_state == ODOM_LC) {
                go_to_state(CMD_RESET_POS_CTRL);
                continue;
            }
        } else if (state == AUTO_TAKEOFF) {
            if (odom_data.p(2) >= 
                (takeoff_land.start_pose(2) + param.takeoff_land.height)) {// reach the desired height
                takeoff_land.delay_trigger.first = true;
                takeoff_land.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
                go_to_state(JS_CTRL);
                continue;
            }
        } else if (state == AUTO_LAND) {
            if (get_landed()) {
                go_to_state(DIRECT_CTRL);
                toggle_arm_disarm(false);
                continue;
            }
        } else {
            ROS_ASSERT(false);
        }
        rc_data.enter_api_mode = false;
        takeoff_land_data.triggered = false;
        dbgss << "\n ---->" << state_to_str() << endl;
        break;
    }
}

void N3CtrlFSM::go_to_state(State_t ds)  // dest state
{
    state = ds;
    switch (ds) {
        case DIRECT_CTRL:
            ROS_WARN("[N3CTRL] Exit API mode!");
            break;
        case JS_CTRL:
            controller.config_gain(param.hover_gain);
            ROS_INFO("[N3CTRL] ---- Enter JS_CTRL mode");
            set_hov_with_odom();
            break;
        case JS_NO_CTRL:
            ROS_WARN("[N3CTRL] JS EMERGENCY!");
            break;
        case JS_RESET_POS_CTRL:
            controller.config_gain(param.hover_gain);
            ROS_WARN("[CTRL] JS LOOP-CLOSURE!");
            set_hov_with_odom();
            break;
        case CMD_HOVER:
            controller.config_gain(param.hover_gain);
            ROS_INFO("[N3CTRL] ---- Enter CMD_HOVER mode.");
            set_hov_with_odom();
            break;
        case CMD_CTRL:
            controller.config_gain(param.track_gain);
            ROS_INFO("[N3CTRL] ---- Enter CMD_CTRL mode.");
            break;
        case CMD_NO_CTRL:
            ROS_WARN("[N3CTRL] CMD EMERGENCY!");
            break;
        case CMD_RESET_POS_CTRL:
            controller.config_gain(param.hover_gain);
            ROS_WARN("[N3CTRL] CMD LOOP-CLOSURE!");
            set_hov_with_odom();
            break;
        case AUTO_TAKEOFF:
            controller.config_gain(param.hover_gain);
            set_start_pose_for_takeoff_land(odom_data);
            toggle_api_mode(true);
            toggle_arm_disarm(true);
            ROS_INFO("\033[32m[n3ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
            break;
        case AUTO_LAND:
            controller.config_gain(param.hover_gain);
            set_start_pose_for_takeoff_land(odom_data);
            ROS_INFO("\033[32m[n3ctrl] AUTO_LAND\033[32m");
            break;
        default:
            ROS_ASSERT(false);
    }
}

void N3CtrlFSM::leave_state(State_t cs)  // current state
{
    switch (cs) {
        case CMD_CTRL:
            flush_last_command_id();
            if (cmd_data.trajectory_flag ==
                quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_COMPLETED) {
                ROS_INFO("Trajectory #%u complete when leave CMD_CTRL", cmd_data.trajectory_id);
                dbgss << "Trajectory #" << cmd_data.trajectory_id << "complete when leave CMD_CTRL."
                      << endl;
            }
            break;
        default:
            ROS_ASSERT(false);
    }
}

void N3CtrlFSM::determine_axis_states() {
    bool channel_out_of_deadzone[4];
    bool velo_in_stop_range[4];

    channel_out_of_deadzone[0] = !uav_utils::in_range(rc_data.pitch, param.rc.deadzone);
    channel_out_of_deadzone[1] = !uav_utils::in_range(rc_data.roll, param.rc.deadzone);
    channel_out_of_deadzone[0] = channel_out_of_deadzone[0] || channel_out_of_deadzone[1];
    channel_out_of_deadzone[1] = channel_out_of_deadzone[0] || channel_out_of_deadzone[1];

    channel_out_of_deadzone[2] = !uav_utils::in_range(rc_data.thr, param.rc.deadzone);
    channel_out_of_deadzone[3] = !uav_utils::in_range(rc_data.yaw, param.rc.deadzone);

    Eigen::Vector3d v_c = rotz(get_yaw_from_quaternion(odom_data.q)).transpose() * odom_data.v;
    velo_in_stop_range[0] = uav_utils::in_range(v_c(0), param.js_ctrl_lock_velo);
    velo_in_stop_range[1] = uav_utils::in_range(v_c(1), param.js_ctrl_lock_velo);

    velo_in_stop_range[0] = velo_in_stop_range[0] && velo_in_stop_range[1];
    velo_in_stop_range[1] = velo_in_stop_range[0] && velo_in_stop_range[1];

    velo_in_stop_range[2] = uav_utils::in_range(v_c(2), param.js_ctrl_lock_velo);
    velo_in_stop_range[3] = true;

    for (int axis_id = 0; axis_id < 4; ++axis_id) {
        while (1) {
            if (axis_states[axis_id] == FIX) {
                if (channel_out_of_deadzone[axis_id]) {
                    go_to_axis_state(MOVE, axis_id);
                    continue;
                }
            } else if (axis_states[axis_id] == MOVE) {
                if (!channel_out_of_deadzone[axis_id]) {
                    go_to_axis_state(BREAK, axis_id);
                    continue;
                }
            } else if (axis_states[axis_id] == BREAK) {
                if (velo_in_stop_range[axis_id]) {
                    go_to_axis_state(FIX, axis_id);
                    continue;
                }
            } else {
                ROS_ASSERT(false);
            }
            break;
        }
    }

    // if (axis_states[0]==MOVE || axis_states[1]==MOVE)

    dbgss << "." << axis_state_to_str();
    dbgss << boost::format(" rc[%d%d%d%d] velo[%d%d%d%d]") % channel_out_of_deadzone[0] %
                 channel_out_of_deadzone[1] % channel_out_of_deadzone[2] %
                 channel_out_of_deadzone[3] % velo_in_stop_range[0] % velo_in_stop_range[1] %
                 velo_in_stop_range[2] % velo_in_stop_range[3];
}

void N3CtrlFSM::go_to_axis_state(AXIS_State_t ds, int i) {
    axis_states[i] = ds;
    switch (ds) {
        case FIX:
            hover_pose(i) = (i != 3 ? odom_data.p(i) : get_yaw_from_odom());
            break;
        case MOVE:
            break;
        case BREAK:
            break;
        default:
            ROS_ASSERT(false);
    }
}

void N3CtrlFSM::determine_idling(const ros::Time& now_time) {
    if (state == JS_CTRL || state == CMD_CTRL || state == DIRECT_CTRL || state == AUTO_TAKEOFF) {
        // pass
    } else {
        return;
    }

    bool idling_rule_outer = idling_data.need_idling;
    bool idling_rule_podom = (odom_data.p(2) < param.idling.desired_height_limit);
    bool idling_rule_pdes = false;
    bool idling_rule_vodom = (std::fabs(odom_data.v(2)) < param.idling.feedback_velo_limit);
    bool idling_rule_vdes = false;

    if (state == JS_CTRL || state == DIRECT_CTRL) {
        idling_rule_pdes = true;
        idling_rule_vdes = (rc_data.thr < param.idling.js_thrust_limit);
    } else if (state == CMD_CTRL) {
        idling_rule_pdes = (cmd_data.p(2) < param.idling.desired_height_limit);
        idling_rule_vdes = (cmd_data.v(2) < param.idling.desired_velo_limit);
    }

    dbgss << boost::format("Idling cond: outer[%d] podom[%d] pdes[%d] vodom[%d] vdes[%d]") %
                 idling_rule_outer % idling_rule_podom % idling_rule_pdes % idling_rule_vodom %
                 idling_rule_vdes << endl;

    if (idling_state == NOIDLING) {
        if (idling_rule_podom && idling_rule_pdes && idling_rule_vodom && idling_rule_vdes) {
            ROS_INFO("[N3Ctrl] Go into idling automatically.");
            idling_state = AUTOIDLING;
            idling_start_time = now_time;
        } else if (idling_rule_outer) {
            ROS_WARN("[N3Ctrl] Go into idling by outer instruction. [%d%d%d%d]",
                     idling_rule_podom,
                     idling_rule_pdes,
                     idling_rule_vodom,
                     idling_rule_vdes);
            idling_state = OUTERIDLING;
            idling_start_time = now_time;
        }
    } else if (idling_rule_outer) {
        ROS_WARN("[N3Ctrl] Go into idling by outer instruction. [%d%d%d%d]",
                 idling_rule_podom,
                 idling_rule_pdes,
                 idling_rule_vodom,
                 idling_rule_vdes);
        idling_state = OUTERIDLING;
        idling_start_time = now_time;
    }

    else if (idling_state == AUTOIDLING) {
        if (idling_rule_podom && idling_rule_pdes && idling_rule_vdes) {
            // keep auto idling
        } else {
            ROS_INFO("[N3Ctrl] Leave idling automatically.[%d%d%d]",
                     idling_rule_podom,
                     idling_rule_pdes,
                     idling_rule_vdes);
            idling_state = NOIDLING;
        }
    } else if (idling_state == OUTERIDLING) {
        if (idling_rule_outer) {
            // keep auto idling
        } else {
            ROS_WARN("[N3Ctrl] Leave idling by outer instruction.");
            idling_state = NOIDLING;
        }
    } else {
        ROS_ASSERT(false && "Invalid logic branch!");
    }
}

void N3CtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom) {

    static State_t last_state = State_t::DIRECT_CTRL;
    if (last_state == State_t::DIRECT_CTRL && 
        (state == State_t::JS_CTRL || state == State_t::CMD_HOVER || state == State_t::AUTO_TAKEOFF)) {

        takeoff_land.landed = false; // Always holds
    }

    last_state = state;

    if (state == State_t::DIRECT_CTRL) {

        takeoff_land.landed = true;
        return; // No need of other decisions
    }

    // land_detector parameters
    constexpr double POSITION_DEVIATION_C = -0.5; // Constraint 1: target position below real position for POSITION_DEVIATION_C meters.
    constexpr double VELOCITY_THR_C = 0.1;		  // Constraint 2: velocity below VELOCITY_MIN_C m/s.
    constexpr double TIME_KEEP_C = 3.0;			  // Constraint 3: Time(s) the Constraint 1&2 need to keep.

    static ros::Time time_C12_reached; // time_Constraints12_reached
    static bool is_last_C12_satisfy;
    if (takeoff_land.landed) {

        time_C12_reached = ros::Time::now();
        is_last_C12_satisfy = false;
    } else {

        bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
        if (C12_satisfy && !is_last_C12_satisfy) {

            time_C12_reached = ros::Time::now();
        } else if (C12_satisfy && is_last_C12_satisfy) {

            if ((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) { //Constraint 3 reached
            
                takeoff_land.landed = true;
                ROS_INFO("true");
            }
        }

        is_last_C12_satisfy = C12_satisfy;
    }
}