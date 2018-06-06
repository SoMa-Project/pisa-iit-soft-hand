/** @file iit_hand_hw.h
 *  @brief A C++ class which implements a ros_control driver for the Pisa/IIT
 * softhand.
 *  @author Centro di Ricerca "E. Piaggio"
 *  @author Murilo Martins (murilo.martins@ocado.com)
 *
 * Copyright (c) 2014, Centro di Ricerca "E. Piaggio"
 * Copyright (c) 2016, Ocado Technology
 * All rights reserved.
 *
 * This file is part of soft_hand_ros_control:
 * https://github.com/CentroEPiaggio/pisa-iit-soft-hand/tree/master/soft_hand_ros_control
 *
 * soft_hand_ros_control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.

 * soft_hand_ros_control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with soft_hand_ros_control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <memory>

// ROS headers
#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/Duration.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <control_toolbox/filters.h>
#include <urdf/model.h>

// qb tools
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <unistd.h>
#include "soft_hand_ros_control/definitions.h"

#include "soft_hand_ros_control/setControlMode.h"

// QB interface messages
#include <qb_interface/handPos.h>
#include <qb_interface/handCurrent.h>
#include <qb_interface/handRef.h>


namespace iit_hand_hw {
class IITSH_HW: public hardware_interface::RobotHW {
public:
    /**
     * @brief Constructor.
     */
    IITSH_HW();
    /**
     * @brief Destructor.
     */
    virtual ~IITSH_HW();
    

    /* Callback functions for the subscribers to qb_interface */
    void callBackMeas(const qb_interface::handPosConstPtr& pos_msg);
    void callBackCurr(const qb_interface::handCurrentConstPtr& curr_msg);

    /* Function for initializing correctly the hand variables without NaNs */
    bool initHandVars();

    /**
     * @brief Method implementing all the initialisation required by the class.
     * @param unused node handle
     * @param robot (hand) hardware-specific node handle
     * @return True if initialisation was successful; False otherwise.
     */
    virtual bool init(ros::NodeHandle& n, ros::NodeHandle& robot_hw_nh);

    virtual void read(const ros::Time& time, const ros::Duration& period);
    virtual void write(const ros::Time& time, const ros::Duration& period);

    void registerJointLimits(const std::string& joint_name,
                             const hardware_interface::JointHandle& joint_handle,
                             const urdf::Model * const urdf_model,
                             double * const lower_limit,
                             double * const upper_limit,
                             double * const effort_limit);

    void set_input(float pos);

    struct IITSH_device {
        std::vector<std::string> joint_names;

        std::vector<double>
        joint_upper_limits,
        joint_lower_limits,
        joint_effort_limits;

        std::vector<double>
        joint_position,
        joint_position_prev,
        joint_velocity,
        joint_effort,
        joint_position_command;

        void init() {
            joint_position.resize(N_SYN);
            joint_position_prev.resize(N_SYN);
            joint_velocity.resize(N_SYN);
            joint_effort.resize(N_SYN);
            joint_position_command.resize(N_SYN);

            joint_lower_limits.resize(N_SYN);
            joint_upper_limits.resize(N_SYN);
            joint_effort_limits.resize(N_SYN);
        }

        void reset() {
            for (unsigned int i = 0; i < N_SYN; ++i) {
                joint_position[i] = 0.0;
                joint_position_prev[i] = 0.0;
                joint_velocity[i] = 0.0;
                joint_effort[i] = 0.0;
                joint_position_command[i] = 0.0;
            }
        }
    };

    std::shared_ptr<IITSH_HW::IITSH_device> device_;
private:
    
    ros::NodeHandle nh_;

    // A subscriber for getting hand positions from qb_interface
    ros::Subscriber hand_meas_sub;

    // A subscriber for getting hand currents from qb_interface
    ros::Subscriber hand_curr_sub;

    // A publisher for publishing hand commands to qb_interface
    ros::Publisher hand_ref_pub;

    // Variables for temporarily storing hand measurement and current read from topics
    float hand_meas;
    short int hand_curr;

    // Variables for storing previous hand measurement and current which are not NaN (used for NaN problem)
    float prev_hand_meas;
    short int prev_hand_curr;

    // Variables for storing previous hand command which is not NaN (used for NaN problem)
    float prev_pos;

    int device_id_;

    urdf::Model urdf_model_;

    hardware_interface::JointStateInterface state_interface_;
    hardware_interface::PositionJointInterface position_interface_;

    joint_limits_interface::PositionJointSaturationInterface pj_sat_interface_;
    joint_limits_interface::PositionJointSoftLimitsInterface pj_limits_interface_;
    joint_limits_interface::EffortJointSaturationInterface ej_sat_interface_;
    joint_limits_interface::EffortJointSoftLimitsInterface ej_limits_interface_;


    bool start();
    void stop();




};
}
