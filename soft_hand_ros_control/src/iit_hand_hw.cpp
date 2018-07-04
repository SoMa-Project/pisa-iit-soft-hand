/** @file iit_hand_hw.cpp
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

#include "soft_hand_ros_control/iit_hand_hw.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(iit_hand_hw::IITSH_HW, hardware_interface::RobotHW)

namespace iit_hand_hw {
    IITSH_HW::IITSH_HW() {}

    IITSH_HW::~IITSH_HW() {
        stop();
    }

    bool IITSH_HW::init(ros::NodeHandle &n, ros::NodeHandle &robot_hw_nh) {
        nh_ = robot_hw_nh;

        // Initializing subscribers and publishers
        hand_meas_sub = nh_.subscribe(std::string(HAND_MEAS_TOPIC), 1000, &iit_hand_hw::IITSH_HW::callBackMeas, this);
        hand_curr_sub = nh_.subscribe(std::string(HAND_CURR_TOPIC), 1000, &iit_hand_hw::IITSH_HW::callBackCurr, this);
        hand_ref_pub = nh_.advertise<qb_interface::handRef>(std::string(HAND_REF_TOPIC), 1000);

        // Initializing hand curr and meas variables
        hand_meas = 0.0; prev_hand_meas = 0.0;
        hand_curr = 0; prev_hand_curr = 0;
        prev_pos = 0.0;

        initHandVars();

        return start();
    }

    /* Callback function for the position subscriber to qb_interface */
    void IITSH_HW::callBackMeas(const qb_interface::handPosConstPtr& pos_msg){
        hand_meas = pos_msg->closure[0];
    }

    /* Callback function for the current subscriber to qb_interface */
    void IITSH_HW::callBackCurr(const qb_interface::handCurrentConstPtr& curr_msg){
        hand_curr = curr_msg->current[0];
    }

    /* Function for initializing correctly the hand variables without NaNs */
    bool IITSH_HW::initHandVars(){
        // Writing first non NaN measurement and current to prev variables
        bool hand_meas_good = false;
        bool hand_curr_good = false;

        while(!hand_meas_good && !hand_curr_good){
          if(!std::isnan(hand_meas)){
            prev_hand_meas = hand_meas;
            hand_meas_good = true;
            if(DEBUG) std::cout << "Found good hand measurement: " << prev_hand_meas << "." << std::endl;
          }
          if(!std::isnan(hand_curr)){
            prev_hand_curr = hand_curr;
            hand_curr_good = true;
            if(DEBUG) std::cout << "Found good hand current: " << prev_hand_curr << "." << std::endl;
          }

          if(DEBUG) std::cout << "Exiting initHandVars." << std::endl;
        }

    }

    bool IITSH_HW::start() {
        // construct a new device (interface and state storage)
        device_ = std::make_shared<IITSH_HW::IITSH_device>();

        //nh_.param("device_id", device_id_, BROADCAST_ID);
        // nh_.param("device_id", device_id_, 1);

        // TODO: use transmission configuration to get names directly from the URDF model
        if (ros::param::get("iit_hand/joints", this->device_->joint_names)) {
            if (this->device_->joint_names.size() != N_SYN) {
                ROS_ERROR("This robot has 1 joint, you must specify 1 name only until more synergies are not included");
            }
        }
        else {
            ROS_ERROR("No joints to be handled, ensure you load a yaml file naming the joint names this hardware interface refers to.");
            throw std::runtime_error("No joint name specification");
        }
        if (!urdf_model_.initParam("robot_description")) {
            ROS_ERROR("No URDF model in the robot_description parameter, this is required to define the joint limits.");
            throw std::runtime_error("No URDF model available");
        }

        // initialize and set to zero the state and command values
        device_->init();
        device_->reset();

        // general joint to store information
        boost::shared_ptr<const urdf::Joint> joint;

        // create joint handles given the list
        for (unsigned int i = 0; i < N_SYN; ++i) {
            ROS_INFO_STREAM("Handling joint: " << this->device_->joint_names[i]);

            // get current joint configuration
            joint = urdf_model_.getJoint(this->device_->joint_names[i]);
            if (!joint.get()) {
                ROS_ERROR_STREAM("The specified joint "
                                 << this->device_->joint_names[i]
                                 << " can't be found in the URDF model. Check that you loaded an URDF model in the robot description, or that you spelled correctly the joint name.");
                throw std::runtime_error("Wrong joint name specification");
            }

            // joint state handle
            hardware_interface::JointStateHandle state_handle(this->device_->joint_names[i],
                                                              &this->device_->joint_position[i],
                                                              &this->device_->joint_velocity[i],
                                                              &this->device_->joint_effort[i]);

            state_interface_.registerHandle(state_handle);

            // effort command handle
            hardware_interface::JointHandle joint_handle = hardware_interface::JointHandle(
                        state_interface_.getHandle(this->device_->joint_names[i]),
                        &this->device_->joint_position_command[i]);

            position_interface_.registerHandle(joint_handle);

            registerJointLimits(this->device_->joint_names[i],
                                joint_handle,
                                &urdf_model_,
                                &this->device_->joint_lower_limits[i],
                                &this->device_->joint_upper_limits[i],
                                &this->device_->joint_effort_limits[i]);
        }

        // register ros-controls interfaces
        this->registerInterface(&state_interface_);
        this->registerInterface(&position_interface_);
        
        return true;

    }

    void IITSH_HW::stop() {
        usleep(2000000);
    }

    void IITSH_HW::read(const ros::Time &time, const ros::Duration &period) {
        // Position and current are already read from hand by the subscribers but check if NaN
        float non_nan_hand_meas; short int non_nan_hand_curr;

        // Check for NaN in hand measurement and update prev meas if not Nan
        if(std::isnan(hand_meas)){
          non_nan_hand_meas = prev_hand_meas;
        } else {
          non_nan_hand_meas = hand_meas;
          prev_hand_meas = hand_meas;
        }

        // Check for NaN in hand current and update prev curr if not NaN
        if(std::isnan(hand_curr)){
          non_nan_hand_curr = prev_hand_curr;
        } else {
          non_nan_hand_curr = hand_curr;
          prev_hand_curr = hand_curr;
        }
          
        // Setting the inputs using the obtained hand_meas 
        static float inputs[2];                             // Two elements for future (SoftHand 2.5)
        inputs[0] = non_nan_hand_meas;
        inputs[1] = 0.0;

        // Setting the currents using the obtained hand_curr
        static short int currents[2];                       // Two elements for future (SoftHand 2.5)
        currents[0] = non_nan_hand_curr;
        currents[1] = 0;

        // Fill the state variables
        for(int j = 0; j < N_SYN; j++){
          this->device_->joint_position_prev[j] = this->device_->joint_position[j];
          this->device_->joint_position[j] = (double)(inputs[0]/MAX_HAND_MEAS);
          this->device_->joint_effort[j] = currents[0]*1.0;
          this->device_->joint_velocity[j] = filters::exponentialSmoothing((this->device_->joint_position[j]-this->device_->joint_position_prev[j])/period.toSec(), this->device_->joint_velocity[j], 0.2);
        }

        // std::cout << "Measurement is " << this->device_->joint_position[0] << "!" << std::endl;
        // std::cout << "Previous is " << this->device_->joint_position_prev[0] << "!" << std::endl;
        // std::cout << "Current is " << this->device_->joint_effort[0] << "!" << std::endl;

        return;
    }

    void IITSH_HW::write(const ros::Time &time, const ros::Duration &period) {
        // Enforce limits using the period
        pj_sat_interface_.enforceLimits(period);
        pj_limits_interface_.enforceLimits(period);

        // Write to the hand the command given by controller manager
        float pos;
        pos = (float)(MAX_HAND_MEAS*this->device_->joint_position_command[0]);

        // Check for NaN in hand command and update prev pos if not Nan
        if(std::isnan(pos)){
          pos = prev_pos;
        } else {
          prev_pos = pos;
        }
        
        // std::cout << "Command is " << pos << "!" << std::endl;

        set_input(pos);

        return;
    }

    void IITSH_HW::registerJointLimits(const std::string &joint_name,
                                       const hardware_interface::JointHandle &joint_handle,
                                       const urdf::Model * const urdf_model,
                                       double * const lower_limit,
                                       double * const upper_limit,
                                       double * const effort_limit) {

        *lower_limit = -std::numeric_limits<double>::max();
        *upper_limit = std::numeric_limits<double>::max();
        *effort_limit = std::numeric_limits<double>::max();

        joint_limits_interface::JointLimits limits;
        bool has_limits = false;
        joint_limits_interface::SoftJointLimits soft_limits;
        bool has_soft_limits = false;

        if (urdf_model != NULL) {
            const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);

            if (urdf_joint != NULL) {
                // Get limits from the URDF file.
                if (joint_limits_interface::getJointLimits(urdf_joint, limits))
                    has_limits = true;

                if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
                    has_soft_limits = true;
            }
        }

        if (!has_limits)
            return;

        if (limits.has_position_limits) {
            *lower_limit = limits.min_position;
            *upper_limit = limits.max_position;
        }
        if (limits.has_effort_limits)
            *effort_limit = limits.max_effort;

        if (has_soft_limits) {
            const joint_limits_interface::EffortJointSoftLimitsHandle
                    limits_handle(joint_handle, limits, soft_limits);
            ej_limits_interface_.registerHandle(limits_handle);
        }
        else {
            const joint_limits_interface::EffortJointSaturationHandle
                    sat_handle(joint_handle, limits);
            ej_sat_interface_.registerHandle(sat_handle);
        }
    }

    void IITSH_HW::set_input(float pos) {
        static float inputs[2];

        inputs[0] = pos;
        inputs[1] = 0.0;

        // Creating the Hand Ref message for qb_interface
        qb_interface::handRef tmp_ref_msg;
        tmp_ref_msg.closure.push_back(inputs[0]);

        // Publishing the command to robot though qb_interface
        hand_ref_pub.publish(tmp_ref_msg);

        return;
    }
}
