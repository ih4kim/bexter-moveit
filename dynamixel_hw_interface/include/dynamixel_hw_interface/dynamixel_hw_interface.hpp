#include "dynamixel_sdk/dynamixel_sdk.h"
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

class MyRobot : public hardware_interface::RobotHW 
{   
        dynamixel::PortHandler* portHandler_;
        dynamixel::PacketHandler* packetHandler_;
        dynamixel::GroupSyncWrite groupSyncWrite_;
        dynamixel::GroupSyncRead groupSyncRead_;
    public:
        struct JointID {
            std::string name;
            int id;
            int Kp;
            int Kd;
            int Ki;
        };
        MyRobot(ros::NodeHandle& nh);
        ~MyRobot();
        void update(const ros::TimerEvent& e);
        void init(std::vector<JointID> joint_ids);
        void read();
        void write(ros::Duration elapsed_time);
    private:
        void print_result(int dxl_comm_result, uint8_t dxl_error, std::string message);
        int uint_to_int(uint32_t bit);
        ros::NodeHandle nh_;
        double loop_hz_;
        ros::Timer my_control_loop_;
        ros::Duration elapsed_time_;
        boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
        std::vector<JointID> joint_ids_;

        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::EffortJointInterface effort_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;
        
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::EffortJointSaturationInterface effortJointSaturationInterface;
        joint_limits_interface::PositionJointSaturationInterface positionJointSaturationInterface;
        

        double joint_position_[3];
        double joint_velocity_[3];
        double joint_effort_[3];
        double joint_effort_command_[2];
        double joint_position_command_[3];
};