#include <dynamixel_hw_interface/dynamixel_hw_interface.hpp>
#include <ros/console.h>
#include <cmath> 

#define ADDR_GOAL_POSITION      116
#define LEN_GOAL_POSITION       4
#define ADDR_PRESENT_POSITION   132
#define LEN_PRESENT_POSITION    4
#define ADDR_TORQUE_ENABLE      64
#define TORQUE_ENABLE           1
#define TORQUE_DISABLE          0
#define BIT_TO_RAD              0.087891f
#define ADDR_OPERATING_MODE     11
#define EXTENDED_POSITION       4
#define POSITION                3
#define MAX_POSITION            4294967295
#define M_PI                    3.14159265358979323846
#define ADDR_DRIVE_MODE         10
#define REVERSE_MODE            1
#define ANGLE_TRESHOLD          3.14159

MyRobot::MyRobot(ros::NodeHandle& nh) : 
    nh_(nh),
    portHandler_(dynamixel::PortHandler::getPortHandler("/dev/ttyUSB0")),
    packetHandler_(dynamixel::PacketHandler::getPacketHandler(2.0)),
    groupSyncWrite_(portHandler_, packetHandler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION),
    groupSyncRead_(portHandler_, packetHandler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
{
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = 10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    my_control_loop_ = nh_.createTimer(update_freq, &MyRobot::update, this);
}

MyRobot::~MyRobot() {
    for (auto joint_id : joint_ids_) {
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, joint_id.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }
}

void MyRobot::print_result(int dxl_comm_result, uint8_t dxl_error, std::string message) {
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_WARN("%s %d", packetHandler_->getTxRxResult(dxl_comm_result), dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
        ROS_WARN("%s %d", packetHandler_->getRxPacketError(dxl_error), dxl_error);
    }
    else
    {
        ROS_INFO("%s",message.c_str());
    }
}

void MyRobot::init(std::vector<JointID> joint_ids) {
    joint_ids_ = joint_ids;

    if (portHandler_->openPort()) {
        ROS_INFO("Succeeded to open port!");
    }
    else {
        ROS_WARN("Failed to open the port!");
    }
    
    if (portHandler_->setBaudRate(1000000)) {
        ROS_INFO("Succeeded to change baudrate!");
    }
    else {
        ROS_WARN("Failed to change baudrate!");
    }
    int dxl_comm_result = COMM_TX_FAIL;
    while (dxl_comm_result != COMM_SUCCESS) {
        int id_count = 0;
        for (auto joint_id : joint_ids_) {
            hardware_interface::JointStateHandle jointStateHandle(joint_id.name, &joint_position_[id_count], &joint_velocity_[id_count], &joint_effort_[id_count]);
            joint_state_interface_.registerHandle(jointStateHandle);
            hardware_interface::JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[id_count]);
            joint_position_command_[id_count] = 0;
            position_joint_interface_.registerHandle(jointPositionHandle);
            id_count++;
            // Enable Dynamixel Torque
            bool dxl_addparam_result = false;  
            uint8_t dxl_error = 0;
            std::stringstream message;

            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, joint_id.id, ADDR_DRIVE_MODE, REVERSE_MODE, &dxl_error);
            message = std::stringstream();
            message << "Dynamixel "<< joint_id.id << " has been set to reverse mode";
            print_result(dxl_comm_result, dxl_error, message.str());
            if (dxl_comm_result != COMM_SUCCESS) {
                break;
            }

            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, joint_id.id, ADDR_OPERATING_MODE, POSITION, &dxl_error);
            message = std::stringstream();
            message << "Dynamixel "<< joint_id.id << " has been set to position mode";
            print_result(dxl_comm_result, dxl_error, message.str());
            if (dxl_comm_result != COMM_SUCCESS) {
                break;
            }
            
            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, joint_id.id, ADDR_OPERATING_MODE, EXTENDED_POSITION, &dxl_error);
            message = std::stringstream();
            message << "Dynamixel "<< joint_id.id << " has been set to extended mode";
            print_result(dxl_comm_result, dxl_error, message.str());
            if (dxl_comm_result != COMM_SUCCESS) {
                break;
            }

            
            
            dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, joint_id.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
            message = std::stringstream();
            message << "Dynamixel " << joint_id.id << " has been successfully connected";
            print_result(dxl_comm_result, dxl_error, message.str());
            if (dxl_comm_result != COMM_SUCCESS) {
                break;
            }

            dxl_addparam_result = groupSyncRead_.addParam(joint_id.id);
            if (dxl_addparam_result != true)
            {
                ROS_DEBUG("[ID:%d] groupSyncRead addparam failed", joint_id.id);
            }
        }
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&position_joint_interface_);
}

void MyRobot::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

int MyRobot::uint_to_int(uint32_t bit) {
    // check if value is closer to 0 or highest possible value
    if (bit > MAX_POSITION - bit) {
        // Use inverse to get proper position
        return ((~bit)+1)*-1;
    }
    return bit;
}

void MyRobot::read() {
    int dxl_comm_result = groupSyncRead_.txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS) 
        printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
    int id_count = 0;
    for (auto joint_id : joint_ids_) {
        bool dxl_getdata_result = groupSyncRead_.isAvailable(joint_id.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        if (!dxl_getdata_result)
        {
            ROS_WARN("[ID:%03d] groupSyncRead getdata failed", joint_id.id);
        }
        else {
            joint_position_[id_count] = (uint_to_int(groupSyncRead_.getData(joint_id.id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION))*BIT_TO_RAD)*M_PI/180.0;
            //ROS_INFO("ID %d: pos: %.2f", joint_id.id, joint_position_[id_count]);
        }
        id_count++;
    }
}

void MyRobot::write(ros::Duration elapsed_time) {
    std::vector<std::vector<uint8_t>> param_goal_position_vec;
    int count = 0;
    bool safe_rotation = true;
    for (auto joint_id : joint_ids_) {
        // Obtain corresponding goal position
        std::vector<uint8_t> param_goal_position;
        int pos = int((joint_position_command_[count]*180.0/M_PI)/BIT_TO_RAD); // In bits
        double pos_rad = joint_position_command_[count];
        double current_rad = joint_position_[count];
        // Check if the joint position difference is too big
        if (abs(pos_rad-current_rad) > ANGLE_TRESHOLD) {
            ROS_WARN("[ID:%03d] Dangerous jump in angle!! %.2f -> %.2f", joint_id.id, current_rad, pos_rad);
            safe_rotation = false;
            break;
        }
        //ROS_INFO("%d pos: %.2f", joint_id.id, pos_rad);
        param_goal_position.push_back(DXL_LOBYTE(DXL_LOWORD(pos)));
        param_goal_position.push_back(DXL_HIBYTE(DXL_LOWORD(pos)));
        param_goal_position.push_back(DXL_LOBYTE(DXL_HIWORD(pos)));
        param_goal_position.push_back(DXL_HIBYTE(DXL_HIWORD(pos)));
        param_goal_position_vec.push_back(param_goal_position);
        bool dxl_addparam_result = groupSyncWrite_.addParam(joint_id.id, param_goal_position_vec.back().data());
        if (dxl_addparam_result != true)
        {
            ROS_WARN("[ID:%03d] groupSyncWrite addParam failed", joint_id.id);
        }
        count++;
    }
    if (safe_rotation) {
        int dxl_comm_result = groupSyncWrite_.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) 
            printf("%s\n", packetHandler_->getTxRxResult(dxl_comm_result));
    }
    
    // Clear syncwrite parameter storage
    groupSyncWrite_.clearParam();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dynamixel_hw_interface_node");
    ros::NodeHandle nh;
    ros::MultiThreadedSpinner spinner(2);
    MyRobot dynamixel_interface(nh);
    // Create vector for initializing nodes
    MyRobot::JointID joint_1 = {
        .name = "joint_1",
        .id = 1
    };
    MyRobot::JointID joint_2 = {
        .name = "joint_2",
        .id = 2
    };
    MyRobot::JointID joint_3 = {
        .name = "joint_3",
        .id = 3
    };
    std::vector<MyRobot::JointID> joint_ids;
    joint_ids.push_back(joint_1);
    joint_ids.push_back(joint_2);
    joint_ids.push_back(joint_3);
    dynamixel_interface.init(joint_ids);
    spinner.spin();
    return 0;
} 