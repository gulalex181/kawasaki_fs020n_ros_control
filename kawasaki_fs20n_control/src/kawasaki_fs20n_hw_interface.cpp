#include <kawasaki_fs20n_control/kawasaki_fs20n_hw_interface.h>

namespace kawasaki_ns {

KawasakiHWInterface::KawasakiHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model)
    : ros_control_boilerplate::GenericHWInterface(nh, urdf_model) {
    name_ = "kawasaki_fs20n_hw_interface";
    nh_ = nh;
}

KawasakiHWInterface::~KawasakiHWInterface() {
    disconnectFromRobot();
}

void KawasakiHWInterface::init() {
    // Call parent class version of this function
    ros_control_boilerplate::GenericHWInterface::init();

    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "ready");

    connectToRobot();
}

void KawasakiHWInterface::read(ros::Duration& elapsed_time) {
    std::string pos = "";
    for (int i = 0; i < num_joints_; ++i) {
        joint_position_[i] = joint_position_last[i];
        pos += i + ": '" + std::to_string(joint_position_[i]) + "' ";
    }
    // ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << pos);
}

void KawasakiHWInterface::write(ros::Duration& elapsed_time) {
    // Safety
    //enforceLimits(elapsed_time);

    sendCmdToRobot(getGoToPoseCmd());
}

void KawasakiHWInterface::enforceLimits(ros::Duration& period) {
    // Enforces position and velocity
    //pos_jnt_sat_interface_.enforceLimits(period);
}

void KawasakiHWInterface::connectToRobot() {
    // Create client socket
    socket_fd = socket(AF_INET, SOCK_STREAM, 0);

    if (socket_fd < 0) {
        connectionError("creating socket error");
    }

    // Get connection params from parameter server
    std::string ip_address;
    int port;

    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_ip", ip_address);
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_port", port);
    // If there is no any of the requested param then throw an error
    rosparam_shortcuts::shutdownIfError(name_, error);

    // Configure robot server address structure
    serv_addr.sin_addr.s_addr = inet_addr(ip_address.c_str());
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);

    // Connecting to robot server
    if (connect(socket_fd, (struct sockaddr *) &serv_addr, sizeof(serv_addr)) < 0) {
        connectionError("connecting error");
    }

    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "connected to server");

    // Launch robot movement program.
    sendCmdToRobot(getStartMovementProgramCmd());

    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "robot movement program is launched");
}

void KawasakiHWInterface::connectionError(std::string msg) {
    ROS_ERROR_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << msg);
    ros::shutdown();
    exit(0);
}

void KawasakiHWInterface::disconnectFromRobot() {
    sendCmdToRobot(getStopMovementAndCloseConnectionCmd());
    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "robot movement program is stopped");
    
    close(socket_fd);
    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "disconnected from server");
}

void KawasakiHWInterface::sendCmdToRobot(std::string cmd) {
    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "COMMAND TO ROBOT: '" << cmd << "'");
    
    if (::write(socket_fd, cmd.c_str(), cmd.length()) < 0) {
        connectionError("writing to socker error");
    }

    receiveAnsFromRobot();
}

void KawasakiHWInterface::receiveAnsFromRobot() {
    memset(buffer, 0, strlen(buffer));
    
    int count_of_bytes = ::read(socket_fd, buffer, sizeof(buffer) - 1);
    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "buffer: '" << buffer << "'" << std::endl);
    
    if (count_of_bytes < 0) {
        connectionError("reading from socker error");
    }

    buffer[strlen(buffer)] = '\0';
    
    std::string response = std::string(buffer);
    std::vector<std::string> responseVector = splitStringToVector(response, ' ');

    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "STATUS OF ROBOT:" << std::endl
        << "VERSION: " << responseVector[0] << std::endl
        << "ERROR: " << responseVector[1] << std::endl
        << "SWITCH(RUN): " << responseVector[2] << std::endl
        << "SWITCH(REPEAT): " << responseVector[3] << std::endl
        << "SWITCH(TEACH_LOCK): " << responseVector[4] << std::endl
        << "SWITCH(POWER): " << responseVector[5] << std::endl
        << "TASK(1) == 1: " << responseVector[6] << std::endl
        << "TIMER(1): " << responseVector[7] << std::endl
        << "JT1: " << responseVector[8] << std::endl
        << "JT2: " << responseVector[9] << std::endl
        << "JT3: " << responseVector[10] << std::endl
        << "JT4: " << responseVector[11] << std::endl
        << "JT5: " << responseVector[12] << std::endl
        << "JT6: " << responseVector[13] << std::endl
    );

    for (int i = 0; i < num_joints_; ++i) {
        joint_position_last.at(i) = stod(responseVector[8 + i]) * DEG_TO_RAD;
    }
}

std::string KawasakiHWInterface::getStartMovementProgramCmd() {
    return getVersionFromParamServer() + " 1 0";
}

std::string KawasakiHWInterface::getStopMovementProgramCmd() {
    return getVersionFromParamServer() + " 2 0";
}

std::string KawasakiHWInterface::getStopCurrentMovementCmd() {
    return getVersionFromParamServer() + " 5 0";
}

std::string KawasakiHWInterface::getGoToPoseCmd() {
    std::string joint_positions_str = "";
    
    for(int i = 0; i < num_joints_; ++i) {
        if (i == (num_joints_ - 1)) {
            joint_positions_str += std::to_string(joint_position_command_[i] * RAD_TO_DEG);
        } else {
            joint_positions_str += std::to_string(joint_position_command_[i] * RAD_TO_DEG) + " ";
        }
    }

    return getVersionFromParamServer() + " 6 9 0 0 0 " + joint_positions_str;
}

std::string KawasakiHWInterface::getStopMovementAndCloseConnectionCmd() {
    return getVersionFromParamServer() + " 255 0";
}

std::string KawasakiHWInterface::getVersionFromParamServer() {
    std::string version;

    ros::NodeHandle rpnh(nh_, name_);
    std::size_t error = 0;
    error += !rosparam_shortcuts::get(name_, rpnh, "robot_version", version);
    // If there is no any of the requested param then throw an error
    rosparam_shortcuts::shutdownIfError(name_, error);

    return version;
}

std::vector<std::string> KawasakiHWInterface::splitStringToVector(const std::string &str, char delim) {
    // std::vector<std::string> elems;
    // std::stringstream ss;
    // ss.str(s);
    // std::string item;
    // while (std::getline(ss, item, delim)) {
    //     elems.push_back(item);
    // }

    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "string1: '" << str << "'" << std::endl);
    std::string s = std::regex_replace(str, std::regex("^\\s+|\\s+$"), "");
    ROS_INFO_STREAM_NAMED(name_, "[KawasakiHWInterface]: " << "string2: '" << s << "'" << std::endl);

    std::regex ws_re("\\s+"); // whitespace
    std::vector<std::string> elems{
        std::sregex_token_iterator(s.begin(), s.end(), ws_re, -1), {}
    };

    return elems;
}

} // namespace ros_control_boilerplate
