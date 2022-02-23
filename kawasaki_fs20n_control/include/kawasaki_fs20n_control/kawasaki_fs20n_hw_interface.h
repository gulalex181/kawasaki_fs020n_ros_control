#ifndef KAWASAKI_INTERFACE_H
#define KAWASAKI_INTERFACE_H

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <string.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <regex>

#include <ros/ros.h>
#include <ros_control_boilerplate/generic_hw_interface.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#define DEG_TO_RAD 0.01745329251
#define RAD_TO_DEG 57.2957795131

namespace kawasaki_ns {

/** \brief Hardware interface for a robot */
class KawasakiHWInterface: public ros_control_boilerplate::GenericHWInterface {
public:
    /**
     * \brief Constructor
     * \param nh - Node handle for topics.
     */
    KawasakiHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);

    /**
     * \brief Destructor
     */
    ~KawasakiHWInterface();

    /** \brief Initialize the robot hardware interface */
    virtual void init();

    /** \brief Read the state from the robot hardware. */
    virtual void read(ros::Duration& elapsed_time);

    /** \brief Write the command to the robot hardware. */
    virtual void write(ros::Duration& elapsed_time);

    /** \breif Enforce limits for all values before writing */
    virtual void enforceLimits(ros::Duration& period);

protected:
    int socket_fd = 0;
    struct sockaddr_in serv_addr;
    char buffer[100] = {0};

    std::vector<double> joint_position_last = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    void connectToRobot();
    void disconnectFromRobot();
    void connectionError(std::string msg);
    void sendCmdToRobot(std::string cmd);
    void receiveAnsFromRobot();

    std::string getStartMovementProgramCmd();
    std::string getStopMovementProgramCmd();
    std::string getStopCurrentMovementCmd();
    std::string getGoToPoseCmd();
    std::string getStopMovementAndCloseConnectionCmd();
    
    std::string getVersionFromParamServer();
    std::vector<std::string> splitStringToVector(const std::string &s, char delim);
};  // class

}  // namespace ros_control_boilerplate

#endif
