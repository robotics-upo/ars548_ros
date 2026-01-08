/**
 * @file ars548_driver.cpp
 */

#include "ars548_driver/ars548_driver.hpp"

using namespace std::chrono_literals;

// Defaults if not provided by parameters
#define DEFAULT_ARS548_LOCAL_IP "10.13.1.166"
#define DEFAULT_ARS548_IP "10.13.1.113"
#define DEFAULT_ARS548_PORT 42102
#define DEFAULT_ARS548_MULTICAST_IP "224.0.2.2"
#define DEFAULT_FRAME_ID "ARS_548" 

ars548_driver::ars548_driver() : Node("ars_548_driver"), 
    fd(-1),
    run_thread_(false),
    modifierObject(cloud_msgObj), 
    modifierDetection(cloud_msgDetect)
{
    // Parameter declaration
    this->declare_parameter("localIP", DEFAULT_ARS548_LOCAL_IP);
    this->declare_parameter("radarIP", DEFAULT_ARS548_IP);
    this->declare_parameter("radarPort", DEFAULT_ARS548_PORT);
    this->declare_parameter("frameID", DEFAULT_FRAME_ID);
    this->declare_parameter("multicastIP", DEFAULT_ARS548_MULTICAST_IP);
    this->declare_parameter("overrideStamp", true);

    this->ars548_local_ip = this->get_parameter("localIP").as_string();
    this->radar_ip = this->get_parameter("radarIP").as_string();
    this->ars548_multicast_ip = this->get_parameter("multicastIP").as_string();
    this->ars548_port = this->get_parameter("radarPort").as_int();
    this->frame_ID = this->get_parameter("frameID").as_string();
    this->override_stamp = this->get_parameter("overrideStamp").as_bool();

    // Initialize Publishers
    statusPublisher = create_publisher<ars548_messages::msg::Status>("Status", 10);
    objectsPublisher = create_publisher<ars548_messages::msg::ObjectList>("ObjectList", 10);
    detectionsPublisher = create_publisher<ars548_messages::msg::DetectionList>("DetectionList", 10);
    directionPublisher = create_publisher<geometry_msgs::msg::PoseArray>("DirectionVelocity", 10);
    objectsCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudObject", 10);
    detectionsCloudPublisher = create_publisher<sensor_msgs::msg::PointCloud2>("PointCloudDetection", 10);

    // Initialize PointCloud Modifiers
    modifierObject.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vx", 1, sensor_msgs::msg::PointField::FLOAT32,
        "vy", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    modifierObject.clear();
    modifierObject.reserve(ARS548_MAX_OBJECTS);

    modifierDetection.setPointCloud2Fields(8,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "v", 1, sensor_msgs::msg::PointField::FLOAT32,
        "r", 1, sensor_msgs::msg::PointField::FLOAT32,
        "RCS", 1, sensor_msgs::msg::PointField::INT8,
        "azimuth", 1, sensor_msgs::msg::PointField::FLOAT32,
        "elevation", 1, sensor_msgs::msg::PointField::FLOAT32
    );
    modifierDetection.clear();
    modifierDetection.reserve(ARS548_MAX_DETECTIONS);
    cloud_Direction.poses.reserve(ARS548_MAX_OBJECTS);

    // Start receive thread
    run_thread_ = true;
    receive_thread_ = std::make_unique<std::thread>(&ars548_driver::receive_data_loop, this);
    
    RCLCPP_INFO(this->get_logger(), "ARS548 Driver started. Listening on %s Multicast %s", 
        ars548_local_ip.c_str(), ars548_multicast_ip.c_str());
}

ars548_driver::~ars548_driver()
{
    run_thread_ = false;
    if (receive_thread_ && receive_thread_->joinable()) {
        receive_thread_->join();
    }
    int socket_fd = fd.load();
    if (socket_fd >= 0) {
        shutdown(socket_fd, SHUT_RDWR);
        close(socket_fd);
    }
}

void ars548_driver::receive_data_loop()
{
    int socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        RCLCPP_ERROR(this->get_logger(), "Socket creation failed: %s", strerror(errno));
        rclcpp::shutdown();
        return;
    }
    fd.store(socket_fd);

    // Allow reuse
    u_int yes = 1;
    if (setsockopt(socket_fd, SOL_SOCKET, SO_REUSEADDR, (char*)&yes, sizeof(yes)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "setsockopt SO_REUSEADDR failed: %s", strerror(errno));
        close(socket_fd);
        fd.store(-1);
        rclcpp::shutdown();
        return;
    }

    // Set receive timeout to allow thread checking
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000; // 100ms
    if (setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv) < 0) {
        RCLCPP_ERROR(this->get_logger(), "setsockopt SO_RCVTIMEO failed: %s", strerror(errno));
        close(socket_fd);
        fd.store(-1);
        rclcpp::shutdown();
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(ars548_port);

    if (::bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Bind failed: %s", strerror(errno));
        close(socket_fd);
        fd.store(-1);
        rclcpp::shutdown();
        return;
    }

    // Join multicast group
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(ars548_multicast_ip.c_str());
    mreq.imr_interface.s_addr = inet_addr(ars548_local_ip.c_str());

    if (setsockopt(socket_fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*)&mreq, sizeof(mreq)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "%s", strerror(errno));
        close(socket_fd);
        fd.store(-1);
        rclcpp::shutdown();
        return;
    }

    unsigned int addrlen = sizeof(addr);
    
    RCLCPP_INFO(this->get_logger(), "Waiting for data...");

    while (run_thread_ && rclcpp::ok())
    {
        int nbytes = recvfrom(socket_fd, msgbuf, MSGBUFSIZE, 0, (struct sockaddr*)&addr, &addrlen);
        
        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout, just continue and check run_thread_
                continue;
            } else {
                RCLCPP_WARN(this->get_logger(), "recvfrom failed: %s", strerror(errno));
                // Wait a bit before retrying to avoid log spam if network is down
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
        } else if (nbytes > 0) {
             switch (nbytes)
            {
            case STATUS_MESSAGE_PAYLOAD:
                {
                    struct UDPStatus status;
                    if(status.receiveStatusMsg(nbytes, msgbuf))
                    {
                        auto statusMessage = status.toMsg();
                        statusPublisher->publish(statusMessage);
                    }
                }
                break;
            case OBJECT_MESSAGE_PAYLOAD:
                {
                    struct ObjectList object_list;
                    // Ensure buffer size is sufficient before cast usually, but payload check helps
                    object_list = *((struct ObjectList *)msgbuf);
                    object_list.changeEndianness();
                    if(object_list.isValid()) {
                        auto now = this->now();
                        objectsPublisher->publish(object_list.toMsg(frame_ID, now, override_stamp));
                        object_list.fillObjectCloud(cloud_msgObj, modifierObject, frame_ID, now, override_stamp);
                        objectsCloudPublisher->publish(cloud_msgObj);
                        directionPublisher->publish(object_list.getDirectionMessage(frame_ID, now, override_stamp));
                    }
                }
                break;
            case DETECTION_MESSAGE_PAYLOAD:
                {
                    struct DetectionList detection_list;
                    detection_list = *((struct DetectionList *)msgbuf);
                    detection_list.changeEndianness();
                    if (detection_list.isValid()) {
                        auto now = this->now();
                        detectionsPublisher->publish(detection_list.toMsg(frame_ID, now, override_stamp));
                        detection_list.fillDetectionCloud(cloud_msgDetect, modifierDetection, frame_ID, now, override_stamp);
                        detectionsCloudPublisher->publish(cloud_msgDetect);
                    }
                }
                break;
            default:
                // Unknown packet size
                break;
            }
        }
    }

    close(socket_fd);
    fd.store(-1);
    RCLCPP_INFO(this->get_logger(), "ARS548 receive thread stopped.");
}
