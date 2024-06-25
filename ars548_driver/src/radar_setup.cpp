/**
 * @file ars548_change_ip.cpp
 * @brief In this file we try to change the radar IP using the according message.
 * 
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "tclap/CmdLine.h"
#include "../include/ars548_driver/ars548_driver.hpp"
#include "../include/ars548_driver/ars548_data.h"

#define RADAR_INTERFACE "10.13.1.166"
#define CONFIGURATION_SOURCE_PORT 42401
#define CONFIGURATION_DESTINATION_PORT 42101
#define CONFIGURATION_METHOD_ID 390
#define CONFIGURATION_MESSAGE_ID 390
#define CONFIGURATION_PDU_LENGTH 56
#define CONFIGURATION_UDP_PAYLOAD 64
#define CONFIGURATION_UDP_LENGTH 72

#define DEFAULT_RADAR_IP "224.0.2.2"
#define NEW_IP "0.0.0.0"

static SensorConfiguration modifyConfiguration(SensorConfiguration c){
    c.ServiceID=ars548_driver::ChangeEndianness(c.ServiceID);
    c.MethodID=ars548_driver::ChangeEndianness(c.MethodID);
    c.PayloadLength=ars548_driver::ChangeEndianness(c.PayloadLength);
    c.Longitudinal=ars548_driver::ChangeEndianness(c.Longitudinal);
    c.Lateral=ars548_driver::ChangeEndianness(c.Lateral);
    c.Vertical=ars548_driver::ChangeEndianness(c.Vertical);
    c.Yaw=ars548_driver::ChangeEndianness(c.Yaw);
    c.Pitch=ars548_driver::ChangeEndianness(c.Pitch);
    c.PlugOrientation=ars548_driver::ChangeEndianness(c.PlugOrientation);
    c.Length=ars548_driver::ChangeEndianness(c.Length);
    c.Width=ars548_driver::ChangeEndianness(c.Width);
    c.Height=ars548_driver::ChangeEndianness(c.Height);
    c.Wheelbase=ars548_driver::ChangeEndianness(c.Wheelbase);
    c.MaximumDistance=ars548_driver::ChangeEndianness(c.MaximumDistance);
    c.FrequencySlot=ars548_driver::ChangeEndianness(c.FrequencySlot);
    c.CycleTime=ars548_driver::ChangeEndianness(c.CycleTime);
    c.TimeSlot=ars548_driver::ChangeEndianness(c.FrequencySlot);
    c.HCC=ars548_driver::ChangeEndianness(c.HCC);
    c.Powersave_Standstill=ars548_driver::ChangeEndianness(c.Powersave_Standstill);
    c.SensorIPAddress_0=ars548_driver::ChangeEndianness(c.SensorIPAddress_0);
    c.SensorIPAddress_1=ars548_driver::ChangeEndianness(c.SensorIPAddress_1);
    c.NewSensorMounting=ars548_driver::ChangeEndianness(c.NewSensorMounting);
    c.NewVehicleParameters=ars548_driver::ChangeEndianness(c.NewVehicleParameters);
    c.NewRadarParameters=ars548_driver::ChangeEndianness(c.NewRadarParameters);
    c.NewNetworkConfiguration=ars548_driver::ChangeEndianness(c.NewNetworkConfiguration);
    return c;
}

/*class ars548_change_ip
{
    private:
    rclcpp::Publisher<ars548_messages::msg::SensorConfiguration>::SharedPtr publisherConfig;
    public:

    ars548_change_ip(): Node("ars548_driver_change_ip")
    {
        
        
        publisherConfig=this->create_publisher<ars548_messages::msg::SensorConfiguration>("NewConfiguration",10);
        
            subscription_=this->create_subscription<ars548_messages::msg::SensorConfiguration>(
            "confihurationCloud",10,std::bind(&ars548_change_ip::top));
        
    }
};
*/


int main(int argc,char* argv[]){
    int fd;
    int nbytes;
    char msgbuf[MSGBUFSIZE];
    struct SensorConfiguration c;
    //Modify elements of the radar 1==Modify, 0==Ignore
    int modifyRadar=0;
    int modifyNetwork=0;
    int modifyPos=0;
    int modifyVehicle=0;
    
    //Radar possition parameters
    float longitudinal;
    float lateral;
    float vertical;
    float yaw;
    float pitch;
    int plugOrientation;
    
    //Vehicle parameters
    float length;
    float width;
    float height;
    float wheelbase;

    //Radar configuration parameters
    int maxDistance=0;
    int freqSlot=0;
    int cycleTime=0;
    int cycleOffset;
    int countryCode=1;
    int powersaveStandstill=0;
    
    //Network Configuration
    std::uint32_t ip;
    std::uint32_t ip1;
    
    TCLAP::CmdLine cmd("Command description message",' ',"0.9");
    
    fd=socket(AF_INET,SOCK_DGRAM,0);
    if (fd<0)
    {
        perror("socket");
        return 1;
    }
    struct sockaddr_in addr;
    u_int yes=1;
    if(
        setsockopt(
            fd,SOL_SOCKET,SO_REUSEADDR,(char*) &yes,sizeof(yes)
        )<0
    )
    {
        perror("Reusing ADDR failed");
        return 1;
    }
    // set up destination address
    //
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
    addr.sin_port = htons(ars548_driver::ars548_Port);
    // bind to receive address
    //
    if (bind(fd, (struct sockaddr*) &addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    // use setsockopt() to request that the kernel join a multicast group
    //
    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(ars548_driver::ars548_IP.c_str());
    mreq.imr_interface.s_addr = inet_addr(RADAR_INTERFACE);
    
    if (
        setsockopt(
            fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
        ) < 0
    ){
        perror("setsockopt");
        return 1;
    }
    unsigned int addrlen = sizeof(addr);
    nbytes = recvfrom(
        fd,
        msgbuf,
        MSGBUFSIZE,
        0,
        (struct sockaddr *) &addr,
        &addrlen
        );
    if(nbytes<0)
    {
        perror("Failed attempt of getting data");
        return 1;
    }
    struct UDPStatus status;
    status = *((struct UDPStatus *)msgbuf);
    int iterator=0;
    while (ars548_driver::receiveStatusMsg(nbytes,msgbuf,status)==false)
    {
        
        nbytes = recvfrom(
        fd,
        msgbuf,
        MSGBUFSIZE,
        0,
        (struct sockaddr *) &addr,
        &addrlen
        );
        if(nbytes<0)
        {
            perror("Failed attempt of getting data");
            return 1;
        }
        if (iterator>=6)
        {
            perror("Could not receive the status data");
            return 1;
        }
        
    }
    std::cout<<"MaxDistance: "<<status.MaximunDistance<<std::endl;
    std::cout<<"Radar IP 1: "<<status.SensorIPAddress_1<<std::endl;
    std::cout<<"Radar IP 0: "<<status.SensorIPAddress_0<<std::endl;
        //To configure the radar possition and orientation in the vehicle
    TCLAP::ValueArg<_Float32>new_x_pos("X","NewXPos","New Longitudinal possition of the radar",false,status.Longitudinal,"float");//TODO
    TCLAP::ValueArg<_Float32>new_y_pos("Y","NewYPos","New Lateral possition of the radar",false,status.Lateral,"float");//TODO
    TCLAP::ValueArg<_Float32>new_z_pos("Z","NewZPos","New Vertical possition of the radar",false,status.Vertical,"float");//TODO
    TCLAP::ValueArg<_Float32>new_yaw("y","NewYaw","New yaw for the radar",false,status.Yaw,"float");//TODO
    TCLAP::ValueArg<_Float32>new_pitch("P","NewPitch","New pitch for the radar",false,status.Pitch,"float");//TODO
    TCLAP::ValueArg<std::uint8_t>new_plug_otientation("p","NewPlugOr","New plug orientation for the radar(0=RIGHT,1=LEFT)",false,status.PlugOrientation,"uint8_t");//TODO
    //To configure the characteristics of the vehicle the radar is possitioned
    TCLAP::ValueArg<_Float32>new_vehicle_Length("L","NewLength","New vehicle length",false,status.Length,"float");//TODO
    TCLAP::ValueArg<_Float32>new_vehicle_Width("W","NewWidth","New vehicle width",false,status.Width,"float");//TODO
    TCLAP::ValueArg<_Float32>new_vehicle_Height("H","NewHeight","New vehicle heigth",false,status.Height,"float");//TODO
    TCLAP::ValueArg<_Float32>new_vehicle_Wheelbase("w","NewWheelLength","New vehicle wheelbase",false,status.Wheelbase,"float");//TODO
    //To configure the radar parameters
    TCLAP::ValueArg<std::uint16_t>max_dist("D","maxDist","New max distance for the radar",false,status.MaximunDistance,"uint16_t");
    TCLAP::ValueArg<std::uint8_t>new_frequency_slot("F","NewFreq","New Frequency Slot for the radar (0=Low,1=Mid,2=High)",false,status.FrequencySlot,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_cycle_time("C","Newtime","New cycle time for the radar",false,status.CycleTime,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_cycle_offset("O","NewOffset","New radar cycle offset",false,status.TimeSlot,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_country_code("c","NewCountryCode","New radar Country Code (1=Worldwide,2=Japan)",false,status.HCC,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>powersave_standstill("p","PowersaveActive","Turn on or off the powersaving in standstill(0=Off,1=On)",false,status.Powersave_Standstill,"uint8_t");
    //To configure the radar network
    TCLAP::ValueArg<std::string>new_ip0_arg("I","NewIp0","New IP0 to connect to the radar",false,NEW_IP,"string");
    //TCLAP::ValueArg<std::string>new_ip1_arg("i","NewIp1","New IP1 to connect to the radar",false,NEW_IP,"string");
    
    //Radar possition and orientation
    cmd.add(new_x_pos);
    cmd.add(new_y_pos);
    cmd.add(new_z_pos);
    cmd.add(new_yaw);
    cmd.add(new_pitch);
    cmd.add(new_plug_otientation);

    //Vehicle characteristics
    cmd.add(new_vehicle_Length);
    cmd.add(new_vehicle_Width);
    cmd.add(new_vehicle_Height);
    cmd.add(new_vehicle_Wheelbase);

    //Radar configuration
    cmd.add(max_dist);
    cmd.add(new_frequency_slot);
    cmd.add(new_cycle_time);
    cmd.add(new_cycle_offset);
    cmd.add(new_country_code);
    cmd.add(powersave_standstill);

    //Network configuration
    cmd.add(new_ip0_arg);
    //cmd.add(new_ip1_arg);
    cmd.parse(argc,argv);
        
    //fill all elements with the values obtained from the command
    //Radar possition.
    c.Longitudinal=new_x_pos.getValue();
    c.Lateral=new_y_pos.getValue();
    c.Vertical=new_z_pos.getValue();
    c.Yaw=new_yaw.getValue();
    c.Pitch=new_pitch.getValue();
    c.PlugOrientation=new_plug_otientation.getValue();

    //vehicle characteristics.
    c.Length=new_vehicle_Length.getValue();
    c.Width=new_vehicle_Width.getValue();
    c.Height=new_vehicle_Height.getValue();
    c.Wheelbase=new_vehicle_Wheelbase.getValue();

    //radar characteristics.
    c.MaximumDistance=max_dist.getValue();
    c.FrequencySlot=new_frequency_slot.getValue();
    c.CycleTime=new_cycle_time.getValue();
    c.FrequencySlot=new_cycle_offset.getValue();
    c.HCC=new_country_code.getValue();
    c.Powersave_Standstill=powersave_standstill.getValue();
    
    //Network characteristics.
    c.SensorIPAddress_0=inet_addr(new_ip0_arg.getValue().c_str());
    c.SensorIPAddress_1=status.SensorIPAddress_1;
    /**
     *  TODO.
     *  Make the program able to modify every element.
     */
    //Check if the user wants to change the radar possition and orientation
    if (c.Longitudinal!=status.Longitudinal)
        c.NewSensorMounting=1;
    if (c.Lateral!=status.Lateral)
        c.NewSensorMounting=1;
    if (c.Vertical!=status.Vertical)
        c.NewSensorMounting=1;
    if (c.Yaw!=status.Yaw)
        c.NewSensorMounting=1;
    if (c.Pitch!=status.Pitch)
        c.NewSensorMounting=1;
    if (c.PlugOrientation!=status.PlugOrientation)
        c.NewSensorMounting=1;
    
    //Check if the user wants to change the vehicle characteristics
    if (c.Length!=status.Length)
        c.NewVehicleParameters=1;
    if (c.Width!=status.Width)
        c.NewVehicleParameters=1;
    if (c.Height!=status.Height)
        c.NewVehicleParameters=1;
    if (c.Wheelbase!=status.Wheelbase)
        c.NewVehicleParameters=1;
    
    //Check if the user wants to change the radar configuration
    if(c.FrequencySlot!=status.FrequencySlot)
        c.NewRadarParameters=1;
    if (c.MaximumDistance!=status.MaximunDistance)
    {
        c.NewRadarParameters=1;
        if (c.MaximumDistance<190 && c.FrequencySlot!=1)
            c.FrequencySlot=1;
    }
    if(c.CycleTime!=status.CycleTime)
        c.NewRadarParameters=1;
    if (c.TimeSlot!=status.TimeSlot)
        c.NewRadarParameters=1;
    if (c.HCC!=status.HCC)
        c.NewRadarParameters=1;
    if(c.Powersave_Standstill!=status.Powersave_Standstill)
        c.NewRadarParameters=1;

    // Check if the user wants to change the radar IP.
    if(c.SensorIPAddress_0!=status.SensorIPAddress_0&&ip!=inet_addr(NEW_IP))
        c.NewNetworkConfiguration=1;
    
    //change endianness
    
    //fill the configuration structure to send the message to the radar.
    
    
    /**
     * TODO 
     * Connect to the radar to obtain all of the default values of the radar.
    */


 
    return 0;
}