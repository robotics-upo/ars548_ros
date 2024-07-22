/**
 * @file radar_setup.cpp
 * @brief In this file we try to change the radar Parameters using the according message. (Be carefull when changing the IP)
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
#include "ars548_driver/ars548_driver.hpp"
#include "ars548_driver/ars548_data.h"

#define RADAR_INTERFACE "10.13.1.166"
#define CONFIGURATION_SOURCE_PORT 42401
#define CONFIGURATION_DESTINATION_PORT 42101
#define CONFIGURATION_METHOD_ID 390
#define CONFIGURATION_MESSAGE_ID 390
#define CONFIGURATION_PDU_LENGTH 56
#define CONFIGURATION_UDP_PAYLOAD 64
#define CONFIGURATION_UDP_LENGTH 72
#define NEW_IP "0.0.0.0"
#define RADAR_IP "10.13.1.113"


static bool isConfigEqualsToStatus(SensorConfiguration c,UDPStatus s){
    bool isEqual=true;
    if (c.Longitudinal!=s.Longitudinal)
        isEqual=false;
    if (c.Lateral!=s.Lateral)
        isEqual=false;
    if (c.Vertical!=s.Vertical)
        isEqual=false;
    if (c.Yaw!=s.Yaw)
        isEqual=false;
    if (c.Pitch!=s.Pitch)
        isEqual=false;
    if (c.PlugOrientation!=s.PlugOrientation)
        isEqual=false;
    if (c.Length!=s.Length)
        isEqual=false;
    if (c.Width!=s.Width)
        isEqual=false;
    if (c.Height!=s.Height)
        isEqual=false;
    if (c.Wheelbase!=s.Wheelbase)
        isEqual=false;
    if(c.FrequencySlot!=s.FrequencySlot)
        isEqual=false;
    if (c.MaximumDistance!=s.MaximunDistance)
        isEqual=false;
    if(c.CycleTime!=s.CycleTime)
        isEqual=false;
    if (c.TimeSlot!=s.TimeSlot)
        isEqual=false;
    if (c.HCC!=s.HCC)
        isEqual=false;
    if(c.Powersave_Standstill!=s.Powersave_Standstill)
        isEqual=false;
    if(c.SensorIPAddress_0!=s.SensorIPAddress_0 && c.SensorIPAddress_0!=inet_addr(NEW_IP)&& c.SensorIPAddress_0!=0)
        isEqual=false;
    return isEqual;
}

static void printConfig(SensorConfiguration c)
{
    std::cout<<"Config: "<<std::endl;
    std::cout<<"Longitudinal Pos: "<<c.Longitudinal<<std::endl;
    std::cout<<"Lateral Pos: "<<c.Lateral<<std::endl;
    std::cout<<"Vertical Pos: "<<c.Vertical<<std::endl;
    std::cout<<"Yaw: "<<c.Yaw<<std::endl;
    std::cout<<"Pitch: "<<c.Pitch<<std::endl;
    if(c.PlugOrientation==1)
    {
        std::cout<<"Plug Orientation: LEFT"<<std::endl;
    }else
    {
        std::cout<<"Plug Orientation: RIGHT"<<std::endl;
    }
    std::cout<<"Vehicle Length: "<<c.Length<<std::endl;
    std::cout<<"Vehicle Width: "<<c.Width<<std::endl;
    std::cout<<"Vehicle Height: "<<c.Height<<std::endl;
    std::cout<<"Vehicle WheelBase: "<<c.Wheelbase<<std::endl;
    std::cout<<"Max Detection Dist: "<<c.MaximumDistance<<std::endl;
    switch (c.FrequencySlot)
    {
    case 0:
        std::cout<<"Center Frequency: LOW"<<std::endl;
        break;
    case 1:
        std::cout<<"Center Frequency: MID"<<std::endl;
        break;
    default:
        std::cout<<"Center Frequency: HIGH"<<std::endl;
        break;
    }
    std::cout<<"Cycle Time: "<<c.CycleTime<<std::endl;
    std::cout<<"Cycle Offset: "<<(int)c.TimeSlot<<std::endl;
    if(c.HCC==1)
    {
        std::cout<<"Country Code: WORLDWIDE"<<std::endl;
    }else
    {
        std::cout<<"Country Code: JAPAN"<<std::endl;
    }
    if (c.Powersave_Standstill==1)
    {
        std::cout<<"Powersave Standstill: ON"<<std::endl;
    }else
    {
        std::cout<<"Powersave Standstill: OFF"<<std::endl;
    }
    std::cout<<"Sensor IP Address 0: "<<c.SensorIPAddress_0<<std::endl;
    std::cout<<"Sensor IP Address 1: "<<c.SensorIPAddress_1<<std::endl;
    
}
static void printStatus(UDPStatus s)
{
    std::cout<<"Status: "<<std::endl;
    std::cout<<"Longitudinal Pos: "<<s.Longitudinal<<std::endl;
    std::cout<<"Lateral Pos: "<<s.Lateral<<std::endl;
    std::cout<<"Vertical Pos: "<<s.Vertical<<std::endl;
    std::cout<<"Yaw: "<<s.Yaw<<std::endl;
    std::cout<<"Pitch: "<<s.Pitch<<std::endl;
    if(s.PlugOrientation==1)
    {
        std::cout<<"Plug Orientation: LEFT"<<std::endl;
    }else
    {
        std::cout<<"Plug Orientation: RIGHT"<<std::endl;
    }
    std::cout<<"Vehivle Length: "<<s.Length<<std::endl;
    std::cout<<"Vehicle Width: "<<s.Width<<std::endl;
    std::cout<<"Vehicle Height: "<<s.Height<<std::endl;
    std::cout<<"Vehicle WheelBase: "<<s.Wheelbase<<std::endl;
    std::cout<<"Max Detection Dist: "<<s.MaximunDistance<<std::endl;
    switch (s.FrequencySlot)
    {
    case 0:
        std::cout<<"Center Frequency: LOW"<<std::endl;
        break;
    case 1:
        std::cout<<"Center Frequency: MID"<<std::endl;
        break;
    default:
        std::cout<<"Center Frequency: HIGH"<<std::endl;
        break;
    }
    std::cout<<"Cycle Time: "<<s.CycleTime<<std::endl;
    std::cout<<"Cycle Offset: "<<(int)s.TimeSlot<<std::endl;
    if(s.HCC==1)
    {
        std::cout<<"Country Code: WORLDWIDE"<<std::endl;
    }else
    {
        std::cout<<"Country Code: JAPAN"<<std::endl;
    }
    if (s.Powersave_Standstill==1)
    {
        std::cout<<"Powersave Standstill: ON"<<std::endl;
    }else
    {
        std::cout<<"Powersave Standstill: OFF"<<std::endl;
    }
    std::cout<<"Sensor IP Address 0: "<<s.SensorIPAddress_0<<std::endl;
    std::cout<<"Sensor IP Address 1: "<<s.SensorIPAddress_1<<std::endl;
    
}

/**
 * @brief Checks if any of the data has been modified by the anchor.
 * @param c The Sensor Configuration struct with the data sent by the user.
 * @param s The UDP Status message used to get all of the default data of the radar.
 */
static SensorConfiguration changeConfiguration(SensorConfiguration c,UDPStatus s)
{
        //Check if the user wants to change the radar possition and orientation
    if (c.Longitudinal!=s.Longitudinal)
        c.NewSensorMounting=1;
    if (c.Lateral!=s.Lateral)
        c.NewSensorMounting=1;
    if (c.Vertical!=s.Vertical)
        c.NewSensorMounting=1;
    if (c.Yaw!=s.Yaw)
        c.NewSensorMounting=1;
    if (c.Pitch!=s.Pitch)
        c.NewSensorMounting=1;
    if (c.PlugOrientation!=s.PlugOrientation)
        c.NewSensorMounting=1;
    
    //Check if the user wants to change the vehicle characteristics
    if (c.Length!=s.Length)
        c.NewVehicleParameters=1;
    if (c.Width!=s.Width)
        c.NewVehicleParameters=1;
    if (c.Height!=s.Height)
        c.NewVehicleParameters=1;
    if (c.Wheelbase!=s.Wheelbase)
        c.NewVehicleParameters=1;
    
    //Check if the user wants to change the radar configuration
    if(c.FrequencySlot!=s.FrequencySlot)
        c.NewRadarParameters=1;
    if (c.MaximumDistance!=s.MaximunDistance)
    {
        c.NewRadarParameters=1;
        if (c.MaximumDistance<190 && c.FrequencySlot!=1)
            c.FrequencySlot=1;
    }
    if(c.CycleTime!=s.CycleTime)
        c.NewRadarParameters=1;
    if (c.TimeSlot!=s.TimeSlot)
        c.NewRadarParameters=1;
    if (c.HCC!=s.HCC)
        c.NewRadarParameters=1;
    if(c.Powersave_Standstill!=s.Powersave_Standstill)
        c.NewRadarParameters=1;

    // Check if the user wants to change the radar IP.
    if(c.SensorIPAddress_0!=s.SensorIPAddress_0 && c.SensorIPAddress_0!=inet_addr(NEW_IP))
        c.NewNetworkConfiguration=1;
    return c;
}

/**
 * @brief Changes the endianness of each field of the sensor configuration struct.
 * @param c The Sensor Configuration structure.
 */
static SensorConfiguration modifyConfiguration(SensorConfiguration c)
{
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
    c.TimeSlot=ars548_driver::ChangeEndianness(c.TimeSlot);
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

int main(int argc,char* argv[]){
    int fdr,fds;//One to receive the data of the radar, the other to send the data to it
    int nbytes;
    char msgbuf[MSGBUFSIZE];
    struct SensorConfiguration c;
    
    TCLAP::CmdLine cmd("Command description message",' ',"0.9");
    
    fdr=socket(AF_INET,SOCK_DGRAM,0);
    if (fdr<0)
    {
        perror("socket");
        return 1;
    }
    struct sockaddr_in addrR;//The address to receive from the radar
    u_int yes=1;
    if(
        setsockopt(
            fdr,SOL_SOCKET,SO_REUSEADDR,(char*) &yes,sizeof(yes)
        )<0
    )
    {
        perror("Reusing ADDR failed");
        return 1;
    }
    // set up destination address
    //
    memset(&addrR, 0, sizeof(addrR));
    addrR.sin_family = AF_INET;
    addrR.sin_addr.s_addr = htonl(INADDR_ANY); // differs from sender
    addrR.sin_port = htons(ars548_driver::ars548_Port);
    // bind to receive address
    //
    if (bind(fdr, (struct sockaddr*) &addrR, sizeof(addrR)) < 0) {
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
            fdr, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)
        ) < 0
    ){
        perror("setsockopt");
        return 1;
    }
    unsigned int addrlenR = sizeof(addrR);
    nbytes = recvfrom(
        fdr,
        msgbuf,
        MSGBUFSIZE,
        0,
        (struct sockaddr *) &addrR,
        &addrlenR
        );
    if(nbytes<0)
    {
        perror("Failed attempt of getting data");
        return 1;
    }
    struct UDPStatus s;
    s = *((struct UDPStatus *)msgbuf);
    int iterator=0;
    while (!ars548_driver::receiveStatusMsg(nbytes,msgbuf,s))
    {
        
        nbytes = recvfrom(
        fdr,
        msgbuf,
        MSGBUFSIZE,
        0,
        (struct sockaddr *) &addrR,
        &addrlenR
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
        s = *((struct UDPStatus*)msgbuf);
        iterator++;
    }
    //To configure the radar possition and orientation in the vehicle
    TCLAP::ValueArg<_Float32>new_x_pos("X","NewXPos","New Longitudinal possition of the radar",false,s.Longitudinal,"float");
    TCLAP::ValueArg<_Float32>new_y_pos("Y","NewYPos","New Lateral possition of the radar",false,s.Lateral,"float");
    TCLAP::ValueArg<_Float32>new_z_pos("Z","NewZPos","New Vertical possition of the radar",false,s.Vertical,"float");
    TCLAP::ValueArg<_Float32>new_yaw("y","NewYaw","New yaw for the radar",false,s.Yaw,"float");
    TCLAP::ValueArg<_Float32>new_pitch("P","NewPitch","New pitch for the radar",false,s.Pitch,"float");
    TCLAP::ValueArg<std::uint8_t>new_plug_otientation("p","NewPlugOr","New plug orientation for the radar(0=RIGHT,1=LEFT)",false,s.PlugOrientation,"uint8_t");
    //To configure the characteristics of the vehicle the radar is possitioned
    TCLAP::ValueArg<_Float32>new_vehicle_Length("L","NewLength","New vehicle length",false,s.Length,"float");
    TCLAP::ValueArg<_Float32>new_vehicle_Width("W","NewWidth","New vehicle width",false,s.Width,"float");
    TCLAP::ValueArg<_Float32>new_vehicle_Height("H","NewHeight","New vehicle heigth",false,s.Height,"float");
    TCLAP::ValueArg<_Float32>new_vehicle_Wheelbase("w","NewWheelLength","New vehicle wheelbase",false,s.Wheelbase,"float");
    //To configure the radar parameters
    TCLAP::ValueArg<std::uint16_t>max_dist("D","maxDist","New max distance for the radar",false,s.MaximunDistance,"uint16_t");
    TCLAP::ValueArg<std::uint8_t>new_frequency_slot("F","NewFreq","New Frequency Slot for the radar (0=Low,1=Mid,2=High)",false,s.FrequencySlot,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_cycle_time("C","Newtime","New cycle time for the radar",false,s.CycleTime,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_cycle_offset("O","NewOffset","New radar cycle offset",false,s.TimeSlot,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_country_code("c","NewCountryCode","New radar Country Code (1=Worldwide,2=Japan)",false,s.HCC,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>powersave_standstill("s","PowersaveActiveStandstill","Turn on or off the powersaving in standstill(0=Off,1=On)",false,s.Powersave_Standstill,"uint8_t");
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
    std::cout<<c.MaximumDistance<<std::endl;
    c.FrequencySlot=new_frequency_slot.getValue();
    c.CycleTime=new_cycle_time.getValue();
    c.TimeSlot=new_cycle_offset.getValue();
    c.HCC=new_country_code.getValue();
    c.Powersave_Standstill=powersave_standstill.getValue();
    
    //Network characteristics.
    c.SensorIPAddress_0=s.SensorIPAddress_0;
    c.SensorIPAddress_1=s.SensorIPAddress_1;

    //Modify characteristics
    c.NewSensorMounting=0;
    c.NewVehicleParameters=0;
    c.NewRadarParameters=0;
    c.NewNetworkConfiguration=0;

    c=changeConfiguration(c,s);
    //Check if the user wants to modify any of the radar parameters
    if(c.NewSensorMounting==1||c.NewVehicleParameters==1||c.NewRadarParameters==1||c.NewNetworkConfiguration==1)
    {   
        if (c.NewSensorMounting==1)
        {
            std::cout<<"Changing the sensor mounting Possition"<<std::endl;
        }
        if (c.NewVehicleParameters==1)
        {
            std::cout<<"Changing the vehicle parameters"<<std::endl;
        }
        if (c.NewRadarParameters==1)
        {
            std::cout<<"Changing the Radar parameters"<<std::endl;
        }
        if (c.NewNetworkConfiguration==1)
        {
            std::cout<<"Changing the Network Configuration"<<std::endl;
        }
        //change endianness
        c.ServiceID=0;
        c.MethodID=390;
        c.PayloadLength=56;
        SensorConfiguration sc;
        sc=modifyConfiguration(c);
        //send the message to the radar
        fds = socket(AF_INET, SOCK_DGRAM, 0);
        if (fds < 0) {
            perror("socket");
            return 1;
        }
        struct sockaddr_in addrS;
        std::memset(&addrS, 0, sizeof(addrS));
        addrS.sin_family = AF_INET;
        addrS.sin_port = htons(CONFIGURATION_DESTINATION_PORT);
        int addrlenS=sizeof(addrS);
        inet_pton(AF_INET, RADAR_IP, &addrS.sin_addr);
        int sent_bytes=sendto(fds,&sc,sizeof(sc),0, (struct sockaddr *) &addrS,addrlenS);
        if (sent_bytes < 0) {
                perror("Failed sending the message");
                return 1;
        }
        close(fds);
    }
    while (!ars548_driver::receiveStatusMsg(nbytes,msgbuf,s)){
        nbytes = recvfrom(
        fdr,
        msgbuf,
        MSGBUFSIZE,
        0,
        (struct sockaddr *) &addrR,
        &addrlenR
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
    printStatus(s);
    printConfig(c);
    if(isConfigEqualsToStatus(c,s))
    {
        return 0;
    }else
    {
        return 1;
    } 
}