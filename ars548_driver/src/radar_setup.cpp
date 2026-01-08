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
#include <unistd.h>
#include "tclap/CmdLine.h"
#include "ars548_driver/ars548_driver.hpp"
#include "ars548_driver/ars548_data.h"
#include <stdfloat>

// Default values, can be overridden by arguments
#define DEFAULT_RADAR_INTERFACE "10.13.1.166"
#define CONFIGURATION_SOURCE_PORT 42401
#define CONFIGURATION_DESTINATION_PORT 42101

#define DEFAULT_NEW_IP "0.0.0.0"
#define DEFAULT_RADAR_IP "10.13.1.113"


int main(int argc,char* argv[]){
    int fdr = -1, fds = -1; 
    int nbytes;
    char msgbuf[MSGBUFSIZE];
    struct SensorConfiguration c;
    
    TCLAP::CmdLine cmd("Command description message",' ',"0.9");
    
    // IP Arguments
    TCLAP::ValueArg<std::string> arg_local_ip("l", "local_ip", "Local Interface IP", false, DEFAULT_RADAR_INTERFACE, "string");
    TCLAP::ValueArg<std::string> arg_radar_ip("r", "radar_ip", "Radar IP", false, DEFAULT_RADAR_IP, "string");
    TCLAP::ValueArg<std::string> arg_multicast_ip("m", "multicast_ip", "Multicast IP", false, "224.0.2.2", "string");

    //To configure the radar position and orientation in the vehicle
    TCLAP::ValueArg<float>new_x_pos("X","NewXPos","New Longitudinal position of the radar (-100,100)",false,0.0,"float");
    TCLAP::ValueArg<float>new_y_pos("Y","NewYPos","New Lateral position of the radar (-100,100)",false,0.0,"float");
    TCLAP::ValueArg<float>new_z_pos("Z","NewZPos","New Vertical position of the radar (0.01,10)",false,0.0,"float");
    TCLAP::ValueArg<float>new_yaw("y","NewYaw","New yaw for the radar (-3.14159,3.14159)",false,0.0,"float");
    TCLAP::ValueArg<float>new_pitch("P","NewPitch","New pitch for the radar (-1.5707,1.5707)",false,0.0,"float");
    TCLAP::ValueArg<std::uint8_t>new_plug_orientation("p","NewPlugOr","New plug orientation for the radar(0=RIGHT,1=LEFT)",false,0,"uint8_t");
    //To configure the characteristics of the vehicle the radar is positioned
    TCLAP::ValueArg<float>new_vehicle_Length("L","NewLength","New vehicle length (0.01,100)",false,0.0,"float");
    TCLAP::ValueArg<float>new_vehicle_Width("W","NewWidth","New vehicle width (0.01,100)",false,0.0,"float");
    TCLAP::ValueArg<float>new_vehicle_Height("H","NewHeight","New vehicle height (0.01,100)",false,0.0,"float");
    TCLAP::ValueArg<float>new_vehicle_Wheelbase("w","NewWheelLength","New vehicle wheelbase (0.01,100)",false,0.0,"float");
    //To configure the radar parameters
    TCLAP::ValueArg<std::uint16_t>max_dist("D","maxDist","New max distance for the radar (93,1514)",false,0,"uint16_t");
    TCLAP::ValueArg<std::uint8_t>new_frequency_slot("F","NewFreq","New Frequency Slot for the radar (0=Low,1=Mid,2=High)",false,0,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_cycle_time("C","Newtime","New cycle time for the radar (50,100)",false,0,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_cycle_offset("O","NewOffset","New radar cycle offset (10,90)",false,0,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>new_country_code("c","NewCountryCode","New radar Country Code (1=Worldwide,2=Japan)",false,0,"uint8_t");
    TCLAP::ValueArg<std::uint8_t>powersave_standstill("s","PowersaveActiveStandstill","Turn on or off the powersaving in standstill(0=Off,1=On)",false,0,"uint8_t");
    //To configure the radar network
    TCLAP::ValueArg<std::string>new_ip0_arg("I","NewIp0","New IP0 to connect to the radar (Example: 169.254.116.113)",false,DEFAULT_NEW_IP,"string");
    
    // Add args
    cmd.add(arg_local_ip);
    cmd.add(arg_radar_ip);
    cmd.add(arg_multicast_ip);
    
    cmd.add(new_x_pos);
    cmd.add(new_y_pos);
    cmd.add(new_z_pos);
    cmd.add(new_yaw);
    cmd.add(new_pitch);
    cmd.add(new_plug_orientation);
    cmd.add(new_vehicle_Length);
    cmd.add(new_vehicle_Width);
    cmd.add(new_vehicle_Height);
    cmd.add(new_vehicle_Wheelbase);
    cmd.add(max_dist);
    cmd.add(new_frequency_slot);
    cmd.add(new_cycle_time);
    cmd.add(new_cycle_offset);
    cmd.add(new_country_code);
    cmd.add(powersave_standstill);
    cmd.add(new_ip0_arg);

    cmd.parse(argc,argv);
    
    std::string local_ip = arg_local_ip.getValue();
    std::string radar_ip_dest = arg_radar_ip.getValue();
    std::string multicast_ip = arg_multicast_ip.getValue();

    fdr=socket(AF_INET,SOCK_DGRAM,0);
    if (fdr<0)
    {
        perror("socket");
        return 1;
    }
    
    u_int yes=1;
    if(setsockopt(fdr,SOL_SOCKET,SO_REUSEADDR,(char*) &yes,sizeof(yes))<0)
    {
        perror("Reusing ADDR failed");
        close(fdr);
        return 1;
    }
    
    struct sockaddr_in addrR;
    memset(&addrR, 0, sizeof(addrR));
    addrR.sin_family = AF_INET;
    addrR.sin_addr.s_addr = htonl(INADDR_ANY); 
    addrR.sin_port = htons(42102); // Default ARS548 Port
    
    if (bind(fdr, (struct sockaddr*) &addrR, sizeof(addrR)) < 0) {
        perror("bind");
        close(fdr);
        return 1;
    }

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_ip.c_str());
    mreq.imr_interface.s_addr = inet_addr(local_ip.c_str());
    
    bool connected=true;
    if (setsockopt(fdr, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char*) &mreq, sizeof(mreq)) < 0)
    {
        perror("setsockopt - Join Multicast Failed");
        connected=false;
        // Don't return, just mark as not connected? Or failing here is better?
        // Let's assume user wants to know
    }

    // Set timeout for recvfrom
    struct timeval tv;
    tv.tv_sec = 2; // 2 seconds timeout
    tv.tv_usec = 0;
    setsockopt(fdr, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof tv);

    unsigned int addrlenR = sizeof(addrR);
    struct UDPStatus s;
    
    if(connected){
        // Try to get status
        printf("Waiting for status message...\n");
        bool status_received = false;
        int retries = 5;
        while(retries > 0 && !status_received) {
            nbytes = recvfrom(fdr, msgbuf, MSGBUFSIZE, 0, (struct sockaddr *) &addrR, &addrlenR);
            if(nbytes < 0) {
                 if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    printf("Timeout waiting for data. Retrying... (%d)\n", retries);
                } else {
                    perror("recvfrom failed");
                }
            } else {
                 if (s.receiveStatusMsg(nbytes, msgbuf)) {
                     status_received = true;
                     printf("Status received.\n");
                 }
            }
            retries--;
        }

        if (!status_received) {
            printf("Could not receive valid status data after retries.\n");
            close(fdr);
            return 1;
        }
    }
    
    // Fill c with current s values or defaults if s is not valid (but here we ensured s is valid-ish if status_received)
    // Actually the logic below assumes we want to CHANGE things based on args.
    
    // Logic from original file to check if values are set in args
    // Note: This logic is a bit flawed in original because ValueArg always has a value.
    // However, we can check if the value provided is different from defaults or specifically use isSet() if we switched to non-required args without defaults, but strict defaults are used here.
    // For now we preserve the logic of assigning values if 'connected'.
    
    if (connected)
    {
        c.Longitudinal = new_x_pos.isSet() ? new_x_pos.getValue() : s.Longitudinal;
        c.Lateral = new_y_pos.isSet() ? new_y_pos.getValue() : s.Lateral;
        c.Vertical = new_z_pos.isSet() ? new_z_pos.getValue() : s.Vertical;
        c.Yaw = new_yaw.isSet() ? new_yaw.getValue() : s.Yaw;
        c.Pitch = new_pitch.isSet() ? new_pitch.getValue() : s.Pitch;
        c.PlugOrientation = new_plug_orientation.isSet() ? new_plug_orientation.getValue() : s.PlugOrientation;

        c.Length = new_vehicle_Length.isSet() ? new_vehicle_Length.getValue() : s.Length;
        c.Width = new_vehicle_Width.isSet() ? new_vehicle_Width.getValue() : s.Width;
        c.Height = new_vehicle_Height.isSet() ? new_vehicle_Height.getValue() : s.Height;
        c.Wheelbase = new_vehicle_Wheelbase.isSet() ? new_vehicle_Wheelbase.getValue() : s.Wheelbase;

        c.MaximumDistance = max_dist.isSet() ? max_dist.getValue() : s.MaximumDistance;
        c.FrequencySlot = new_frequency_slot.isSet() ? new_frequency_slot.getValue() : s.FrequencySlot;
        c.CycleTime = new_cycle_time.isSet() ? new_cycle_time.getValue() : s.CycleTime;
        c.TimeSlot = new_cycle_offset.isSet() ? new_cycle_offset.getValue() : s.TimeSlot;
        c.HCC = new_country_code.isSet() ? new_country_code.getValue() : s.HCC;
        c.Powersave_Standstill = powersave_standstill.isSet() ? powersave_standstill.getValue() : s.Powersave_Standstill;
        
        c.SensorIPAddress_0 = s.SensorIPAddress_0; // Not changing unless arg provided? 
        // Logic for IP change wasn't clearly implemented in original for IP args properly mapping to struct, let's assume no change unless...
        // Original: c.SensorIPAddress_0=s.SensorIPAddress_0;
        
        // Check changeConfiguration
        c.NewSensorMounting=0;
        c.NewVehicleParameters=0;
        c.NewRadarParameters=0;
        c.NewNetworkConfiguration=0;

        if (c.changeConfiguration(s)) 
        {   
            if (c.NewSensorMounting==1) std::cout<<"Changing the sensor mounting Position \n";
            if (c.NewVehicleParameters==1) std::cout<<"Changing the vehicle parameters\n";
            if (c.NewRadarParameters==1) std::cout<<"Changing the Radar parameters\n";
            if (c.NewNetworkConfiguration==1) std::cout<<"Changing the Network Configuration\n";
            
            c.setIDsAndPayload();
            c.changeEndianness();
            
            // Send
            fds = socket(AF_INET, SOCK_DGRAM, 0);
            if (fds < 0) {
                perror("socket");
                close(fdr);
                return 1;
            }
            struct sockaddr_in addrS;
            std::memset(&addrS, 0, sizeof(addrS));
            addrS.sin_family = AF_INET;
            addrS.sin_port = htons(CONFIGURATION_DESTINATION_PORT);
            int addrlenS=sizeof(addrS);
            inet_pton(AF_INET, radar_ip_dest.c_str(), &addrS.sin_addr);
            
            int sent_bytes = sendto(fds, &c, sizeof(c), 0, (struct sockaddr *) &addrS, addrlenS);
            
            c.changeEndianness(); // Swap back if needed or just for logging
            
            if (sent_bytes < 0) {
                    perror("Failed sending the message");
                    close(fds);
                    close(fdr);
                    return 1;
            }
            close(fds);
        }
        
        // Wait for update confirmation
        printf("Waiting for status update confirmation...\n");
        int wait_cycles = 0;
        bool confirmed = false;
        while(wait_cycles < 30 && !confirmed) // Wait up to ~60s ? Original was infinite-ish with sleep(1)
        {
            // We use the receive timeout from before
            nbytes = recvfrom(fdr, msgbuf, MSGBUFSIZE, 0, (struct sockaddr *) &addrR, &addrlenR);
            if(nbytes > 0)
            {
               if (s.receiveStatusMsg(nbytes, msgbuf))
               {
                   if(c.isEqualToStatus(s))
                    {
                        confirmed = true;
                    }
               }
            }
            else
            {
                 // Timeout or error
                 if (errno != EAGAIN && errno != EWOULDBLOCK) {
                     perror("recvfrom error in confirmation loop");
                 }
            }
            wait_cycles++;
        }

        std::cout <<"\n\n -------------- Desired configuration -------------- \n\n";
        c.print();
        std::cout <<" -------------- Received status updated ------------ \n\n";
        s.print();
        
        if (confirmed)
        {
            std::cout <<"\n\n Configured OK!!!!!!!!\n\n";
            close(fdr);
            return 0;
        }
        else
        {
            std::cout <<"\n\n Failed to configure the system (or timeout) \n\n";
            close(fdr);
            return 1;
        }
    }  
    close(fdr);
    return 0;
}