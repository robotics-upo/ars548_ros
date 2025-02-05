#pragma once
#include "ars548_driver/util/byteswap.hpp"
#include "ars548_driver/util/rough_equal.hpp"
#include "udp_status.h"

#define CONFIGURATION_SERVICE_ID 0
#define CONFIGURATION_METHOD_ID 390
#define CONFIGURATION_MESSAGE_ID 390
#define CONFIGURATION_PDU_LENGTH 56
#define CONFIGURATION_UDP_PAYLOAD 64
#define CONFIGURATION_UDP_LENGTH 72
#define ARS548_MINIMUM_DISTANCE_SLOT_1 190
#define CONFIGURATION_PRECISION 0.001f

#pragma pack(1)

#ifndef NEW_IP
#define NEW_IP "0.0.0.0"
#endif

struct SensorConfiguration
{
    uint16_t ServiceID;
    uint16_t MethodID;
    uint32_t PayloadLength;
    float Longitudinal;
    float Lateral;
    float Vertical;
    float Yaw;
    float Pitch;
    uint8_t PlugOrientation;
    float Length;
    float Width;
    float Height;
    float Wheelbase;
    uint16_t MaximumDistance;
    uint8_t FrequencySlot;
    uint8_t CycleTime;
    uint8_t TimeSlot;
    uint8_t HCC;
    uint8_t Powersave_Standstill;
    uint32_t SensorIPAddress_0;
    uint32_t SensorIPAddress_1;
    uint8_t NewSensorMounting;
    uint8_t NewVehicleParameters;
    uint8_t NewRadarParameters;
    uint8_t NewNetworkConfiguration;

    inline void changeEndianness();

    inline bool isEqualToStatus(const UDPStatus &s) const;
    inline void print() const;

    inline bool changeConfiguration(const UDPStatus &s);

    inline void setIDsAndPayload() {
        ServiceID = CONFIGURATION_SERVICE_ID;
        MethodID = CONFIGURATION_METHOD_ID;
        PayloadLength = CONFIGURATION_PDU_LENGTH;
    }
};
#pragma pack(4)

// @brief Changes the endianness of the data structure (uint8_t fields don't need to)
inline void SensorConfiguration::changeEndianness() {
    ServiceID = byteswap(ServiceID);
    MethodID = byteswap(MethodID);
    PayloadLength = byteswap(PayloadLength);
    Longitudinal = byteswap(Longitudinal);
    Lateral = byteswap(Lateral);
    Vertical = byteswap(Vertical);
    Yaw = byteswap(Yaw);
    Pitch = byteswap(Pitch);
// PlugOrientation = byteswap(PlugOrientation);
    Length = byteswap(Length);
    Width = byteswap(Width);
    Height = byteswap(Height);
    Wheelbase = byteswap(Wheelbase);
    MaximumDistance = byteswap(MaximumDistance);
    // FrequencySlot = byteswap(FrequencySlot);
    // CycleTime = byteswap(CycleTime);
    // TimeSlot = byteswap(TimeSlot);
    // HCC = byteswap(HCC);
    // Powersave_Standstill = byteswap(Powersave_Standstill);
    SensorIPAddress_0 = byteswap(SensorIPAddress_0);
    SensorIPAddress_1 = byteswap(SensorIPAddress_1);
    // NewSensorMounting = byteswap(NewSensorMounting);
    // NewVehicleParameters = byteswap(NewVehicleParameters);
    // NewRadarParameters = byteswap(NewRadarParameters);
    // NewNetworkConfiguration = byteswap(NewNetworkConfiguration);
}

inline bool SensorConfiguration::isEqualToStatus(const UDPStatus &s) const {
    bool isEqual = true;
    if (!rough_eq(Longitudinal,s.Longitudinal, CONFIGURATION_PRECISION))
        isEqual = false;
    if (!rough_eq(Lateral,s.Lateral, CONFIGURATION_PRECISION))
        isEqual = false;
    if (!rough_eq(Vertical, s.Vertical, CONFIGURATION_PRECISION))
        isEqual = false;
    if (!rough_eq(Yaw, s.Yaw, CONFIGURATION_PRECISION))
        isEqual = false;
    if (!rough_eq(Pitch, s.Pitch, CONFIGURATION_PRECISION))
        isEqual = false;
    if (PlugOrientation != s.PlugOrientation)
        isEqual = false;
    if (!rough_eq(Length, s.Length, CONFIGURATION_PRECISION))
        isEqual = false;
    if (!rough_eq(Width, s.Width, CONFIGURATION_PRECISION))
        isEqual = false;
    if (!rough_eq(Height, s.Height))
        isEqual = false;
    if (!rough_eq(Wheelbase, s.Wheelbase, CONFIGURATION_PRECISION))
        isEqual = false;
    if(FrequencySlot != s.FrequencySlot)
        isEqual = false;
    if (MaximumDistance != s.MaximumDistance)
        isEqual = false;
    if(CycleTime != s.CycleTime)
        isEqual = false;
    if (TimeSlot != s.TimeSlot)
        isEqual = false;
    if (HCC != s.HCC)
        isEqual = false;
    if(Powersave_Standstill != s.Powersave_Standstill)
        isEqual = false;
    if(SensorIPAddress_0 != s.SensorIPAddress_0 && 
       SensorIPAddress_0 != inet_addr(NEW_IP) && 
       s.SensorIPAddress_0 != inet_addr(NEW_IP))
        isEqual = false;
    return isEqual;
}

inline void SensorConfiguration::print() const
{
    std::cout<<"Config: \n";
    std::cout<<"Longitudinal Pos: "<<Longitudinal<<"\n";
    std::cout<<"Lateral Pos: "<<Lateral<<"\n";
    std::cout<<"Vertical Pos: "<<Vertical<<"\n";
    std::cout<<"Yaw: "<<Yaw<<"\n";
    std::cout<<"Pitch: "<<Pitch<<"\n";
    if(PlugOrientation == 1)
    {
        std::cout<<"Plug Orientation: LEFT\n";
    }else
    {
        std::cout<<"Plug Orientation: RIGHT\n";
    }
    std::cout<<"Vehicle Length: "<<Length<<"\n";
    std::cout<<"Vehicle Width: "<<Width<<"\n";
    std::cout<<"Vehicle Height: "<<Height<<"\n";
    std::cout<<"Vehicle WheelBase: "<<Wheelbase<<"\n";
    std::cout<<"Max Detection Dist: "<<MaximumDistance<<"\n";
    switch (FrequencySlot)
    {
    case 0:
        std::cout<<"Center Frequency: LOW\n";
        break;
    case 1:
        std::cout<<"Center Frequency: MID\n";
        break;
    default:
        std::cout<<"Center Frequency: HIGH\n";
        break;
    }
    std::cout<<"Cycle Time: "<<CycleTime<<"\n";
    std::cout<<"Cycle Offset: "<<(int)TimeSlot<<"\n";
    if(HCC == 1)
    {
        std::cout<<"Country Code: WORLDWIDE\n";
    }else
    {
        std::cout<<"Country Code: JAPAN\n";
    }
    if (Powersave_Standstill == 1)
    {
        std::cout<<"Powersave Standstill: ON\n";
    }else
    {
        std::cout<<"Powersave Standstill: OFF\n";
    }
    std::cout<<"Sensor IP Address 0: "<<SensorIPAddress_0<<"\n";
    std::cout<<"Sensor IP Address 1: "<<SensorIPAddress_1<<"\n";

    std::cout << std::endl; // Flush only at the end of the message
}


/**
 * @brief Checks if any of the data has been modified by the anchor. And modifies the proper fields
 * @param c The Sensor Configuration struct with the data sent by the user.
 * @param s The UDP Status message used to get all of the default data of the radar.
 * @return true If there are changes to update in the configuration (false otherwise)
 */
inline bool SensorConfiguration::changeConfiguration(const UDPStatus &s)
{
    //Check if the user wants to change the radar possition and orientation
    if (!rough_eq(Longitudinal,s.Longitudinal, CONFIGURATION_PRECISION))
        NewSensorMounting = 1;
    if (!rough_eq(Lateral,s.Lateral, CONFIGURATION_PRECISION))
        NewSensorMounting = 1;
    if (!rough_eq(Vertical,s.Vertical, CONFIGURATION_PRECISION))
        NewSensorMounting = 1;
    if (!rough_eq(Yaw,s.Yaw, CONFIGURATION_PRECISION))
        NewSensorMounting = 1;
    if (!rough_eq(Pitch,s.Pitch, CONFIGURATION_PRECISION))
        NewSensorMounting = 1;
    if (PlugOrientation != s.PlugOrientation)
        NewSensorMounting = 1;
    
    //Check if the user wants to change the vehicle characteristics
    if (!rough_eq(Length,s.Length, CONFIGURATION_PRECISION))
        NewVehicleParameters = 1;
    if (!rough_eq(Width,s.Width, CONFIGURATION_PRECISION))
        NewVehicleParameters = 1;
    if (!rough_eq(Height,s.Height, CONFIGURATION_PRECISION))
        NewVehicleParameters = 1;
    if (!rough_eq(Wheelbase,s.Wheelbase, CONFIGURATION_PRECISION))
        NewVehicleParameters = 1;
    
    //Check if the user wants to change the radar configuration
    if(FrequencySlot != s.FrequencySlot)
        NewRadarParameters = 1;
    if (MaximumDistance != s.MaximumDistance)
    {
        NewRadarParameters = 1;
        if (MaximumDistance < ARS548_MINIMUM_DISTANCE_SLOT_1  && FrequencySlot != 1)
            FrequencySlot = 1;
    }
    if(CycleTime != s.CycleTime)
        NewRadarParameters = 1;
    if (TimeSlot != s.TimeSlot)
        NewRadarParameters = 1;
    if (HCC != s.HCC)
        NewRadarParameters = 1;
    if(Powersave_Standstill != s.Powersave_Standstill)
        NewRadarParameters = 1;

    // Check if the user wants to change the radar IP.
    if(SensorIPAddress_0 != s.SensorIPAddress_0 && SensorIPAddress_0 != inet_addr(NEW_IP))
        NewNetworkConfiguration = 1;

    return NewSensorMounting == 1 || NewVehicleParameters == 1 || 
           NewRadarParameters == 1 || NewNetworkConfiguration == 1;
}



