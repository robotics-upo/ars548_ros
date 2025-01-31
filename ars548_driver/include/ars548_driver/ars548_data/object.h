#pragma once
#include <cstdint>
#include <ars548_driver/byteswap.hpp>

struct Object{
    uint16_t u_StatusSensor;
    uint32_t u_ID;
    uint16_t u_Age;
    uint8_t u_StatusMeasurement;
    uint8_t u_StatusMovement;
    uint16_t u_Position_InvalidFlags;
    uint8_t u_Position_Reference;
    float u_Position_X;
    float u_Position_X_STD;
    float u_Position_Y;
    float u_Position_Y_STD;
    float u_Position_Z;
    float u_Position_Z_STD;
    float u_Position_CovarianceXY;
    float u_Position_Orientation;
    float u_Position_Orientation_STD;
    uint8_t u_Existence_InvalidFlags;
    float u_Existence_Probability;
    float u_Existence_PPV;
    uint8_t u_Classification_Car;
    uint8_t u_Classification_Truck;
    uint8_t u_Classification_Motorcycle;
    uint8_t u_Classification_Bicycle;
    uint8_t u_Classification_Pedestrian;
    uint8_t u_Classification_Animal;
    uint8_t u_Classification_Hazard;
    uint8_t u_Classification_Unknown;
    uint8_t u_Classification_Overdrivable;
    uint8_t u_Classification_Underdrivable;
    uint8_t u_Dynamics_AbsVel_InvalidFlags;
    float f_Dynamics_AbsVel_X;
    float f_Dynamics_AbsVel_X_STD;
    float f_Dynamics_AbsVel_Y;
    float f_Dynamics_AbsVel_Y_STD;
    float f_Dynamics_AbsVel_CovarianceXY;
    uint8_t u_Dynamics_RelVel_InvalidFlags;
    float f_Dynamics_RelVel_X;
    float f_Dynamics_RelVel_X_STD;
    float f_Dynamics_RelVel_Y;
    float f_Dynamics_RelVel_Y_STD;
    float f_Dynamics_RelVel_CovarianceXY;
    uint8_t u_Dynamics_AbsAccel_InvalidFlags;
    float f_Dynamics_AbsAccel_X;
    float f_Dynamics_AbsAccel_X_STD;
    float f_Dynamics_AbsAccel_Y;
    float f_Dynamics_AbsAccel_Y_STD;
    float f_Dynamics_AbsAccel_CovarianceXY;
    uint8_t u_Dynamics_RelAccel_InvalidFlags;
    float f_Dynamics_RelAccel_X;
    float f_Dynamics_RelAccel_X_STD;
    float f_Dynamics_RelAccel_Y;
    float f_Dynamics_RelAccel_Y_STD;
    float f_Dynamics_RelAccel_CovarianceXY;
    uint8_t u_Dynamics_Orientation_InvalidFlags;
    float u_Dynamics_Orientation_Rate_Mean;
    float u_Dynamics_Orientation_Rate_STD;
    uint32_t u_Shape_Length_Status;
    uint8_t u_Shape_Length_Edge_InvalidFlags;
    float u_Shape_Length_Edge_Mean;
    float u_Shape_Length_Edge_STD;
    uint32_t u_Shape_Width_Status;
    uint8_t u_Shape_Width_Edge_InvalidFlags;
    float u_Shape_Width_Edge_Mean;
    float u_Shape_Width_Edge_STD;

    inline void changeEndianness();
};

void Object::changeEndianness() {
    u_StatusSensor = byteswap(u_StatusSensor);
    u_ID = byteswap(u_ID);
    u_Age = byteswap(u_Age);
    u_Position_InvalidFlags = byteswap(u_Position_InvalidFlags);
    u_Position_X = byteswap(u_Position_X);
    u_Position_X_STD = byteswap(u_Position_X_STD);
    u_Position_Y = byteswap(u_Position_Y);
    u_Position_Y_STD = byteswap(u_Position_Y_STD);
    u_Position_Z = byteswap(u_Position_Z);
    u_Position_Z_STD = byteswap(u_Position_Z_STD);
    u_Position_CovarianceXY = byteswap(u_Position_CovarianceXY);
    u_Position_Orientation = byteswap(u_Position_Orientation);
    u_Position_Orientation_STD = byteswap(u_Position_Orientation_STD);
    u_Existence_Probability = byteswap(u_Existence_Probability);
    u_Existence_PPV = byteswap(u_Existence_PPV);
    f_Dynamics_AbsVel_X = byteswap(f_Dynamics_AbsVel_X);
    f_Dynamics_AbsVel_X_STD = byteswap(f_Dynamics_AbsVel_X_STD);
    f_Dynamics_AbsVel_Y = byteswap(f_Dynamics_AbsVel_Y);
    f_Dynamics_AbsVel_Y_STD = byteswap(f_Dynamics_AbsVel_Y_STD);
    f_Dynamics_AbsVel_CovarianceXY = byteswap(f_Dynamics_AbsVel_CovarianceXY);
    f_Dynamics_RelVel_X = byteswap(f_Dynamics_RelVel_X);
    f_Dynamics_RelVel_X_STD = byteswap(f_Dynamics_RelVel_X_STD);
    f_Dynamics_RelVel_Y = byteswap(f_Dynamics_RelVel_Y);
    f_Dynamics_RelVel_Y_STD = byteswap(f_Dynamics_RelVel_Y_STD);
    f_Dynamics_RelVel_CovarianceXY = byteswap(f_Dynamics_RelVel_CovarianceXY);
    f_Dynamics_AbsAccel_X = byteswap(f_Dynamics_AbsAccel_X);
    f_Dynamics_AbsAccel_X_STD = byteswap(f_Dynamics_AbsAccel_X_STD);
    f_Dynamics_AbsAccel_Y = byteswap(f_Dynamics_AbsAccel_Y);
    f_Dynamics_AbsAccel_Y_STD = byteswap(f_Dynamics_AbsAccel_Y_STD);
    f_Dynamics_AbsAccel_CovarianceXY = byteswap(f_Dynamics_AbsAccel_CovarianceXY);
    f_Dynamics_RelAccel_X = byteswap(f_Dynamics_RelAccel_Y_STD);
    f_Dynamics_RelAccel_CovarianceXY = byteswap(f_Dynamics_RelAccel_CovarianceXY);
    u_Dynamics_Orientation_Rate_Mean = byteswap(u_Dynamics_Orientation_Rate_Mean);
    u_Dynamics_Orientation_Rate_STD = byteswap(u_Dynamics_Orientation_Rate_STD);
    u_Shape_Length_Status = byteswap(u_Shape_Length_Status);
    u_Shape_Length_Edge_Mean = byteswap(u_Shape_Length_Edge_Mean);
    u_Shape_Length_Edge_STD = byteswap(u_Shape_Length_Edge_STD);
    u_Shape_Width_Status = byteswap(u_Shape_Width_Status);
    u_Shape_Width_Edge_Mean = byteswap(u_Shape_Width_Edge_Mean);
    u_Shape_Width_Edge_STD = byteswap(u_Shape_Width_Edge_STD);
}


    