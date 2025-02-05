#pragma once
#include <cstdint>
#include <ars548_driver/util/byteswap.hpp>

#pragma pack(1)

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
    ars548_messages::msg::Object toMsg();
};

#pragma pack(4)

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

ars548_messages::msg::Object Object::toMsg() {
    ars548_messages::msg::Object o;

    o.u_statussensor = u_StatusSensor;
    o.u_id = u_ID;
    o.u_age = u_Age;
    o.u_position_invalidflags = u_Position_InvalidFlags;
    o.u_position_x = u_Position_X;
    o.u_position_x_std = u_Position_X_STD;
    o.u_position_y = u_Position_Y;
    o.u_position_y_std = u_Position_Y_STD;
    o.u_position_z = u_Position_Z;
    o.u_position_z_std = u_Position_Z_STD;
    o.u_position_covariancexy = u_Position_CovarianceXY;
    o.u_position_orientation = u_Position_Orientation;
    o.u_position_orientation_std = u_Position_Orientation_STD;
    o.u_existence_invalidflags = u_Existence_InvalidFlags;
    o.u_existence_ppv = u_Existence_PPV;
    o.u_existence_probability = u_Existence_Probability;
    o.u_dynamics_absaccel_invalidflags = u_Dynamics_AbsAccel_InvalidFlags;
    o.u_dynamics_absvel_invalidflags = u_Dynamics_AbsVel_InvalidFlags;
    o.u_dynamics_orientation_invalidflags = u_Dynamics_Orientation_InvalidFlags;
    o.u_dynamics_orientation_rate_mean = u_Dynamics_Orientation_Rate_Mean;
    o.u_dynamics_orientation_rate_std = u_Dynamics_Orientation_Rate_STD;
    o.u_dynamics_relaccel_invalidflags = u_Dynamics_RelAccel_InvalidFlags;
    o.u_dynamics_relvel_invalidflags = u_Dynamics_RelVel_InvalidFlags;
    o.u_shape_length_edge_invalidflags = u_Shape_Length_Edge_InvalidFlags;
    o.u_shape_length_edge_mean = u_Shape_Length_Edge_Mean;
    o.u_shape_length_edge_std = u_Shape_Length_Edge_STD;
    o.u_shape_length_status = u_Shape_Length_Status;
    o.u_shape_width_edge_invalidflags = u_Shape_Width_Edge_InvalidFlags;
    o.u_shape_width_edge_mean = u_Shape_Width_Edge_Mean;
    o.u_shape_width_edge_std = u_Shape_Length_Edge_STD;
    o.u_shape_width_status = u_Shape_Width_Status;
    o.u_statusmeasurement = u_StatusMeasurement;
    o.u_statusmovement = u_StatusMovement;
    o.u_statussensor = u_StatusSensor;
    o.u_classification_animal = u_Classification_Animal;
    o.u_classification_bicycle = u_Classification_Bicycle;
    o.u_classification_car = u_Classification_Car;
    o.u_classification_hazard = u_Classification_Hazard;
    o.u_classification_motorcycle = u_Classification_Motorcycle;
    o.u_classification_overdrivable = u_Classification_Overdrivable;
    o.u_classification_pedestrian = u_Classification_Pedestrian;
    o.u_classification_truck = u_Classification_Truck;
    o.u_classification_underdrivable = u_Classification_Underdrivable;
    o.u_classification_unknown = u_Position_CovarianceXY;
    o.f_dynamics_absaccel_covariancexy = f_Dynamics_AbsAccel_CovarianceXY;
    o.f_dynamics_absaccel_x = f_Dynamics_AbsAccel_X;
    o.f_dynamics_absaccel_x_std = f_Dynamics_AbsAccel_X_STD;
    o.f_dynamics_absaccel_y = f_Dynamics_AbsAccel_Y;
    o.f_dynamics_absaccel_y_std = f_Dynamics_AbsAccel_Y_STD;
    o.f_dynamics_absvel_covariancexy = f_Dynamics_AbsVel_CovarianceXY;
    o.f_dynamics_absvel_x = f_Dynamics_AbsVel_X;
    o.f_dynamics_absvel_x_std = f_Dynamics_AbsVel_X_STD;
    o.f_dynamics_absvel_y = f_Dynamics_AbsVel_Y;
    o.f_dynamics_absvel_y_std = f_Dynamics_AbsVel_Y_STD;
    o.f_dynamics_relaccel_covariancexy = f_Dynamics_RelAccel_CovarianceXY;
    o.f_dynamics_relaccel_x = f_Dynamics_RelAccel_X;
    o.f_dynamics_relaccel_x_std = f_Dynamics_RelAccel_X_STD;
    o.f_dynamics_relaccel_y = f_Dynamics_RelAccel_Y;
    o.f_dynamics_relaccel_y_std = f_Dynamics_RelAccel_Y_STD;
    o.f_dynamics_relvel_covariancexy = f_Dynamics_RelVel_CovarianceXY;
    o.f_dynamics_relvel_x = f_Dynamics_RelVel_X;
    o.f_dynamics_relvel_x_std = f_Dynamics_RelVel_X_STD;
    o.f_dynamics_relvel_y = f_Dynamics_RelVel_Y;
    o.f_dynamics_relvel_y_std = f_Dynamics_RelVel_Y_STD;  

    return o;
}
    