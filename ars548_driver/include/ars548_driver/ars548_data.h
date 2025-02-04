/**
 * @file ars548_data.h
 * @brief ars548_data contains all of the Structures needed to save and modify the data from the radar.
 * It is also used to fill the messages sent to the user.
 * @brief Data obtained from the RadarSensors_Annex_AES548_IO SW 05.48.04.pdf
 */
#pragma once
#include <cstdint>


#pragma pack(1)


#include "ars548_data/udp_status.h"
#include "ars548_data/object_list.h"
#include "ars548_data/detection_list.h"

// CONFIGURATION MESSAGES
#include "ars548_data/sensor_configuration.h"

// AUTO_ALIGNMENT MESSAGES
#include "ars548_data/acceleration_lateral_cog.h"
#include "ars548_data/acceleration_longitudinal_cog.h"
#include "ars548_data/characteristic_speed.h"
#include "ars548_data/driving_direction.h"
#include "ars548_data/steering_angle_front_axle.h"
#include "ars548_data/velocity_vehicle.h"
#include "ars548_data/yaw_rate.h"

#pragma pack(4)


