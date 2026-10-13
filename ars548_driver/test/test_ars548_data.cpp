/**
 * @file test_ars548_data.cpp
 * @brief Checks that every list-level field decoded from the radar UDP structs
 *        is copied into the published ROS messages.
 *
 * The structs are filled in host byte order (i.e. as they look after
 * changeEndianness() has run on a received datagram) and converted with
 * toMsg(), exactly as ars548_driver.cpp does.
 */
#include <gtest/gtest.h>

#include <memory>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "ars548_driver/ars548_data.h"

TEST(DetectionList, ToMsgCopiesListLevelFields)
{
    // The struct is ~35 kB, keep it off the stack. Value-initialisation zeroes it.
    auto list = std::make_unique<DetectionList>();
    list->MethodID = DETECTION_MESSAGE_METHOD_ID;
    list->PayloadLength = DETECTION_MESSAGE_PDU_LENGTH;
    ASSERT_TRUE(list->isValid());

    list->List_InvalidFlags = 0x5A;
    list->List_RadVelDomain_Min = -41.25f;
    list->List_RadVelDomain_Max = 41.25f;
    list->Aln_AzimuthCorrection = 0.0123f;
    list->Aln_ElevationCorrection = -0.0045f;
    list->Aln_Status = 2;

    list->List_NumOfDetections = 1;
    list->List_Detections[0].f_Range = 12.5f;
    list->List_Detections[0].f_RangeRate = -3.0f;

    const ars548_messages::msg::DetectionList msg =
        list->toMsg("radar", rclcpp::Time(0), true);

    // Fields that were silently left at zero before this test existed.
    EXPECT_EQ(msg.list_invalidflags, 0x5A);
    EXPECT_FLOAT_EQ(msg.list_radveldomain_min, -41.25f);
    EXPECT_FLOAT_EQ(msg.list_radveldomain_max, 41.25f);
    EXPECT_FLOAT_EQ(msg.aln_azimuthcorrection, 0.0123f);
    EXPECT_FLOAT_EQ(msg.aln_elevationcorrection, -0.0045f);

    // Fields that were already copied, kept as regression guard.
    EXPECT_EQ(msg.aln_status, 2);
    EXPECT_EQ(msg.list_numofdetections, 1u);
    EXPECT_FLOAT_EQ(msg.list_detections[0].f_range, 12.5f);
    EXPECT_FLOAT_EQ(msg.list_detections[0].f_rangerate, -3.0f);
    EXPECT_EQ(msg.header.frame_id, "radar");
}

TEST(DetectionList, ToMsgClampsDetectionCount)
{
    auto list = std::make_unique<DetectionList>();
    list->List_NumOfDetections = ARS548_MAX_DETECTIONS + 7;

    const ars548_messages::msg::DetectionList msg =
        list->toMsg("radar", rclcpp::Time(0), true);

    EXPECT_EQ(msg.list_numofdetections, static_cast<uint32_t>(ARS548_MAX_DETECTIONS));
}

TEST(Object, ToMsgCopiesPositionReference)
{
    Object object{};
    object.u_ID = 42;
    object.u_Position_Reference = 3;
    object.u_Position_X = 1.5f;

    const ars548_messages::msg::Object msg = object.toMsg();

    EXPECT_EQ(msg.u_id, 42u);
    EXPECT_EQ(msg.u_position_reference, 3);
    EXPECT_FLOAT_EQ(msg.u_position_x, 1.5f);
}

TEST(ObjectList, ToMsgCopiesListLevelFields)
{
    auto list = std::make_unique<ObjectList>();
    list->MethodID = OBJECT_MESSAGE_METHOD_ID;
    list->PayloadLength = OBJECT_MESSAGE_PDU_LENGTH;
    ASSERT_TRUE(list->isValid());

    list->Timestamp_Seconds = 1700000000u;
    list->Timestamp_Nanoseconds = 123456789u;
    list->Timestamp_SyncStatus = 1;
    list->ObjectList_NumOfObjects = 1;
    list->ObjectList_Objects[0].u_Position_Reference = 5;

    const ars548_messages::msg::ObjectList msg =
        list->toMsg("radar", rclcpp::Time(0), false);

    EXPECT_EQ(msg.timestamp_seconds, 1700000000u);
    EXPECT_EQ(msg.timestamp_nanoseconds, 123456789u);
    EXPECT_EQ(msg.timestamp_syncstatus, 1);
    EXPECT_EQ(msg.objectlist_numofobjects, 1);
    EXPECT_EQ(msg.objectlist_objects[0].u_position_reference, 5);
    EXPECT_EQ(msg.header.stamp.sec, 1700000000);
    EXPECT_EQ(msg.header.stamp.nanosec, 123456789u);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
