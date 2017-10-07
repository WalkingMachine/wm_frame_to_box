//
// Created by philippe on 07/10/17.
//

#include "wm_frame_to_box.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>


// from: https://answers.ros.org/question/90696/get-depth-from-kinect-sensor-in-gazebo-simulator/
typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;
int ReadDepthData(unsigned int height_pos, unsigned int width_pos, sensor_msgs::ImageConstPtr depth_image)
{
    // If position is invalid
    if ((height_pos >= depth_image->height) || (width_pos >= depth_image->width))
        return -1;
    int index = (height_pos*depth_image->step) + (width_pos*(depth_image->step/depth_image->width));
    // If data is 4 byte floats (rectified depth image)
    if ((depth_image->step/depth_image->width) == 4) {
        U_FloatConvert depth_data;
        int i, endian_check = 1;
        // If big endian
        if ((depth_image->is_bigendian && (*(char*)&endian_check != 1)) ||  // Both big endian
            ((!depth_image->is_bigendian) && (*(char*)&endian_check == 1))) { // Both lil endian
            for (i = 0; i < 4; i++)
                depth_data.byte_data[i] = depth_image->data[index + i];
            // Make sure data is valid (check if NaN)
            if (depth_data.float_data == depth_data.float_data)
                return int(depth_data.float_data*1000);
            return -1;  // If depth data invalid
        }
        // else, one little endian, one big endian
        for (i = 0; i < 4; i++)
            depth_data.byte_data[i] = depth_image->data[3 + index - i];
        // Make sure data is valid (check if NaN)
        if (depth_data.float_data == depth_data.float_data)
            return int(depth_data.float_data*1000);
        return -1;  // If depth data invalid
    }
    // Otherwise, data is 2 byte integers (raw depth image)
    int temp_val;
    // If big endian
    if (depth_image->is_bigendian)
        temp_val = (depth_image->data[index] << 8) + depth_image->data[index + 1];
        // If little endian
    else
        temp_val = depth_image->data[index] + (depth_image->data[index + 1] << 8);
    // Make sure data is valid (check if NaN)
    if (temp_val == temp_val)
        return temp_val;
    return -1;  // If depth data invalid
}





void ImageCB(const sensor_msgs::ImageConstPtr& msg){
    uint x;
    uint y;

    x = (uint)(msg->width/2);
    y = (uint)(msg->height/2);
    ROS_INFO("x %d", x);
    ROS_INFO("y %d", y);
    double pixel = ReadDepthData( x, y, msg )/1000.0;

    ROS_INFO("pixel %d", pixel);


    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pixel, 0.0, 0.0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "head_xtion_depth_frame", "center_of_view"));

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "frame_to_box");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/head_xtion/depth/image_raw", 1, ImageCB);

    ros::spin();
}
