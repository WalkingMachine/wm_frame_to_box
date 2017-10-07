//
// Created by philippe on 07/10/17.
//

#include "wm_frame_to_box.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>

void ImageCB(const sensor_msgs::ImageConstPtr& msg){
    ROS_INFO("pixel %d", msg->data[0]);
}


int main(int argc, char **argv) {


    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("in_image_base_topic", 1, ImageCB);

}
