//
// Created by philippe on 07/10/17.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <wm_frame_to_box/BoundingBoxes3D.h>

darknet_ros_msgs::BoundingBoxes BoundingBoxes2D;

double _IMAGE_RATIO_H = 1.221730476;  // pixel to rad
double _IMAGE_RATIO_V = 0.785398163397;  // pixel to rad
std::string _CAMERA_TOPIC = "/head_xtion/depth/image_raw";
std::string _YOLO_TOPIC = "/darknet_ros/bounding_boxes";
std::string _CAMERA_FRAME = "head_xtion_depth_frame";
std::string _BOUNDING_BOXES_TOPIC = "/frame_to_box/bounding_boxes";
double _DEFAULT_BOX_SIZE = 0.1;

wm_frame_to_box::BoundingBoxes3D boxes;
ros::Publisher posePub;

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



// Save the bounding boxes for later
void DNBB( darknet_ros_msgs::BoundingBoxes msg ){

    BoundingBoxes2D = msg;
}

// Receive the image and process the depth of the bounding boxes
void ImageCB(const sensor_msgs::ImageConstPtr& msg){

    boxes.boundingBoxes.clear();


    ulong L = BoundingBoxes2D.boundingBoxes.size();
    for (ulong i=0; i < L; i++ ){

        // get frame
        int xmin = BoundingBoxes2D.boundingBoxes[i].xmin;
        int xmax = BoundingBoxes2D.boundingBoxes[i].xmax;
        int ymin = BoundingBoxes2D.boundingBoxes[i].ymin;
        int ymax = BoundingBoxes2D.boundingBoxes[i].ymax;

        // get center
        int x = (xmax+xmin)/2;
        int y = (ymax+ymin)/2;

        // apply limitations
        if (x<0) x=0;
        if (x>msg->width) x=msg->width;
        if (y<0) y=0;
        if (y>msg->height) y=msg->height;


        double dist = ReadDepthData( (uint)x, (uint)y, msg )/1000.0;  // mm to m

//        ROS_INFO("x = %d", x );
//        ROS_INFO("y = %d", y );
//        ROS_INFO("dist = %f", dist );

        double xratio = msg->width*_IMAGE_RATIO_H;
        double yratio = msg->height*_IMAGE_RATIO_V;

        double ax = -((double)x-(double)msg->width/2)/xratio;  // pixel to angle
        double ay = -((double)y-(double)msg->height/2)/yratio;  // pixel to angle

        double ry = dist*std::sin(ax);  // ang to 3D point (rad to m)
        double rz = dist*std::sin(ay);  // ang to 3D point (rad to m)
        double rx = dist*std::cos(ax)*std::cos(ay);  // ang to 3D point (rad to m)

        // add the box to the message
        wm_frame_to_box::BoundingBox3D box;
        box.Class = BoundingBoxes2D.boundingBoxes[i].Class;
        geometry_msgs::Point po;
        po.x = rx;
        po.y = ry;
        po.z = rz;
        box.Center = po;  // TODO set the right frame
        box.Depth = _DEFAULT_BOX_SIZE;
        box.Width = _DEFAULT_BOX_SIZE;
        box.Height = _DEFAULT_BOX_SIZE;
        box.probability = BoundingBoxes2D.boundingBoxes[i].probability;
        boxes.boundingBoxes.push_back(box);
        posePub.publish(boxes);

        // broadcast the boxes to TF
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(rx, ry, rz) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _CAMERA_FRAME, BoundingBoxes2D.boundingBoxes[i].Class));
    }

    BoundingBoxes2D.boundingBoxes.clear();







}


int main(int argc, char **argv) {

    ros::init(argc, argv, "frame_to_box");

    ros::NodeHandle nh;

    nh.getParam("image_ratio_h", _IMAGE_RATIO_H);
    nh.getParam("image_ratio_v", _IMAGE_RATIO_V);
    nh.getParam("camera_topic", _CAMERA_TOPIC);
    nh.getParam("yolo_topic", _YOLO_TOPIC);
    nh.getParam("bounding_boxes_topic", _BOUNDING_BOXES_TOPIC);
    nh.getParam("default_box_size", _DEFAULT_BOX_SIZE);
    nh.getParam("camera_frame", _CAMERA_FRAME);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(_CAMERA_TOPIC, 1, ImageCB);

    ros::Subscriber bbsub = nh.subscribe(_YOLO_TOPIC, 1, DNBB);

    posePub = nh.advertise<wm_frame_to_box::BoundingBoxes3D>( _BOUNDING_BOXES_TOPIC, 10 );

    ros::Rate period(0.1);  // 10 Hz
    while( ros::ok() ){
        ros::spinOnce();
        period.sleep();
    }

}
