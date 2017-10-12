//
// Created by philippe on 07/10/17.
//

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <wm_frame_to_box/BoundingBoxes3D.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

darknet_ros_msgs::BoundingBoxes BoundingBoxes2D;

double _CAMERA_ANGLE_WIDTH = 1.221730476;  // pixel to rad
double _CAMERA_ANGLE_HEIGHT = 0.785398163397;  // pixel to rad
std::string _CAMERA_TOPIC = "/head_xtion/depth/image_raw";
std::string _YOLO_TOPIC = "/darknet_ros/bounding_boxes";
std::string _CAMERA_FRAME = "head_xtion_depth_frame";
std::string _BOUNDING_BOXES_TOPIC = "/frame_to_box/bounding_boxes";
double _DEFAULT_BOX_SIZE = 0.1;
std::string _BASE_FRAME = "base_link";  // frame of the output

wm_frame_to_box::BoundingBoxes3D boxes;
ros::Publisher posePub;
tf::Transformer transformer;

double GetDepth( int x, int y, const sensor_msgs::ImageConstPtr& msg ){

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return -1;
    }

    int depth = cv_ptr->image.at<short int>(x, y);
    ROS_INFO("Depth: %d", depth);

    return depth/1000.0;
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


        double dist = GetDepth( x, y, msg );

//        ROS_INFO("x = %d", x );
//        ROS_INFO("y = %d", y );
//        ROS_INFO("dist = %f", dist );

        double xratio = _CAMERA_ANGLE_WIDTH/msg->width;
        double yratio = _CAMERA_ANGLE_HEIGHT/msg->height;

        double ax = -((double)x-(double)msg->width/2)/xratio;  // pixel to angle
        double ay = -((double)y-(double)msg->height/2)/yratio;  // pixel to angle

        double ry = dist*std::sin(ax);  // ang to 3D point (rad to m)
        double rz = dist*std::sin(ay);  // ang to 3D point (rad to m)
        double rx = dist*std::cos(ax)*std::cos(ay);  // ang to 3D point (rad to m)

        std::string BoxName = BoundingBoxes2D.boundingBoxes[i].Class;

        // broadcast the boxe to TF
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(rx, ry, rz) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _CAMERA_FRAME, BoxName));

        // create a box and set all the parameters
        wm_frame_to_box::BoundingBox3D box;
        box.Class = BoundingBoxes2D.boundingBoxes[i].Class;

        // get the center of the box
        tf::StampedTransform tranform;
        transformer.lookupTransform( _BASE_FRAME, BoxName, ros::Time::now() , tranform );
        tf::Vector3 origin = tranform.getOrigin();
        geometry_msgs::Point po;
        po.x = origin.x();
        po.y = origin.y();
        po.z = origin.z();
        box.Center = po;

        // set the dimentions of the box
        box.Depth = _DEFAULT_BOX_SIZE;
        box.Width = _DEFAULT_BOX_SIZE;
        box.Height = _DEFAULT_BOX_SIZE;
        box.probability = BoundingBoxes2D.boundingBoxes[i].probability;
        boxes.boundingBoxes.push_back(box);
        posePub.publish(boxes);

    }

    BoundingBoxes2D.boundingBoxes.clear();

}


int main(int argc, char **argv) {

    ros::init(argc, argv, "frame_to_box");

    ros::NodeHandle nh;

    nh.getParam("camera_angle_width", _CAMERA_ANGLE_WIDTH);
    nh.getParam("camera_angle_height", _CAMERA_ANGLE_HEIGHT);
    nh.getParam("camera_topic", _CAMERA_TOPIC);
    nh.getParam("yolo_topic", _YOLO_TOPIC);
    nh.getParam("bounding_boxes_topic", _BOUNDING_BOXES_TOPIC);
    nh.getParam("default_box_size", _DEFAULT_BOX_SIZE);
    nh.getParam("camera_frame", _CAMERA_FRAME);
    nh.getParam("base_frame", _BASE_FRAME);

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
