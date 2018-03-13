//
// Created by philippe on 07/10/17.
//

#include <image_transport/image_transport.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include "sara_msgs/BoundingBoxes2D.h"
#include "sara_msgs/BoundingBoxes3D.h"
#include "wm_frame_to_box/GetBoundingBoxes3D.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

darknet_ros_msgs::BoundingBoxes BoundingBoxes2D;

double _CAMERA_ANGLE_WIDTH;
double _CAMERA_ANGLE_HEIGHT;
std::string _CAMERA_TOPIC;
std::string _YOLO_TOPIC;
std::string _CAMERA_FRAME;
std::string _BOUNDING_BOXES_TOPIC;
double _DEFAULT_BOX_SIZE;
std::string _BASE_FRAME;
cv_bridge::CvImagePtr LastImage;

ros::Publisher posePub;

tf::TransformListener *Listener2;



/**
 * Receive a list of bounding boxes from sara_msgs and convert them to Darknet standard bounding boxes
 * @param DBBs 		sara bounding boxes
 * @return BBsOut 	Darknet bounding boxes
 */
std::vector<darknet_ros_msgs::BoundingBox> ConvertBB(std::vector<sara_msgs::BoundingBox2D> DBBs) {
    std::vector<darknet_ros_msgs::BoundingBox> BBsOut;
    for (auto DBB : DBBs) {
        darknet_ros_msgs::BoundingBox BBOut;
        BBOut.Class = DBB.Class;
        BBOut.probability = DBB.probability;
        BBOut.xmax = DBB.xmax;
        BBOut.xmin = DBB.xmin;
        BBOut.ymax = DBB.ymax;
        BBOut.ymin = DBB.ymin;
        BBsOut.push_back(BBOut);
    }
    return BBsOut;
}

/**
 * Give the depth of a point from a depth image
 * @param x 		x position
 * @param x 		y position
 * @param cv_ptr    pointer to the depth image
 * @return depth    depth of the point
 */
double GetDepth(int x, int y, const cv_bridge::CvImagePtr cv_ptr) {
    int depth = cv_ptr->image.at<short int>(y, x);
    return depth / 1000.0;
}


/**
 * Receive the image and process the depth of the bounding boxes
 * @param Img		depth image
 * @param BBs		2D bounding boxes
 * @return boxes    3D bounding boxes
 */
std::vector<sara_msgs::BoundingBox3D>
get_BB(cv_bridge::CvImagePtr Img, std::vector<darknet_ros_msgs::BoundingBox> BBs) {

    sara_msgs::BoundingBoxes3D_<std::allocator<void>> boxes;
    boxes = sara_msgs::BoundingBoxes3D();

    ulong L = BBs.size();
    if (L == 0)
        return boxes.boundingBoxes;

    for (ulong i = 0; i < L; i++) {
        // get frame
        int xmin = BBs[i].xmin;
        int xmax = BBs[i].xmax;
        int ymin = BBs[i].ymin;
        int ymax = BBs[i].ymax;

        // get center
        int x = (xmax + xmin) / 2;
        int y = (ymax + ymin) / 2;

        ROS_INFO("width: %d", Img->image.size.p[1]);
        ROS_INFO("height: %d", Img->image.size.p[0]);

        // apply limitations
        if (x < 0) x = 0;
        if (x > Img->image.size.p[1]) x = Img->image.size.p[1];
        if (y < 0) y = 0;
        if (y > Img->image.size.p[0]) y = Img->image.size.p[0];
        double dist = GetDepth(x, y, Img);

        // add a 5 cm offset to compensate depth
        dist += 0.05;

        // filter invalid falues
        if (dist < 0.2)
            dist = 0.2;
        ROS_INFO("%s dist: %lf", BBs[i].Class.data(), dist);


        // get pixel to rad ratio // TODO check if the dimentions are right
        double xratio = _CAMERA_ANGLE_WIDTH / Img->image.size.p[1];
        double yratio = _CAMERA_ANGLE_HEIGHT / Img->image.size.p[0];

        // get the IRL angles from the camera center to the object
        double ax = -((double) x - (double) Img->image.size.p[1] / 2) * xratio;  // pixel to angle
        double ay = -((double) y - (double) Img->image.size.p[0] / 2) * yratio;  // pixel to angle

        // calculate the relative position in the camera frame.
        double ry = dist * std::sin(ax);  // ang to 3D point (rad to m)
        double rz = dist * std::sin(ay);  // ang to 3D point (rad to m)
        double rx = dist * std::cos(ax) * std::cos(ay);  // ang to 3D point (rad to m)

        // broadcast the boxe to TF
        auto BoxName = BBs[i].Class;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(rx, ry, rz));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), _CAMERA_FRAME, BoxName));

        // create a box message and fill all the parameters
        sara_msgs::BoundingBox3D box;
        box.Class = BBs[i].Class;

        // get the center of the box relatively to the base_link using tf
        tf::StampedTransform tranform;
        std::string buffer = "/" + BoxName;
        const char *BoxFrame = buffer.c_str();
        Listener2->waitForTransform(_BASE_FRAME, BoxFrame, ros::Time(0), ros::Duration(40));
        Listener2->lookupTransform(_BASE_FRAME, BoxFrame, ros::Time(0), tranform);
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
        box.probability = BBs[i].probability;
        boxes.boundingBoxes.push_back(box);
        posePub.publish(boxes);

    }
    return boxes.boundingBoxes;
}


/**
 * Bounding Boxes callback. Receive a list of 2D bounding boxes and publish 3D bounding boxes.
 * @param msg 		received image
 */
void callbackBB(darknet_ros_msgs::BoundingBoxes msg) {
    if (!LastImage){
        ROS_WARN("no image received");
        return;
    }
    try {
        std::vector<sara_msgs::BoundingBox3D> boxes3D;
        boxes3D = get_BB(LastImage, msg.boundingBoxes);
        posePub.publish(boxes3D);
    }catch (std::string exeption) {
        ROS_ERROR("callack error");
    }
}


/**
 * Image callback. Receive an image and save it for later.
 * @param msg 		received image
 */
void ImageCB(const sensor_msgs::ImageConstPtr &msg) {
    LastImage = cv_bridge::toCvCopy(msg);
}


/**
 * Bounding Boxes convertion service. Convert BountingBoxes2D into BoundingBoxes3D
 * @param msg 		service message
 */
bool seviceCB(wm_frame_to_box::GetBoundingBoxes3D::Request &req, wm_frame_to_box::GetBoundingBoxes3D::Response &resp) {
    try {
        cv_bridge::CvImagePtr ptr = cv_bridge::toCvCopy(req.image);
        resp.boundingBoxes3D = get_BB(ptr, ConvertBB(req.boundingBoxes2D));
    }
    catch (std::string exeption) {
        ROS_ERROR("service error");
        return false;
    }
    return true;
}


/**
 * Main routine
 * @param argc 		ros argc
 * @param argv 		ros argv
 */
int main(int argc, char **argv) {

    ros::init(argc, argv, "frame_to_box");

    ros::NodeHandle nh;
    tf::TransformListener Listener;
    Listener2 = &Listener;

    // get all parameters
    nh.param("camera_angle_width", _CAMERA_ANGLE_WIDTH, 1.012290966);
    ROS_INFO("camera_angle_width = %f", _CAMERA_ANGLE_WIDTH);
    nh.param("camera_angle_height", _CAMERA_ANGLE_HEIGHT, 0.785398163397);
    ROS_INFO("camera_angle_height = %f", _CAMERA_ANGLE_HEIGHT);
    nh.param("camera_topic", _CAMERA_TOPIC, std::string("/head_xtion/depth/image_raw"));
    ROS_INFO("camera_topic = %s", _CAMERA_TOPIC.c_str());
    nh.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
    ROS_INFO("yolo_topic = %s", _YOLO_TOPIC.c_str());
    nh.param("bounding_boxes_topic", _BOUNDING_BOXES_TOPIC, std::string("/frame_to_box/bounding_boxes"));
    ROS_INFO("bounding_boxes_topic = %s", _BOUNDING_BOXES_TOPIC.c_str());
    nh.param("default_box_size", _DEFAULT_BOX_SIZE, 0.1);
    ROS_INFO("default_box_size = %f", _DEFAULT_BOX_SIZE);
    nh.param("camera_frame", _CAMERA_FRAME, std::string("head_xtion_depth_frame"));
    ROS_INFO("camera_frame = %s", _CAMERA_FRAME.c_str());
    nh.param("base_frame", _BASE_FRAME, std::string("/base_link"));
    ROS_INFO("base_frame = %s", _BASE_FRAME.c_str());

    // subscribe to the camera topic
//    image_transport::ImageTransport it(nh);
//    image_transport::Subscriber sub = it.subscribe(_CAMERA_TOPIC, 1, ImageCB);
    LastImage = nullptr;

    // subscribe to the yolo topic
//    ros::Subscriber bbsub = nh.subscribe(_YOLO_TOPIC, 1, callbackBB);

    // advertise the box3D topic
    posePub = nh.advertise<sara_msgs::BoundingBoxes3D>(_BOUNDING_BOXES_TOPIC, 10);

    // advertise the service
    ros::ServiceServer service = nh.advertiseService("get_3d_bounding_boxes", seviceCB);
    // run run run!
    ros::spin();
}
