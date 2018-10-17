//
// Created by philippe on 07/10/17.
//

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include "visualization_msgs/Marker.h"
#include "sara_msgs/BoundingBoxes2D.h"
#include "sara_msgs/BoundingBoxes3D.h"
#include "wm_frame_to_box/GetBoundingBoxes3D.h"

double _CAMERA_ANGLE_WIDTH;
double _CAMERA_ANGLE_HEIGHT;
std::string _CAMERA_TOPIC;
std::string _YOLO_TOPIC;
std::string _CAMERA_FRAME;
std::string _BOUNDING_BOXES_TOPIC;
bool _AUTO_PLUBLISHER{false};
double _MIN_DIST;
double _MAX_DIST;
double _DEFAULT_BOX_SIZE;
int _NBSAMPLES{10};
std::string _OUTPUT_FRAME;
cv_bridge::CvImagePtr LastImage;
ros::Publisher posePub;
tf::TransformListener *tfl;
ros::Publisher markerPublisher;
bool _PUBLISH_MARKERS{false};


// Declare functions
double GetDepth(int x, int y, const cv_bridge::CvImagePtr cv_ptr);
sara_msgs::BoundingBoxes3D
get_BB(cv_bridge::CvImagePtr Img, sara_msgs::BoundingBoxes2D BBs, std::string input_frame, std::string output_frame);
void callbackBB(sara_msgs::BoundingBoxes2D msg);
void ImageCB(const sensor_msgs::ImageConstPtr &msg);
bool seviceCB(wm_frame_to_box::GetBoundingBoxes3D::Request &req, wm_frame_to_box::GetBoundingBoxes3D::Response &resp);




/**
 * Give the depth of a point from a depth image
 * @param x 		x position
 * @param x 		y position
 * @param cv_ptr    pointer to the depth image
 * @return depth    depth of the point
 */
double GetDepth(int x, int y, const cv_bridge::CvImagePtr cv_ptr) {
    // apply limitations
    if (x < 0) x = 0;
    if (x > cv_ptr->image.size.p[1]) x = cv_ptr->image.size.p[1];
    if (y < 0) y = 0;
    if (y > cv_ptr->image.size.p[0]) y = cv_ptr->image.size.p[0];

    int depth = cv_ptr->image.at<short int>(y, x);
    return depth / 1000.0;
}


/**
 * Give an estimation of the objects distance
 * @param x 		center x position
 * @param x 		center y position
 * @param cv_ptr    pointer to the depth image
 * @return distance depth of the point
 */
double GetMedDist(int xmin, int xmax, int ymin, int ymax, const cv_bridge::CvImagePtr cv_ptr) {

    int x{(xmin+xmax)/2};
    int y{(ymin+ymax)/2};

    int width{(xmax-xmin)/4};
    int height{(ymax-ymin)/4};

    double med{0};
    int nbSamples{0};

    for ( int i = 0 ;i < _NBSAMPLES; ++i){
        double dist{GetDepth(int(x+(cos(i)*i*width/_NBSAMPLES)), int(y+(sin(i)*i*height/_NBSAMPLES)), cv_ptr)};
        if (dist > _MIN_DIST){
            med += dist;
            nbSamples++;
        }
    }
    if (nbSamples == 0)
        return 0;

    med /= nbSamples;

    return med;
}


/**
 * Receive the image and process the depth of the bounding boxes
 * @param Img		depth image
 * @param BBs		2D bounding boxes
 * @return boxes    3D bounding boxes
 */
sara_msgs::BoundingBoxes3D
get_BB(cv_bridge::CvImagePtr Img, sara_msgs::BoundingBoxes2D BBs, std::string input_frame, std::string output_frame) {

    sara_msgs::BoundingBoxes3D_<std::allocator<void>> boxes;
    boxes = sara_msgs::BoundingBoxes3D();

    if (tfl == nullptr)
        return boxes;

    ulong L = BBs.boundingBoxes.size();
    if (L == 0)
        return boxes;

    for (ulong i = 0; i < L; i++) {
        // get frame
        int xmin = BBs.boundingBoxes[i].xmin;
        int xmax = BBs.boundingBoxes[i].xmax;
        int ymin = BBs.boundingBoxes[i].ymin;
        int ymax = BBs.boundingBoxes[i].ymax;

        // get center
        int x = (xmax + xmin) / 2;
        int y = (ymax + ymin) / 2;

        double dist = GetMedDist(xmin, xmax, ymin, ymax, Img);

        // add a 5 cm offset to compensate depth
        dist += 0.05;

        // filter invalid falues
        if (dist < _MIN_DIST || dist > _MAX_DIST){
            BBs.boundingBoxes[i].probability = 0;
            dist = 0.2;
        }
        ROS_INFO("%s dist: %lf prob: %lf", BBs.boundingBoxes[i].Class.data(), dist, BBs.boundingBoxes[i].probability);


        // get pixel to rad ratio // TODO check if the dimentions are right
        double xratio = _CAMERA_ANGLE_WIDTH / Img->image.size.p[1];
        double yratio = _CAMERA_ANGLE_HEIGHT / Img->image.size.p[0];

        // get the IRL angles from the camera center to the object
        double ax = -((double) x - (double) Img->image.size.p[1] / 2) * xratio;  // pixel to angle
        double ay = -((double) y - (double) Img->image.size.p[0] / 2) * yratio;  // pixel to angle
        double aw = -((double) xmax - (double) Img->image.size.p[1] / 2) * xratio;  // pixel to angle
        double ah = -((double) ymax - (double) Img->image.size.p[0] / 2) * yratio;  // pixel to angle

        // Convert the angeles and distance to x y z coordinates
        double pz{dist * std::cos(ax) * std::cos(ay)};  // ang to 3D point (rad to m)
        double px{-dist * std::sin(ax)};  // ang to 3D point (rad to m)
        double py{-dist * std::sin(ay)};  // ang to 3D point (rad to m)
        double pzwh{dist * std::cos(aw) * std::cos(ah)};  // ang to 3D point (rad to m)
        double pxwh{-dist * std::sin(aw)};  // ang to 3D point (rad to m)
        double pywh{-dist * std::sin(ah)};  // ang to 3D point (rad to m)


        /*** TF frame transformation ***/
        // Wait for the availability of the transformation
        tf::Stamped<tf::Vector3> loc;
        loc.stamp_ = BBs.header.stamp;
        tfl->waitForTransform(output_frame, input_frame, BBs.header.stamp, ros::Duration(0.01));


        // Apply transformation to the new reference frame and Generate the center of the box
        loc.frame_id_ = input_frame;  // Reference frame
        loc.setX(px); loc.setY(py); loc.setZ(pz);
        tfl->transformPoint(output_frame, loc, loc );
        geometry_msgs::Point po;
        po.x = loc.x(); po.y = loc.y(); po.z = loc.z();

        // Apply transformation to the new reference frame and Generate the dimentions of the box
        loc.frame_id_ = input_frame;  // Reference frame
        loc.setX(pxwh); loc.setY(pywh); loc.setZ(pzwh);
        tfl->transformPoint(output_frame, loc, loc );
        geometry_msgs::Point dims;
        double radius{sqrt(pow(loc.x()-po.x, 2) + pow(loc.y()-po.y,2))};
        dims.x = radius*2; dims.y = radius*2; dims.z = (loc.z()-po.z)*2;


        /*** Create the box ***/
        // create a box message and fill all the parameters
        sara_msgs::BoundingBox3D box;
        box.Class = BBs.boundingBoxes[i].Class;
        box.Center = po;

        // set the dimentions of the box
        box.Depth = dims.x;
        box.Width = dims.y;
        box.Height = dims.z;
        box.probability = BBs.boundingBoxes[i].probability;

        // Add the box to the list of boxes
        boxes.boundingBoxes.push_back(box);

        /*** Publish the boxes ***/
        if (_PUBLISH_MARKERS){  // Publish visual box
            visualization_msgs::Marker m;
            m.header.stamp = ros::Time::now();
            m.lifetime = ros::Duration(0.1);
            m.header.frame_id = output_frame;
            m.ns = box.Class;
            m.id = ros::Time::now().toNSec()+int(box.probability*1000);
            m.type = m.CUBE;
            m.pose.position.x = box.Center.x;
            m.pose.position.y = box.Center.y;
            m.pose.position.z = box.Center.z;
            m.scale.x = box.Depth;
            m.scale.y = box.Width;
            m.scale.z = box.Height;
            m.color.r = 0;
            m.color.g = 1;
            m.color.b = 0;
            m.color.a = 0.2;
            markerPublisher.publish(m);
        }

    }
    boxes.header = BBs.header;
    return boxes;
}


/**
 * Bounding Boxes callback. Receive a list of 2D bounding boxes and publish 3D bounding boxes.
 * @param msg 		received image
 */
void callbackBB(sara_msgs::BoundingBoxes2D msg) {
    if (!LastImage){
        ROS_WARN("no image received");
        return;
    }
    try {
        sara_msgs::BoundingBoxes3D boxes3D;
        boxes3D = get_BB(LastImage, msg, msg.header.frame_id, _OUTPUT_FRAME);
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
        resp.boundingBoxes3D = get_BB(ptr, req.boundingBoxes2D, req.input_frame, req.output_frame);
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

    // get all parameters
    nh.param("auto_publisher", _AUTO_PLUBLISHER, false);
    nh.param("minimum_distance", _MIN_DIST, 0.2);
    nh.param("maximum_distance", _MAX_DIST, 50.0);
    nh.param("camera_angle_width", _CAMERA_ANGLE_WIDTH, 1.012290966);
    nh.param("camera_angle_height", _CAMERA_ANGLE_HEIGHT, 0.785398163397);
    nh.param("camera_topic", _CAMERA_TOPIC, std::string("/head_xtion/depth/image_raw"));
    nh.param("yolo_topic", _YOLO_TOPIC, std::string("/darknet_ros/bounding_boxes"));
    nh.param("bounding_boxes_topic", _BOUNDING_BOXES_TOPIC, std::string("/frame_to_box/bounding_boxes"));
    nh.param("default_box_size", _DEFAULT_BOX_SIZE, 0.1);
    nh.param("camera_frame", _CAMERA_FRAME, std::string("head_xtion_depth_frame"));
    nh.param("base_frame", _OUTPUT_FRAME, std::string("/base_link"));
    nh.param("nb_samples", _NBSAMPLES, 10);
    nh.param("publish_markers", _PUBLISH_MARKERS, false);

    // Initialise tf listener
    tfl = new tf::TransformListener(nh, ros::Duration(20) ,true);

    markerPublisher = nh.advertise<visualization_msgs::Marker>("/boxes", 100);

    if (_AUTO_PLUBLISHER) {
        // subscribe to the camera topic
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber sub = it.subscribe(_CAMERA_TOPIC, 1, ImageCB);
        LastImage = nullptr;

        // subscribe to the yolo topic
        ros::Subscriber bbsub = nh.subscribe(_YOLO_TOPIC, 1, callbackBB);

        // advertise the box3D topic
        posePub = nh.advertise<sara_msgs::BoundingBoxes3D>(_BOUNDING_BOXES_TOPIC, 10);
    }
    // advertise the service
    ros::ServiceServer service = nh.advertiseService("get_3d_bounding_boxes", seviceCB);
    // run run run!
    ros::spin();

    delete(tfl);
}
