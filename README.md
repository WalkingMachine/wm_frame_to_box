# wm_frame_to_box

ROS node receiving 2D bounding boxes from YOLO v2 (ROS wrapper) and convert them to 3D bounding boxes using camera depthcloud

## Getting Started

* ```cd catkin_workspace/src```    
* ```git clone```  
* ```git@github.com:WalkingMachine/wm_frame_to_box.git```    
* ```cd ../```      
* ```catkin_make```
* ```roslaunch wm_frame_to_box wm_frame_to_box.launch```

### Prerequisites

* Ubuntu 16.04
* ROS Kinetic
* Openni
* OpenCV
* [cv_bridge](http://wiki.ros.org/cv_bridge)
* [sara_msgs](https://github.com/WalkingMachine/sara_msgs)
* [YOLO v2 for ROS](https://github.com/WalkingMachine/darknet_ros)


### Subscribed topics

* **`/head_xtion/depth/image`** [sensor_msgs/Image]

	Depth camera topic.

* **`/darknet_ros/bounding_boxes`** [darknet_ros_msgs/BoundingBoxes]

	Darknet 2D bounding boxes topic.

### Published topics

* **`/frame_to_box/bounding_boxes`** [wm_frame_to_box/BoundingBoxes3D]

	3D bounding boxes published topic.

### Services
* **[`/frame_to_box/GetBoundingBoxes3D`](https://github.com/WalkingMachine/wm_frame_to_box/blob/master/srv/GetBoundingBoxes3D.srv)**



### Parameters
* **`auto_publisher`** (bool, default: true)

	Auto-publish to the topics.

* **`camera_angle_width`** (float, default: 0.785398163397)

	Camera angle of view width in rad.

* **`camera_angle_height`** (float, default: 0.785398163397)

	Camera angle of view height in rad.

* **`minimum_distance`** (float, default: 0.2)

	Bounding boxes minimum distance in meters.

* **`maximum_distance`** (float, default: 50)

	Bounding boxes maximum distance in meters.

* **`camera_topic`** (string, default: "/head_xtion/depth/image_raw")

	Depth camera topic.

* **`camera_frame`** (string, default: "head_xtion_depth_frame")

	Depth camera tf frame.

* **`base_frame`** (string, default: "base_link")

	Reference base frame.

* **`frame_ag`** (float, default: 0.0)

	Time shift to look for bounding boxes tf.

* **`yolo_topic`** (string, default: "/darknet_ros/bounding_boxes")

	Darknet 2D bounding boxes topic.

* **`bounding_boxes_topic`** (string, default: "/frame_to_boxes/bounding_boxes")

	3D bounding boxes published topic.

* **`default_box_size`** (float, default: 0.1)

	Bounding boxes default size


## Contributing

Please read [CONTRIBUTING.md](https://github.com/walkingmachine/wm_frame_to_box/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.

## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/WalkingMachine/wm_frame_to_box/tags).

## Authors

* **Philippe La Madeleine** - *Initial work* - [Philippe117](https://github.com/Philippe117)

See also the list of [contributors](https://github.com/walkingmachine/wm_frame_to_box/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
