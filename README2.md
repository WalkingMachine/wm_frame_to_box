# Project Title

ROS node receiving 2D bounding boxes and convert them to 3D bounding boxes using camera depthcloud

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
* [YOLO v2 for ROS](https://github.com/WalkingMachine/darknet_ros)

### Parameters
* **`auto_publisher`** (bool, default: true)

	Faut-il souscrire et publier automatiquement aux topic. Le service fonctionne quand même.

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

	Temps de décalage en (s) dans le passé où regarder pour les tf.

* **`yolo_topic`** (string, default: "/darknet_ros/bounding_boxes")

	Topic où aller chercher les cadres de darknet

* **`bounding_boxes_topic`** (string, default: "/frame_to_boxes/bounding_boxes")

	Topic où publier les boites 3d

* **`default_box_size`** (float, default: 0.1)

	Dimensions standards des boites





## Contributing

Please read [CONTRIBUTING.md](https://github.com/walkingmachine/wm_frame_to_box/CONTRIBUTING.md) for details on our code of conduct, and the process for submitting pull requests to us.


## Authors

* **Philippe La Madeleine** - *Initial work* - [Philippe117](https://github.com/Philippe117)

See also the list of [contributors](https://github.com/walkingmahcine/wm_frame_to_box/contributors) who participated in this project.

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details
