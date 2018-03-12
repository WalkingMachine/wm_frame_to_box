# wm_frame_to_box

## Description

Node recevant des cadres et les convertissant en boites à l'aide du depth de la camera

Testé pour fonctionner avec [ROS] kinetic et Ubuntu 16.04. Le code risque de changer à certain moments.
La licence du code source est [MIT license](LICENSE).

**Auteur(s): Philippe La Madeleine
Mainteneur: Philippe La Madeleine  
Affiliation: Club étudiant Walking Machine, ÉTS**

## Installation

### Buildé de la source

#### Dépendances

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware pour la robotique),
- [sara_msg](https://github.com/WalkingMachine/sara_msgs) (repo pour les messages utilisé par sara),
- [darknet_ros](https://github.com/leggedrobotics/darknet_ros) (package de darknet fait par leggedrobotics),
- [Openni] (camera 3d)
- [Open_cv] (vision artificielle)


#### Building

Pour build de la source, clonez la dernière version de ce repo dans votre workspace et compilez avec

	cd catkin_workspace/src
	git clone git@github.com:WalkingMachine/wm_frame_to_box.git
	cd ../
	catkin_make

## Utilisation

Lancer le launchfile avec

	roslaunch wm_frame_to_box wm_frame_to_box.launch


## Nodes

### frame_to_box

Reçoit le depth de la camera et les cadres publiés par darknet. Retourne des boites en 3d sous forme de tf et de message.


#### Topics Souscris

* **`/head_xtion/depth/image`** ([sensor_msgs/Image])

	L'image de profondeur de la camera 3d.

* **`/darknet_ros/bounding_boxes`** ([darknet_ros_msgs/BoundingBoxes])

	Les cadres 2d retournés par darknet_ros

#### Topics Publiés

* **`/frame_to_box/bounding_boxes`** ([wm_frame_to_box/BoundingBoxes3D])

	Les boites 3d obtenues

#### Service

* **`/frame_to_box/GetBoundingBoxes3D`** ([wm_frame_to_box/GetBoundingBoxes3D])

	Recois une depth Image et une liste de BoundingBoxes2D.
	- sara_msgs/BoundingBoxes2D boundingBoxes2D
	- sensor_msgs/Image Image
	
	Retourne une liste des BoundingBoxes3D
	- sara_msgs/BoundingBoxes3D boundingBoxes3D

#### Paramètres

* **`camera_angle_width`** (float, default: 0.785398163397)

	Largeur en radians de l'ouverture de la camera

* **`camera_angle_height`** (float, default: 0.785398163397)

	Hauteur en radians de l'ouverture de la camera

* **`camera_topic`** (string, default: "/head_xtion/depth/image_raw")

	Topic où aller chercher le depth de la camera

* **`camera_frame`** (string, default: "head_xtion_depth_frame")

	Frame tf de la camera

* **`base_frame`** (string, default: "base_link")

	Frame tf de référence pour le message des boites 3d

* **`yolo_topic`** (string, default: "/darknet_ros/bounding_boxes")

	Topic où aller chercher les cadres de darknet

* **`bounding_boxes_topic`** (string, default: "/frame_to_boxes/bounding_boxes")

	Topic où publier les boites 3d

* **`default_box_size`** (float, default: 0.1)

	Dimensions standards des boites
  

[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[darknet_ros]: https://github.com/leggedrobotics/darknet_ros
[opencv]: http://wiki.ros.org/opencv3
[readme template]: https://github.com/ethz-asl/ros_best_practices/blob/master/ros_package_template/README.md
