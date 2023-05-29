
# 1. About the package <a name="atp"></a>
 **Course:**  Multidisciplinary Project (RO47007) \
 **Program:** Msc Robotics @ TU Delft            
 **Developer Group:**  Group 19 - SpotOnCare                     
 **Last Maintained Date:**  25.05.2023   


This repository is part the submission for the project of the course **Multidisciplinary Project (RO47007)**, in collaboration with  [TNO](https://www.tno.nl/en?gclid=.CjwKCAjw1MajBhAcEiwAagW9MSsTkBs0QeVZAyaxq9Fz1mtmGNJCkYzUVTuIwKk3bHhMCr6WwW6XnhoCvmsQAvD_BwE).


The package provides the necessary code to build and run the yolo detecion code. 

  
It contains all necessary files to build the one ROS node, namely: 
- ``detection`` 

This package can be used in combination with the other ROS packages contained in the parent repository ``champ_spot`` to simulate and run autonomous missions designed for the healthcare sector on a [Boston Dynamics SPOT robot](https://www.bostondynamics.com/products/spot).

--- 

# Table of Contents

1.  [About the package](#atp) \
    1.1 [ROS-Node detection_node](#r1)

2. [Getting Started](#gs)\
    2.1 [Prerequisites](#pr)\
    2.2 [Installation](#i)

3. [Usage](#u)\
    3.1 [Running the detection_node](#rbrac)
    

---

<p>&nbsp;</p>




## ROS-Node detection_node <a name="r1"></a>
The package contains all the necessary files to build and run ``detection`` that:
- Detects persons and then uses point-cloud data to assign a 3d location to the person, this data is then published.
- Detects persons and objects and draws bounding boxes around them. The images with bounding boxes are published.


# 2. Getting Started <a name="gs"></a>
## Prerequisites <a name="pr"></a>
This project was developed and tested on a Ubuntu 20.04 LTS machine running ROS Noetic. The following steps will guide you through the process of setting up the workspace and running the project.


## Installation <a name="i"></a>
**1. CHAMP installation**

If you have not done so yet, follow the instructions to setup the CHAMP workspace on your machine: 
[CHAMP Installation](https://gitlab.tudelft.nl/cor/ro47007/2023/team-19/champ_spot). This will guide you through the installation of our version of the CHAMP repository containing all the necessary packages to run the project.


**2. Install the following dependencies:**

```python
cv2
numpy
os
tf
rospkg
ros_numpy
cv_bridge
```

**3. Configure yolo**

The weights for the model can be downloaded inside of yolo_config:

```bash
cd yolo_config
wget https://pjreddie.com/media/files/yolov3.weights
```

Depending on your system you might want to change the configuration of the yolo system used, for example you might want to use 3 channels for rgb images instead of 1. In this case you can edit the cfg file inside of the yolo_config folder.


# 3. Usage <a name="u"></a>
## Running the detection_node <a name="rbrac"></a>

After building the packages and sourcing your workspace (follow all the steps in **Getting started**) the code can be run using: 

```bash
rosrun yolo yolo_detection.py 2> >(grep -v TF_REPEATED_DATA buffer_core)
```

The part behind yolo_detection.py makes sure terminal is not flooded with warnings from tf (tf has a known issue that is non-critical)

