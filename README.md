# TurtleBot3

## Description:

"This package about Turtlebot."


---

**Installation:**
- Before uploading the project file, we need to source the ros version we are working with to the terminal we are working with.

        source /opt/ros/$ROS_DISTRO/setup.bash


- At first, first we create a workspace and create the src folder under Workspace.

        mkdir -p ros2_ws/src
        cd ros2ws/src


- After, Download this package from github in the link under src folder.

    So clone this [here](https://github.com/epikrobotiksoftware/turtlebot3.git)

        https://github.com/epikrobotiksoftware/turtlebot3.git

- Go your workspace  

- After downloading the project from Github, we can use rosdep to download the dependencies of the project.

        sudo rosdep init
        rosdep update
        rosdep install --from-paths src -y --ignore-src

- After installing the package dependencies, we can build our package.

        colcon build
        source install/setup.bash

---
**Usage:**

- To run the project file 

    ros2 launch turtlebot3_bringup turtlebot3.launch.py 


---


## Contact:

### [Github: Mehmet KUTAN](https://github.com/MehmetKUTAN)

### [Linkedin: Mehmet KUTAN](https://www.linkedin.com/in/mehmet-kutan-664591151/)

### [Github: EPİK ROBOTİK](https://github.com/epikrobotiksoftware)

### [Linkedin: EPİK ROBOTİK](https://www.linkedin.com/company/epi%CC%87k-roboti%CC%87k/)
