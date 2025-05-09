# Gesture-Controlled Quadruped in Gazebo with ROS2

## Team Members:

- Nageswara Rao Janapareddy ,njanapar@buffalo.edu

## Project Objective
The goal of this project is to design and simulate a Quadruped-like robot in Gazebo using ROS2, which can be controlled using full-body gestures (arms and elbow joints). The robot will be controlled by specific gestures.

## Contributions
This project introduces a simple gesture control system for a robot. It focuses entirely on translating specific human gestures into robot movements. The system will offer a hands-on, interactive way to control the robot using full-body gestures.

Controlling legged robots like Boston Dynamics' Spot is both a challenging and fascinating robotics problem. This project demonstrates how to simulate a Spot-like quadruped in Gazebo and control it using ROS 2. It includes:

  URDF-based robot model with 12 DOF (3 joints per leg).

  Gait controllers for walking, crawling, and standing.

  Teleoperation via gesture control (MediaPipe).

## Demonstration

![alt text](https://github.com/[njanapar]/[quadruped]/blob/[main]/sim1.jpg?raw=true)
![alt text](https://github.com/[njanapar]/[quadruped]/blob/[main]/sim2.jpg?raw=true)
![alt text](https://github.com/[njanapar]/[quadruped]/blob/[main]/sim3.jpg?raw=true)

## Installation Instructions

This project is tested in ROS 2 Jazzy and Gazebo Harmonic. The installation links for both of them are provided below

    1. https://docs.ros.org/en/jazzy/Installation.html
    2. https://gazebosim.org/docs/latest/ros_installation/

And other important packages required are provided in requirements.txt file and can be installed using following command line

    pip install -r requirements.txt


## How to Run the Code

Clone this repository into your home folder. And then build the workspace using following command and source it

    
    ```
    colcon build
    source install/setup.bash
    ```

    And then we can launch file required to launch the simulation and start the controllers:

    ```
    ros2 launch spot_control description.launch.py
    ```

    Then we need to run the file required to control the robot using gestures in another terminal:
    ```
    ros2 run spotrobot_control test_joy.py
    ```

## References:
```
M. H. Zafar, E. F. Langås and F. Sanfilippo, "Real-Time Gesture-Based Control of a Quadruped Robot Using a Stacked Convolutional Bi-Long Short-Term Memory    (Bi-LSTM) Neural Network," 2024 10th International Conference on Automation, Robotics and Applications (ICARA), Athens, Greece, 2024, pp. 81-86, doi: 10.1109/ICARA60736.2024.10553163.
https://medium.com/swlh/training-a-spot-inspired-quadruped-robot-using-reinforcement-learning-678b9e5df164
B. Ozkaynak and B. Ugurlu, "Preview Control-based Jumping and Spot-Jogging Trajectory Generation for Quadruped Robots," 2023 IEEE 21st International Conference on Industrial Informatics (INDIN), Lemgo, Germany, 2023, pp. 1-6, doi: 10.1109/INDIN51400.2023.10217866.
https://github.com/OpenQuadruped/spot_mini_mini
https://github.com/curieuxjy/Awesome_Quadrupedal_Robots
https://docs.ros.org/en/jazzy/p/ros_gz_bridge/
https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html
https://github.com/ros-controls/ros2_controllers
```

## Future Work:




