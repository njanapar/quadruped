# Gesture-Controlled Quadruped in Gazebo with ROS2

## Team Members
- Nageswara Rao Janapareddy, njanapar@buffalo.edu

## Project Objective
This project aims to design and simulate a quadruped robot, inspired by Boston Dynamics' Spot, in Gazebo using ROS2. The robot is controlled using full-body gestures (focusing on arm and elbow joint movements) detected via MediaPipe. The system translates specific human gestures into robot movements, providing an interactive and intuitive control interface.

## Contributions
This project introduces a novel gesture-based control system for a quadruped robot, emphasizing:
- A URDF-based robot model with 12 degrees of freedom (3 joints per leg).
- Gait controllers for walking, crawling, and standing.
- Real-time teleoperation using gesture recognition powered by MediaPipe.
- A hands-on demonstration of controlling a Spot-like quadruped in a simulated Gazebo environment.

The project tackles the challenging problem of legged robot control, offering a foundation for further research in human-robot interaction and robotic locomotion.

## Demonstration
Below are screenshots of the quadruped simulation in Gazebo:



![Simulation 1](https://github.com/njanapar/quadruped/blob/main/imgs/sim1.png?raw=true)
![Simulation 2](https://github.com/njanapar/quadruped/blob/main/imgs/sim2.png?raw=true)
![Simulation 3](https://github.com/njanapar/quadruped/blob/main/imgs/sim3.png?raw=true)


*Note: Update the GitHub username (`njanapar`) and repository name (`quadruped`) in the image URLs if they differ.*

## Installation Instructions
This project is tested with **ROS2 Jazzy** and **Gazebo Harmonic**. Follow these steps to set up the environment:

1. Install ROS2 Jazzy: [Official ROS2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html)
2. Install Gazebo Harmonic: [Gazebo ROS Installation Guide](https://gazebosim.org/docs/latest/ros_installation/)
3. Install required Python packages:
   ```bash
   pip install -r requirements.txt

## How to Run the Code
1. Clone the repository to your home directory:
   ```bash
   git clone https://github.com/njanapar/quadruped.git


2.Build the workspace 
   Clone the repository and navigate to the project directory. Then, build the workspace with the following commands:

   ```bash
   colcon build
   source install/setup.bash
  ```


3. Launch the Simulation and Start the Controllers

After the workspace is built, launch the simulation and the necessary controllers:

```bash
ros2 launch spot_control description.launch.py
```

4. Run the Gesture Control

Open another terminal and run the gesture control file to start controlling the robot with gestures:

```bash
ros2 run spotrobot_control test_joy.py
```
### Gesture Control Logic

The robot is controlled by specific gestures of the user's arms, as follows:

- **Right hand is up and not bent**: Robot moves **forward**.
- **Right hand is up and bent**: Robot turns **left**.
- **Left hand is up and not bent**: Robot moves **backward**.
- **Left hand is up and bent**: Robot turns **right**.

## References:

1. M. H. Zafar, E. F. Langås, and F. Sanfilippo, "Real-Time Gesture-Based Control of a Quadruped Robot Using a Stacked Convolutional Bi-Long Short-Term Memory (Bi-LSTM) Neural Network," 2024 10th International Conference on Automation, Robotics and Applications (ICARA), Athens, Greece, 2024, pp. 81-86, doi: [10.1109/ICARA60736.2024.10553163](https://doi.org/10.1109/ICARA60736.2024.10553163).

2. [Training a Spot-inspired Quadruped Robot Using Reinforcement Learning](https://medium.com/swlh/training-a-spot-inspired-quadruped-robot-using-reinforcement-learning-678b9e5df164)

3. B. Ozkaynak and B. Ugurlu, "Preview Control-based Jumping and Spot-Jogging Trajectory Generation for Quadruped Robots," 2023 IEEE 21st International Conference on Industrial Informatics (INDIN), Lemgo, Germany, 2023, pp. 1-6, doi: [10.1109/INDIN51400.2023.10217866](https://doi.org/10.1109/INDIN51400.2023.10217866).

4. [OpenQuadruped GitHub Repository](https://github.com/OpenQuadruped/spot_mini_mini)

5. [Awesome Quadrupedal Robots GitHub Repository](https://github.com/curieuxjy/Awesome_Quadrupedal_Robots)

6. [ROS-Gazebo Bridge Documentation](https://docs.ros.org/en/jazzy/p/ros_gz_bridge/)

7. [ROS2 Gazebo Tutorial](https://docs.ros.org/en/jazzy/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

8. [ROS2 Controllers Documentation](https://github.com/ros-controls/ros2_controllers)

### Future Work

To improve the smoothness and consistency of the robot's movements, I plan to enhance the gait control algorithms. By implementing advanced **motion planning**, **trajectory interpolation**, and **path smoothing**, I aim to ensure seamless transitions between gaits and steady movement.

I will also focus on improving **footstep planning** by dynamically adjusting foot placements based on the robot's real-time state, and incorporate **inverse kinematics** and **predictive motion planning** to optimize step trajectories.

Additionally, I will integrate **feedback loops** from onboard sensors to allow real-time adjustments based on environmental conditions, ensuring stable movement across varying terrains.

### Project Explanation

You can watch the detailed explanation of the project on [YouTube](https://youtu.be/puTWiiMsdBg).

