# Gesture-Controlled Quadruped in Gazebo with ROS2

## Team Members:

- Nageswara Rao Janapareddy ,njanapar@buffalo.edu

## Project Objective
The goal of this project is to design and simulate a Quadruped-like robot in Gazebo using ROS2, which can be controlled using full-body gestures (arms, legs, and elbow joints). The robot will be controlled by specific gestures.

## Contributions
This project introduces a simple gesture control system for a robot. It focuses entirely on translating specific human gestures into robot movements. The system will offer a hands-on, interactive way to control the robot using full-body gestures.

## Project Plan

### Set up the Environment:
- Install ROS2 and Gazebo.
- Create a basic workspace for the simulation.
- Set up a simple indoor environment in Gazebo for the robot to navigate.

### Create Gesture-Control Map:
- Define and document gestures that will control the robot’s movements (e.g., hand raised = move forward, arm outstretched = turn left).
- Create a gesture-to-movement mapping document to outline which gestures correspond to which actions.

### Robot Movement Algorithms:
- Write algorithms that map the gestures to specific robot movements in Gazebo.
- Develop basic code that allows the robot to respond to gestures (e.g., move forward, stop, or turn).

### Testing and Refining:
- Test the robot’s movement in response to gestures within the Gazebo simulation.
- Refine the gesture recognition and movement mapping to ensure smooth operation.

## Milestones/Schedule Checklist

- **Complete Proposal Document**  
  - **Due Date:** Feb. 28  
  - **Tasks:** Document objectives, resources, project plan, and schedule.

- **Set Up Development Environment**  
  
  - **Due Date:** March 5  
  - **Tasks:** Install ROS2 and Gazebo, create a basic workspace, set up robot model environment.

- **Create Gesture-Control Map**  
  
  - **Due Date:** March 17  
  - **Tasks:** Define and map gestures (e.g., raised hand = move forward, arm out = turn).

- **Write Robot Movement Algorithms**  
   
  - **Due Date:** march 28
  - **Tasks:** Implement algorithms to map gestures to robot movements.

- **Test Robot Movement in Gazebo**  
  
  - **Due Date:** april 1 
  - **Tasks:** Test the robot's response to gestures and refine the control algorithms.

- **Create Progress Report**  
  - **Due Date:** April 3  
  - **Tasks:** Document project status, highlight progress and challenges.

- **Create Final Presentation**  
  - **Due Date:** May 6  
  - **Tasks:** Prepare final presentation for demonstrating the project’s achievements.

- **Update Documentation Based on Feedback**  
  - **Due Date:** May 10  
  - **Tasks:** Revise the project documentation as per the feedback from the presentation.

- **Provide System Documentation (README.md)**  
  - **Due Date:** May 13  
  - **Tasks:** Complete system documentation detailing setup, usage instructions, and troubleshooting.

## Measures of Success
- The robot should respond accurately to specific gestures (e.g., raising a hand moves the robot forward).
- The robot should function smoothly in the Gazebo environment with gesture-based control.
- A classmate should be able to follow the instructions in the README and successfully run the simulation without assistance.


# **MidTerm project review
# **Quadruped Robot Gesture Control in Gazebo with ROS2**  

## **Overview**  
This project aims to design and simulate a quadruped-like robot in **Gazebo** using **ROS2**, controlled via **full-body gestures** (arms, legs, and elbow joints). The robot’s movements will be dictated by predefined gestures, offering a **hands-free and interactive control system**.  

## **Project Objective**  
- Simulate a **quadruped robot** in Gazebo using ROS2.  
- Enable basic movement control using a controller.  
- Implement **gesture-based control** for the quadruped robot, allowing it to respond to human gestures in real-time.  
- Establish seamless communication between **ROS2 and Gazebo** for controlling the robot’s movement.  

## **Current Progress**  
**Quadruped Robot Simulation:** Successfully simulated a quadruped robot in **Gazebo Harmonic** with **ROS2 Jazzy**.  
**Basic Control Implementation:** The robot can be minimally controlled using a Python-based GUI controller.  
**Gesture Control System:** Developed a gesture-based control system for a **four-wheeled robot** in **Gazebo**.  
**ROS-Gazebo Communication:** Integrated `ros_gz_bridge` for data exchange between **ROS2** and **Gazebo**.  

## **Upcoming Tasks**  
Improve the movement and control of the quadruped robot.  
Integrate the gesture control system with the quadruped robot.  

## **Project Implementation**  

### **1. Environment Setup**  
- Installed **ROS2 Jazzy** and **Gazebo Harmonic**.  
- Created a ROS2 workspace for the simulation.  
- Set up a **custom environment** in Gazebo for robot navigation.  

### **2. Robot Simulation & Control**  
- Loaded **URDF files** into Gazebo using ROS2.  
- Configured **ROS2 controllers** within URDF and ROS launch files.  
- Used `ros_gz_bridge` for communication between **Gazebo** and **ROS2**.  
- Developed a **Python-based GUI controller** for manual control of the quadruped robot.  

### **3. Gesture Control System**  
- Defined a **gesture-to-movement mapping** for controlling the robot.  
- Implemented a **gesture recognition system** (currently tested on a four-wheeled robot in Gazebo).  
- Created a **ROS2 node** to interpret gesture inputs and translate them into robot commands.  

### **4. Gesture-to-Movement Mapping**  
| **Gesture**           | **Action**           |  
|-----------------------|---------------------|  
| Right hand up        | Move forward        |  
| Left hand up         | Move backward       |  
| Both hands up/down   | Stop                |  
| Right hand bent      | Turn right          |  
| Left hand bent       | Turn left           |  

## **Technologies & Tools Used**  
- **ROS2 Jazzy** – Robot Operating System for control and simulation.  
- **Gazebo Harmonic** – Simulation environment for the quadruped robot.  
- **Python** – Implementing gesture recognition and GUI control.  
- **ros_gz_bridge** – Communication between ROS2 and Gazebo.  
- **URDF/Xacro** – Defining the robot model and control mechanisms.  

