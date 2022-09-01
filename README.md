# Installation

## Prerequisites

1. install [ROS1](http://wiki.ros.org/ROS/Installation)
2. pip install sklearn scikit-image
3. xterm (for controlling the chair in simulation) - **sudo apt install xterm**

## Install the code
1. Clone the repository
2. Run ./setup.bash

# Current Status

## Functionality 
1. **What works** - USB Relay, FLIC button, Joystick, Haptic
2. **Code Complete** - Audio
3. **What partially works** - LIDAR, cameras, decision maker
4. **What doesn't work** - Phone interface, Obstacle classification

## Physical Setup (Raspberry, Jetson, Arduino) 

1. A Raspberry PI running the main control logic, the LIDAR, the Haptic feedback and the FLIC emergency stop button
2. An nVidia Jetson nano processing the images from four Realsense depth cameras and the audio output
3. An arduino controlling LED visual feedback

The system is based on ROS1 melodic (with the Raspberry as ROS_MASTER) and nodes in python2 (except for the LED component which is in C++ on the Arduino). 
The computers are connected directly by ethernet.

## USB Relay

Wizzy CAN reverse-engineered by cyber team. Works  

## Caretaker Emergency Stop Button 

Connected by bluetooth to the raspberry. One click locks, two clicks release. 
The module works with python3 and therefore does not run as a ROS node but independently.
If system is ported to noetic this can be included in main architecture.

### LEDs - *Idan*

works

### Haptic

works

### Audio 

noticeable latency. seems better on the jetson but not really tested yet

## Sensing

### LIDAR 

Partial. Seems to work for obstacles. Not tested enough for stairs

### Realsense 

Obstacles based on depth only worked last time it was checked (a long time ago). 

## Decision Making 
Not finished

# Simulation
In Gazebo

# Wizzybug Driver Assist
Proof of Concept Technical Specification 

**Note: Work in progress**

# Introduction

When a child begins walking, normally at age 9-16 months, they gain the power to explore, move around and impact their environment. For children with disabilities, it is not unusual to have to wait until the age of 5 years, due to the complicated process of learning to drive an electric wheelchair.

By connecting cameras and other sensors to the electric wheelchair and to the care-giver&#39;s application, we hope to close this gap and prevent the developmental deficits related to it, and to allow simple and safe usage of the electric wheelchair for very young children.

# ADAS for the Wizzybug Electric Wheelchair

## Feature Overview

Learning to drive the Wizzybug is hard. Starting at late age (such as age 5) delays the child&#39;s development. Technology can aid in speeding up the learning process, and increase safety.

The system should allow the child to control and move the chair, but simultaneously have the ability to signal and react if the chair is approaching an obstacle such as a person, a wall, or any other object in the wheelchair&#39;s path.

The signaling should occur in several domains, because the system interacts with children that have developmental disabilities: Visual, Audio and Haptic.

When the children first try to drive the electric wheelchair, they might bump into walls, toys and people around them. This is painful and problematic, especially when other handicapped children are involved (who mostly do not move fast enough to avoid the incoming wheelchair).

In case the sensory / vocal signal does not help, the system should apply resistance to the wheelchair problematic trajectory, or even bring the wheelchair to a full stop. Continuing the wheelchair&#39;s movement will be enabled only in an obstacle free direction.

## System Components

1. Four [RealSense 415](https://www.intelrealsense.com/depth-camera-d415/) depth cameras providing depth and RGB images at 30 frames per second
2. Inertial Measurement Unit, able to detect the chair&#39;s inclination
3. Controller joystick, modified to read out chair velocity commands
4. Chair motors – [how do we use them?]
5. LaserScan LIDAR providing a one-dimensional range map in front of the chair
6. [Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) as the system central computing module

## Software Architecture

The system components:

![](RackMultipart20200418-4-1yquz6t_html_5157ef40c0884e24.png)

_Figure 1: Software Architecture_

The system has four main functional components:

1. **Perception** -responsible for understanding the wheelchair&#39;s state within its environment and detection of objects surrounding it.
2. **Decision Making** - responsible for assessing the safety of the chair state and required measures to be taken to mitigate a dangerous situation if such an instance occurs.
3. **Control** - decrease wheelchair speed, or bring it to a complete stop, according to commands from the decision making module.
4. **Human** - **Machine Interface** – indicate the state of the wheelchair, appropriate warnings and required measures to be taken, to the child and attendant adult, using visual, audio and haptic cues.

The middleware software that integrates all the components is the [Robot Operating System](https://www.ros.org/) (ROS), which implements a Publisher-Subscriber architecture that enables synchronized data flow between the different components.

## Functionality

![](RackMultipart20200418-4-1yquz6t_html_4e5c19fa24b47f56.png)

_Figure 2: ADAS Functionality_

The system shall provide the following main functions:

1. Warn the child of a potential collision with an obstacle
2. Warn the parent / caretaker of a potential collision with an obstacle
3. Provide the parent / caretaker with the capability to remotely stop the chair

## Safety

1. The system is a proof of concept prototype to be used solely under adult attendant supervision.
2. The system shall not have any adverse safety effect on the wheelchair&#39;s mobility; it may only decrease velocity in case of obstacle collision avoidance. In all other cases the system is advisory.
3. The human machine interface shall not provide misleading safety information and shall not have any adverse effect on the child&#39;s cognitive capability to control the chair&#39;s movement.

# Component Design

## Perception

### Depth Cameras

Four depth and RGB images acquired from four [RealSense 415](https://www.intelrealsense.com/depth-camera-d415/) depth cameras at resolution of 640x480, at 30 frames per second.

1. Segment the depth images and extract objects within the warning zone. This is immediately passed to the Decision Making module in case immediate action is necessary
2. Classify the close objects using a Neural Network object detector, and pass this information to the Decision Maker (this might affect non-safety behavior, for example after an emergency stop)

### LIDAR

![](RackMultipart20200418-4-1yquz6t_html_70be395d689f1a0.png)Based on [RpLidar type A2](https://www.slamtec.com/en/Lidar/A2). The Lidar transits 360 measurements per cycle, at the speed of 5Hz.

The Lidar is placed perpendicular to the floor, enabling to identify walls and drops in the floor.

### IMU

Reads the Wizzybug inclination and passes on to DM.

### _Figure 3: Lidar scanning the front of the Wheelchair path_Joystick

From the joystick readings we get a reading of the chair velocity. This is used by the Decision Making module to estimate Time to Collision (TTC) with detected obstacles.

## Decision Making

Algorithm Flow Chart:

_ **TBD** _

## Control

The Control module activates Numato Lab&#39;s 1 Channel USB Powered Relay Module. Activates DX Control Power Module to enable and disable the Wizzybug&#39;s motors.

![](RackMultipart20200418-4-1yquz6t_html_c594f9b2cd0aa973.png)

_Figure 4: USB Controlled relay_

The USB Relay control the [DX2 Power Module](https://www.dynamiccontrols.com/sites/default/files/2018-05/dx2-pma-installation-manual-issue-1.pdf) of the Wheelchair:

![](RackMultipart20200418-4-1yquz6t_html_d631c1d651db9f0b.png)

_Figure 5: Power Module controlling the Motors_

![](RackMultipart20200418-4-1yquz6t_html_664377a4dd50e549.png)

_Figure 6: Controlling Speed Limit using DCI Connectors in the Power Module_

## HMI

The system has two main users: the child and the caregiver. The aim of the design is to provide both types of users a set of tools to assess the **situation awareness** of the system at any given moment. The bug senses its environment and identifies the obstacles around it.

**Educate &amp; Protect** - The design enables the child experiment with her environment, but also protects her when needed. We assume two operation areas. These are shown by figure 7. In the first, the inner area (marked in grey), the child is in control. The system should notify her when she approaches an obstacle and the notification should become more alerting as she approaches a problematic obstacle, but the child is in control over the bug. In the second, outer area (marked in white), control is taken from the child. The bug is now in control.

**Privacy** - Another principle that the design wishes to promote is the one of Privacy **.** We do not want to impair to adorability of the bug by making it noisy and thereby deter other kids from approaching the child. The system should provide the child the required situation awareness by haptic stimulation in a private manner. Sound feedback (using earcons or auditory icons) and spoken instructions should be used in an appropriate manner, and only when needed. The caregiver will be notified using a visual feedback on a ring of LED, and by the sound and speech delivered to the child.

Both types of feedback can be turned off in the system&#39;s set of preferences.

![](RackMultipart20200418-4-1yquz6t_html_ee82adcc9a0a5e8a.png)_Figure 7. Areas of control_

**Obstacles classification scheme** - The classification scheme refers to three types of obstacles, each of which is defined by the potential harm to the child or the environment:

Type A events - still objects, shorter than 4-5 cm (e.g., soft or hard toys, doorstep, etc.). The bug should **inform** that it senses these types of obstacles but shouldn&#39;t stop before them. The bug could drive over them. Since one cannot distinguish between the caregiver and other grown-ups, all adults are classified as A.

Type B events – These obstacles do not impose a serious harm to the child, but the bug cannot drive over them or through them (e.g., doorframes, walls, ascending staircase, static objects that are higher than 5 cm). The bug should **motivate the child to stop** , by sending notifications with an increasing sense of urgency as one approaches the obstacle. If the child does not stop the bug, the bug should stop driving just before the obstacle (50 cm will allow the child to change the bug&#39;s position in order to pass through a door frame in a smoother move). Experimentation - The **Stop and go mechanism** will enable the child to continue driving toward the obstacle, if child insists to do so. The purpose of this feature is to teach the child what can, and cannot be done with the bug, yet support her wish to experiment with the world.

Type C events – These obstacles impose harm to the child or the environment (e.g., descending staircase, Ramp, Glass wall, wall with adjacent tall obstacle that impose the danger of flip over, babies, children, animals). **The stop &amp; guide/limit control** feature makes the bug stop, when type C objects are sensed (50 cm will allow the child to change the bug&#39;s position). However, as opposed to B events, in type C events we cannot allow the child to experiment with the environment in a free manner. In this case, the bug will not be able to proceed toward the dangerous area; it will be able to maneuver only to the directions that are defined as non-risky.

Type U- Unknowns – Assuming that the system fails to classify an obstacle that has the potential to harm the child as type C obstacles do, the feedback should appear as C

### Multimodal feedback

The set of feedback consists of **synchronized multimodal visual-haptic** feedback that will give the child a sense of directionality for the obstacles and their level of threat, plus sound alerts when needed.

### Haptic Feedback

Vibration alerting is a great alternative to lights and beeps. By mounting them on the body, the child&#39;s eyes are free, and the child can drive toward her areas of interest. The motor needs to be strong enough to overcome the damping of soft materials (nappies, coats and jumpers). This can be aided by ensuring the vibrations are directed towards the user. In our system, we will be used coin motors.

Haptic stimulation will be given by actuators creating vibration. These will be placed along the upper seatbelt and the center straps to provide a sense of **directionality** for rear and front objects notification.

![](RackMultipart20200418-4-1yquz6t_html_3d4bd2802683a686.gif) ![](RackMultipart20200418-4-1yquz6t_html_d8f55e6783427fd9.gif) ![](RackMultipart20200418-4-1yquz6t_html_4f75c41289854de.gif) ![](RackMultipart20200418-4-1yquz6t_html_103eae3757b4f709.gif) ![](RackMultipart20200418-4-1yquz6t_html_b4e0bcc86756d932.gif) ![](RackMultipart20200418-4-1yquz6t_html_9bc222f1ea5207ff.gif) ![](RackMultipart20200418-4-1yquz6t_html_7667d6dafba48ea4.gif) ![](RackMultipart20200418-4-1yquz6t_html_7f10f04082fc8d07.gif) ![](RackMultipart20200418-4-1yquz6t_html_1c5947eb9498832a.gif) ![](RackMultipart20200418-4-1yquz6t_html_e71570fc7201651.gif) ![](RackMultipart20200418-4-1yquz6t_html_79ff96936be9a86e.gif) ![](RackMultipart20200418-4-1yquz6t_html_70e553adec9fab92.gif) ![](RackMultipart20200418-4-1yquz6t_html_cbe401c55c8be3e3.gif) ![](RackMultipart20200418-4-1yquz6t_html_70e553adec9fab92.gif)

Front

Rear

Left/Rear

Left/Front

Right/Front

Right/Rear

 ![](RackMultipart20200418-4-1yquz6t_html_8808236d13c73bfe.jpg)

_Figure 8. Placement of Haptic Actuators_

The **potential urgency** of the event will be conveyed by manipulating the following features:

- Intensity of the pulse
- Pulse duration
- Period (blinking pace)
- Number of repetitions

High intensity pulse with sharp attack and decay of the stimuli envelope, short pulses with short intervals and high number of repetitions will give the user a high sense of urgency.

### Visual feedback

Visual feedback will be given by marking areas on a LED ring. A 24 LED ring placed around the joystick will be used to mark 6 areas, 4 LED lights per area: Front, Right/Front, Right/Rear, Rear, Left/Rear, and Left/Front.

![](RackMultipart20200418-4-1yquz6t_html_6787c004c8d04932.png) ![](RackMultipart20200418-4-1yquz6t_html_ad4ca7bfe3fde376.png)

_Figure 9. Visual Feedback via LED ring display_

![](RackMultipart20200418-4-1yquz6t_html_8ab525ccf5b9b749.gif) ![](RackMultipart20200418-4-1yquz6t_html_d9e22eb2ee070bcf.gif) ![](RackMultipart20200418-4-1yquz6t_html_febb2caa0c968ece.gif) ![](RackMultipart20200418-4-1yquz6t_html_eb4b35a27a396e.gif) ![](RackMultipart20200418-4-1yquz6t_html_4fb6524d35685fab.gif) ![](RackMultipart20200418-4-1yquz6t_html_66b617cf18579911.gif) ![](RackMultipart20200418-4-1yquz6t_html_70e553adec9fab92.gif) ![](RackMultipart20200418-4-1yquz6t_html_fe83b11a5eb0bf9f.gif) ![](RackMultipart20200418-4-1yquz6t_html_ceb65ce603843801.gif) ![](RackMultipart20200418-4-1yquz6t_html_5328ede5b8efb1ed.gif)

Left/Rear

Right/Rear

Rear

FRONT

Each event category is assigned with a different color. The traffic light metaphor was chosen for simplicity:

T ![](RackMultipart20200418-4-1yquz6t_html_fcd6ee5b6b3aa035.png)
 ype A is marked by Green – The bug can drive over the obstacle

_Figure 10. Visual Feedback for Type A events_

Type B is marked by Yellow – The bug can drive up to a point. No control in the next cycle.

![](RackMultipart20200418-4-1yquz6t_html_36b54121aaa5411b.png)Type C is marked by RED – The bug can drive up to a point. Control over the driving direction in the next cycle

_Figure 11. Visual Feedback for Type B &amp; C events_

The **potential urgency** of the event will be conveyed by manipulating the following features:

- Intensity of the pulse
- Pulse duration
- Period (blinking pace)
- Number of repetitions

Same as described above for the haptic stimuli, high intensity pulse with sharp attack and decay of the stimuli envelope, short pulses with short intervals and high number of repetitions will give the user a high sense of urgency.

![](RackMultipart20200418-4-1yquz6t_html_4af81d5da64bff0f.png)

_Figure 12. Visual Feedback_

### Sound alerts

Sound alerts will play when the bug stops in type B &amp; C/U events.

Type B - Mild alert will be given while stopping in B events (see Figure 7). The sound would give the child a very subtle sense of bump with the &quot;invisible wall&quot; that made the bug stop.

Type C - In risky scenarios (C), an intensive sound should play just before the bug stops. The sound file will be selectable by the caregiver who could choose between an urgent alert s Vs. a more softer alert sound (preference settings)

Verbal instructions will play after the car stopped to provide the child maneuvering guidance (should be tailored to front/back alerts).

## Caretaker Remote Application

The caretaker application is run on a mobile phone and provides notifications, connection monitoring and emergency remote stop of the chair.

## Remote Notification

The system shall notify the caretaker of imminent collision with obstacles; the same notifications the child receives (see section ‎3.4) shall be broadcast to the caretaker&#39;s phone and induce:

1. Audible Notification
2. Vibration
3. Visual Notification that includes the direction of the object relative to the chair

## Remote Emergency Stop

The caretaker application shall provide the capability to remotely stop the chair.

## Remote Application Connection Status

The application shall indicate the status of the remote connection to the vehicle and notify the caretaker when there is no connection.
