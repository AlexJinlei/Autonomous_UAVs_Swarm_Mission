# Autonomous UAVs Swarm Mission
## 1. DESCRIPTON
This is an autonomous quadcopter swarm project. The purpose of this project is to achieve a designated mission by multiple drone cooperation. I developed and assembled two sets of the drone as the hardware platform. One set consists of six drones modified from 3DR IRIS+ drone, the other set consists of four drones made from scratch. I also developed a drone swarm control software package, including the functions of flight control, multi-drone communication, multi-drone coordination, stereo vision, failure handle, and so on.

There are three subprojects in this repository. 1) The Drone_Matrix_Formation_Flight is the earliest developed subproject, which achieves four drones formation flight. Four drones can form any formation dynamically in flight. This drone swarm can be easily expanded, i.e., you can add more drones to this troop. 2) The Drone_Matrix_Curvature_Flight subject is developed based on Drone_Matrix_Formation_Flight. This is an improved version of Drone_Matrix_Formation_Flight, in which the functions are modularized and more functions are added. The drone troop achieves curved path flight by specified curvature radius and degrees. An execution queue is implemented to achieve complex path designation. 3) Drone_Matrix_Balloon_Hunter is the latest developed subproject. It is developed based on Drone_Matrix_Curvature_Flight, with lots of more function added. In this project, four drones cooperate to find and destroy four balloons. The four balloons are in four different colors, and each drone is intended to find the ballon with specified color and then destroy it. The advanced stereo vision camera plus sonar system is designed and equipped on each drone. With this stereo vision system, the drone is able to find the ballon and calculate the distance of the ballon. Then the drone can adjust its heading and altitude to lock the balloon.

For all the three subprojects, there is one drone acts as a leader (commander), and the other drones act as followers (soldiers). The leader drone plans the mission and sends real-time commands to all the follower drones, and the follower drones carry out required actions following the leader's instructions. All the drones in the troop are identical, which makes the decentralized swarm structure feasible. With a proper negotiation strategy, it is easy to reassign the leader drone. The mission is still achievable by other drones even if the leader fails.

## 2. HARDWARE CONFIGURATION
### 1) DRONE SWARM SET I: MODIFIED 3DR IRIS+ DRONE
#### a) MECHANICAL SYSTEM
This Drone platform is modified from 3DR iris+ drone, and the mechanical system is unchange. Pleas refer to the official [3DR Operation Manual](Hardware_Configuration/3DR_IRIS/IRIS-Plus-Operation-Manual-vH-web.pdf) for the hardware details. Following is the basic hardware spacification:
- UAV type: QuadCopter
- Frame type: V
- Motors: 950 kV
- Propellers: 9.5 x 4.5 T-Motor multirotor self-tightening counterclockwise (2), 9.5 x 4.5 T-Motor multirotor self-tightening clockwise (2)
- Battery: 3S 5.1 Ah 8C lithium polymer

#### b) AUTOPILOT SYSTEM
The autopilot system shipped with iris+ drone is Pixhawk v2.4.5 with firmware ArduCopter v3.2. I upgraded the firmeare to ArduCopter v3.3.3. All the software packages are tested with ArduCopter v3.3.3, and are guaranteed to work well. 
- Autopilot: Pixhawk v2.4.5
- Firmware: ArduCopter v3.3.3 (upgraded from original v3.2)

#### c) POSITIONING SYSTEM
The 3DR iris+ drone is shipped with GPS. This project uses its original GPS.
- GPS: 3DR uBlox GPS with Compass (LEA-6H module, 5 Hz update)

#### d) ON BOARD COMPUTING SYSTEM (COMPANION COMPUTER)
This project need a high performance companion computer to do mission planning, computer vision task, and other computing-intensive tasks. However, the original iris+ done does not have a companion computer. After research, I found that the intel UP board is the best companion computer for this project.
- Single board computer: Intel UP board
- Processor: Intel® Atom™ x5-Z8350 Processor (2M Cache, up to 1.92 GHz) CPU with 64 bit architecture; Quad Core CPU
- Memory: 4GB DDR3L-1600
- Storage Capacity: 32 GB eMMC
- OS: Ubilinux
 #### e) STEREO VISION SYSTEM
 


