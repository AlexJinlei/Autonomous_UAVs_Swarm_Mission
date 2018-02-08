# Autonomous UAVs Swarm Mission
## 1. DESCRIPTON
This is a autonomous quadcopter swarm project. The purpose of this project is to achieve a designated mission by multiple drone coorperation. I developed and assembled two sets of drone as the hardware platform. One set consists of six drones modified from 3DR IRIS+ drone, the other set consists of four drones made from scratch. I also developed a drone swarm control software package, including the functions of flight control, multi-drone communication, multi-drone coordination, stereo vision, failure handle, and so on.

There are three subprojects in this repository. The Drone_Matrix_Formation_Flight is the earliest developed subprojects, which acheives four drones formation flight. Four dones can form any formation in flight. The Drone_Matrix_Curvature_Flight subject is developed based on Drone_Matrix_Formation_Flight. This is a improved version of Drone_Matrix_Formation_Flight, in which the functions are modularized and more functions are added. The drone troop achieves curved path flight by specified curvature radius and degrees. A execution queue is implemented to achieve complex path designation. 

The leader drone send real time commands to all follower drones, and the follower drones form a specified shape while fly following the leader drone. 
