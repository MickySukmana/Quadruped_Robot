# Quadruped_Robot
Quadruped Robot using arduino uno and ESP8266. the robot is programmed to sit, stand, move forward and backward.  

there's two version of program in this repo. the one who move forward and stop after certain amount of steps and the one that can be controlled using blynk apps.  

for the controlled robot, i used arduino uno as receiver to move the robot legs using inverse kinematics. to control the robot i used ESP8266 to receive a string command from blynk apps and transmit it to receiver using UART.

![alt text](https://github.com/MickySukmana/Quadruped_Robot/blob/main/img/1.jpg=250x250)
![alt text](https://github.com/MickySukmana/Quadruped_Robot/blob/main/img/2.jpg?raw=true)
![alt text](https://github.com/MickySukmana/Quadruped_Robot/blob/main/img/3.jpg?raw=true)
