# Catch-Me-If-You-Can-
Employed i-Creates that litter and a Dagu car to simulate a real time chase acting Using Server-Client Architecture, two Cameras, Ultrasonic Sensors, and an assembled motor arm.

#Problem Definition and Solution

There are two problems we aim to tackle with this project. The first is the widespread problem of littering by people riding in vehicles. Our project will simulate the effective detection of littering and subsequent capture of the offending vehicle. The capturing process itself is going to be autonomous, due to the current trend of fully autonomous vehicles.. 

We want to solve the problem by chasing and stopping vehicles that litter using autonomous police vehicles. This is achieved with the help of several sensors that will collect information and process it in the central server using computer vision algorithms . All the vehicles and sensors should have a reliable synchronization in order to accurately simulate a chase.

#Project Description

We are planning to use two i-Creates, two robotic arms, a Dagu, and a camera. The i-Creates will be used to model cars, Dagu to model the police and a camera that scans the scene in which everything is taking place. To differentiate between the different i-Creates, each of them will have a colored paper placed on its surface. One of the i-Creates will throw litter on the ground, using an Arm. The camera that shall be placed above the entire scene, will have three main jobs:

●	It should detect if there is any litter thrown. The litter will have the same colour as the I-create that throws it and using computer vision, the camera will sense a change in the environment and detect that litter has been thrown on the ground.

●	The Camera then shall detect which I-Create threw the litter by using a certain computer vision algorithm.

●	It then shall sends a signal to the Dagu indicating which I-Create threw the litter and its location so it can move towards this I-Create and stop it.

●	There will be a designated safe zone which, if reached by the Icreate, the Dagu will not be able to stop it. 

●	Another camera will be placed on the Dagu, which will be used to display the Dagu’s perspective.  This will enable the audience to closely follow the movement and the process of capturing.


#Equipment

●	Two  I-Creates

●	One Dagu

●	Central camera

●	Two  Robotic Arms

●	One Udoo Boards

●	Three Panda Board

●	Camera on Dagu 

●	One 11.1 V -3 Cell for the Dagu Car

●	Four Power Bank for the Udoo Boards


#System Architecture

![alt tag](https://github.com/yehiahesham/Catch-Me-If-You-Can-/blob/master/2.png)

All of our project components will be synced together to act more realistically and effectively. We believe that the Server-Client Architecture is best suited for our project. We will have a PC or laptop acting as a centralized server, running the main code, and initiating  and controlling everything. The Camera Client should stream the video to the server where it is processed and analyzed. On the other hand, the Dagu car should wait till the ignition signal to go and catch a certain vehicle with a given speed and location, based on previous and continuous calculations did through the server to optimize the dagu car. Moreover, the iCreates robots and the Arms installed on them should also wait for the signal to either move or throw an object, which will be treated as if  trash  was thrown out of a car. At this point, the object and the responsible iCreate robot  should be already detected by the camera and the Dagu car will start the chase. Our Communication protocol will be a wireless TCP connections. Below is the System Architecture figure of  our project.
  

#Conclusion 
To conclude, we hope that through our project we have been able to demonstrate the system’s ability to detect and then direct the police car with a reasonable reaction time to capture efficiently and autonomously the vehicles that have littered, providing a miniature but viable model for future implementations.
