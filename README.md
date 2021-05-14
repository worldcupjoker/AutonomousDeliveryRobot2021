AutonomousDeliveryRobot2021
 
In this project, the goal was to develop an autonomous delivery robot that is capable of navigating in a 7’ x 7’ simulated service map with walls around the perimeter, picking up orders from two different vendors and then safely delivering the food to the customer. Instead of traveling on sidewalks and crossing streets as an actual delivery robot, which requires vision systems and multiple sensors, the developed robot relies on distance sensors to autonomously navigate through virtual grid of 12” x 12” blocks with assigned position codes (see Figure below). A color sensor simulates the food loading mechanism, during the five seconds the robot stops in a 4” color circle simulating a vendor, the measured color is printed in a LED (Light-Emitting Diode) RGB (Red Green Blue color model) strip.
 
![image](https://user-images.githubusercontent.com/73008183/118209662-09781e00-b437-11eb-9f14-e3b45c8cc95a.png)
 
The final robot design is presented below.
 
![image](https://user-images.githubusercontent.com/73008183/118210051-d84c1d80-b437-11eb-8cc5-a4171b2bda47.png)
 
![image](https://user-images.githubusercontent.com/73008183/118210070-e13cef00-b437-11eb-8c40-3561aeda1bf4.png)
 
The robot plans its own route while it is in motion after each turns, detects and avoids obstacles that block its path, and redesign its own route avoiding the obstacles.
