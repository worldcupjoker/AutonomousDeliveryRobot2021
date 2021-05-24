AutonomousDeliveryRobot2021
 
In this project, the goal was to develop an autonomous delivery robot that is capable of navigating in a 7’ x 7’ simulated service map with walls around the perimeter, picking up orders from two different vendors and then safely delivering the food to the customer. Instead of traveling on sidewalks and crossing streets as an actual delivery robot, which requires vision systems and multiple sensors, the developed robot relies on distance sensors to autonomously navigate through virtual grid of 12” x 12” blocks with assigned position codes (see Figure below). A color sensor simulates the food loading mechanism, during the five seconds the robot stops in a 4” color circle simulating a vendor, the measured color is printed in a LED (Light-Emitting Diode) RGB (Red Green Blue color model) strip.
 
![image](https://user-images.githubusercontent.com/73008183/118209662-09781e00-b437-11eb-9f14-e3b45c8cc95a.png)
 
The final robot design is presented below.
 
![image](https://user-images.githubusercontent.com/73008183/118210051-d84c1d80-b437-11eb-8cc5-a4171b2bda47.png)
 
![image](https://user-images.githubusercontent.com/73008183/118210070-e13cef00-b437-11eb-8c40-3561aeda1bf4.png)
 
The robot plans its own route while it is in motion after each turns, detects and avoids obstacles that block its path, and redesigns its own route after avoiding the obstacles.

Wiring Diagrams
 
![image](https://user-images.githubusercontent.com/73008183/119402558-1accf000-bcab-11eb-90bb-abe825dabe1a.png)

![image](https://user-images.githubusercontent.com/73008183/119402572-1f91a400-bcab-11eb-94d9-0b0d292e764a.png)

![image](https://user-images.githubusercontent.com/73008183/119402581-23252b00-bcab-11eb-98e8-49e836e13876.png)

![image](https://user-images.githubusercontent.com/73008183/119402590-27e9df00-bcab-11eb-9458-8a2d969b2741.png)

![image](https://user-images.githubusercontent.com/73008183/119402603-2cae9300-bcab-11eb-8bbf-cac9d27e01a5.png)

CAD Model of the Car Chassis

![image](https://user-images.githubusercontent.com/73008183/119402691-494acb00-bcab-11eb-97a3-f46619f09e11.png)

![image](https://user-images.githubusercontent.com/73008183/119402700-4d76e880-bcab-11eb-9f72-871614a25700.png)

![image](https://user-images.githubusercontent.com/73008183/119402722-55368d00-bcab-11eb-9c5b-995ee290220f.png)
