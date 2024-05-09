HERBIE
1. Initial Set up
   First of all you need to make sure Herbie is connected to the Wifi, that the Jetson and the Arduino are connected through serial (USB cable) and that both the Jetson and the Arduino have power.

2. Connecting to the Jetson
  You need to ssh into the Jetson. The Jetson's IP is 10.205.1.41 . It will ask for a password. Consult with the all mighty Titan, god killer, Luis for the password.

3. Operating
   Open the terminal and you can access whatever you want. 


Adding a sensor: 
When adding a sensor you need to connect the cables to the pins of the Jetson Nvidia. Be careful the GPIO pins use 2 diferent mappings. On the one hand is the BOARD, those are the ones you can see on the Jetson. On the other you the BCM, those are use predisposed placements for the pins. I recomend using the BOARD. 
When connecting to the pins, not all pins are SPI so you have to make sure you are using the correct type of pin for your sensor. 
You can consult both BCM and BOARD mappings with the type of pin here: https://jetsonhacks.com/nvidia-jetson-nano-j41-header-pinout/ 















Commands to move in the OYOSOO App: 

Pause - Stop 
Top arrow - move forward 
Botton arrow - move backwards 
Left arrow - move left 
Right arrow - move right 


Obstacle - move sideways to the right 
Tracking -  move sideways to the left
F1 - move foward on the diagonal from left to right 
F6 - move backward on the diagonal from left to right 
F3 - move foward on the diagonal from to right to left 
F4 move backward on the diagonal from to right to left 
