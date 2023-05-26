# Electronic_Speed_Controller
An electronic speed controller (ESC) is a vital component in electronic systems that control the speed and direction of electric motors. Primarily used in remote-controlled vehicles, such as drones, RC cars, boats, and airplanes, ESCs are responsible for regulating the power delivered to the motor, allowing precise control over speed and maneuverability.

This project was a part of a 15 ECTS course which was done in OAMK. The basic goal of this project was to create a complete circuit which was capable of driving a brushless motor without heating up excessively.

## The simulation
My first goal was to get a basic grasp of the circuit's operation. I had been searching on the internet on how these circuits work, and landed on a great video on the basic principle of the ESC (Video: )
I used my experience from my school to create a basic simulation. This circuit incorporates a synchronous clock circuit to create the 6 different motor phases which are needed for a full commutation.

## Prototyping
The prototype phase of this project took the longest. To ensure that all the components were functioning as is, all components of the circuit were to be tested individually and together. Notable challenges in this development phase were prototyping the SMD components we used, as they were incredibly fragile to both heat and voltage transients, which broke quite a few of our components.

## Circuit Design
The circuit was designed in KiCad, with custom models for the DRV8300 mosfet driver, as well as the analog multiplexer 74HC5042. This design is quite sparse, due to my inexperience in designing complex circuits. As a result, this PCB incorporates 2 sides with the 12V power plane on the other side.
