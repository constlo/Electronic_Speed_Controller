# Electronic_Speed_Controller
An electronic speed controller (ESC) is a vital component in electronic systems that control the speed and direction of electric motors. Primarily used in remote-controlled vehicles, such as drones, RC cars, boats, and airplanes, ESCs are responsible for regulating the power delivered to the motor, allowing precise control over speed and maneuverability.

This project was a part of a 15 ECTS course which was done in OAMK. The basic goal of this project was to create a complete circuit which was capable of driving a brushless motor without heating up excessively.

Here is the complete product:

<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/20230526_182518.jpg" />

## Components used in this project:
 - STM32F091RBT6 (Microcontroller)
 - DRV8300DIPWR Mosfet Driver
 - IRF3025 Power mosfet x6
 - 74HC4052 Analog multiplexers x2
 - 74HC05 Hex inverter
 - LM2575T LDO with reference design

## The simulation
My first goal was to get a basic grasp of the circuit's operation. I had been searching on the internet on how these circuits work, and landed on a great video on the basic principle of the ESC (Video: )
I used my experience from my school to create a basic simulation. This circuit incorporates a synchronous clock circuit to create the 6 different motor phases which are needed for a full commutation.

<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/motor_model.png" />
Image: Motor model in LTSpice

<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/motor_phases.png" />
Image: Motor phases and pulsed current.

<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/fet_drivers.png" />
Image: Fet Gate array shown above. This is the key component in the ESC.

## Prototyping
The prototype phase of this project took the longest. To ensure that all the components were functioning as is, all components of the circuit were to be tested individually and together. Notable challenges in this development phase were prototyping the SMD components we used, as they were incredibly fragile to both heat and voltage transients, which broke quite a few of our components.
<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/20230411_123105.jpg" />

## Circuit Design
The circuit was designed in KiCad, with custom models for the DRV8300 mosfet driver, as well as the analog multiplexer 74HC5042. This design is quite sparse to make space for future improvements and components.
<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/designed_circuit.png" />

## The code
The basic principle of the software is to create a fixed-frequency PWM signal, which is then driven through analog multiplexers (and inverted in the 74HC05 inverter). It incorporates then 4 GPIO pins and 1 PWM pin, saving on the total pin usage in case this circuit is ever expanded to work on 4 motors.

Most of the code in this project was done in the STMCubeIDE generation tool, as the challenge was to find the most optimum starting clock speed, as well as the PWM frequency. Here is the pinout of the STM32:
<img src="https://github.com/constlo/Electronic_Speed_Controller/blob/master/STM_IOC.png" />

The 4 GPIO pins in the upper section of the STM32 are used to control the multiplexer outputs. They are controlled by the ToggleState() function in the main.c file.
There are 4 ADC inputs as well. One of the inputs is used for reading the potentiometer, which was used in demoing this project. The other 3 inputs are unused, due to time constraints in the project. They were used to read the back EMF of the motor, but due to the relatively low speed of the STM32 they did not sample the signals fast enough. Thus, they were left out of the final project.

## Improvements
This is currently an active project that I'm working on. The next goal of mine is to include Back EMF technology, such as Zero-crossing to detect the motor position accurately. Without the back-EMF, the motor can cog quite easily, drawing excess current and heating up. On the software side I'm also looking forward to implementing PWM- or protocol-based control to work with a Flight controller unit.
