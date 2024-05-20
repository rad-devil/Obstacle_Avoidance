# Obstacle_Avoidance
  Developed obstacle avoidance and lane following algorithm for a four wheeled autonomous vehicle using the FRDM-KL46z board. The I2C interface on the FRDM board was used to communicate with the color sensor (TCS34725) to retrieve color information about the surface.
The implementation for both I2C and ADC for line sensor was done by searching and retrieving information from the respective data sheets.
The assembly of the robot including the wiring to conenct the external battery power source and connection to the motor controller chip was all done from scratch. Wires were cross checked through voltmeter to ensure proper wiring
The abilities of the robot was following:
<img width="571" alt="Screenshot 2024-05-19 at 10 05 15 PM" src="https://github.com/rad-devil/Obstacle_Avoidance/assets/145730909/647d73f8-2aa9-42ec-b202-207239ed5943">


C file is attached with the project implementation and utilizes time delays, communication instantiation (including setting up i/o pins to communicate from the board) were done by skimming though the FRDM board data sheet.
A video demonstration of the project has also been uploaded


