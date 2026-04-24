# ProjectPortfolio

## STM32 Block Sorter
![Prototype STM32 block-sorting robot built from repurposed 3D-printer parts, showing stepper axes, servo gripper, and block positions.](Images/BlockSorterProject.jpg)
Embedded C firmware for an STM32F446RETx NUCLEO board that automates physical block sorting.
The system uses a VL53L0X time-of-flight sensor to measure block heights, then drives stepper motors and a servo gripper to execute swaps.
Users select bubble sort or insertion sort with onboard buttons, and the robot performs the full sorting sequence.
Developed in STM32CubeIDE for pin/peripheral setup, code generation, build, flashing, and debugging.
The robot’s mechanical platform was built by repurposing components from an old 3D printer.