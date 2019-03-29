# Arm Operating System - XY Plotter
![Capture](https://user-images.githubusercontent.com/12455851/55190823-4c200300-51a9-11e9-9c39-618e30362e96.PNG)

## Arm Operating System Project
Metropolia UAS - 2018

The project is an image plotter using two orthogonal axes (X and Y) which are driven by two stepper motors.
This mechanism can allocate a pen anywhere within a rectangular work space. 
A compter software mDraw analyses and produces G codes from the interested image. 
The G codes containing coordinations to move the pen to, moving speed, etc. are then received 
by an arm-based processor LPCXpresso1549 through serial connection. The LPC controls the plotter 
based on the resulting information to produce high quality image.

## Technical topics involved:
**Real time operating system FreeRTOS:** assigning LPC's actions into tasks and managing them so that as many tasks can be performed
in a period of time as possible<br/>
**SCT timer:** a powerful timer which can be utilised in a lot of applications. In this particular project, it is to generate
PWM signal for a servo motor<br/>
**RIT timer:** producing a PWM signals with period of microseconds to drive stepper motors<br/>
Semaphore and mutex: task managing mechanism in order to tell the program which task allowed to execute at the time being

## Demo
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/zUFKqqQhi9s/0.jpg)](https://www.youtube.com/watch?v=zUFKqqQhi9s)

## Useful links
**1. FreeRTOS Kernel** https://www.freertos.org/<br/>
**2. XY Plotter Robot Kit:** https://makeblock.com/project/xy-plotter-robot-kit<br/>
**3. LPCXpresso1549 Board:** https://www.nxp.com/support/developer-resources/evaluation-and-development-boards/lpcxpresso-boards/lpcxpresso-board-for-lpc1549:OM13056<br/>
