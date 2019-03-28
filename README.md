# ArmOperatingSystem_XYPlotter
![Capture](https://user-images.githubusercontent.com/12455851/55190823-4c200300-51a9-11e9-9c39-618e30362e96.PNG)

## Arm Operating System Project
Metropolia UAS - 2018

The project is an image plotter using two orthogonal axes (X and Y) which are driven by two stepper motors.
This mechanism can allocate a pen anywhere within a rectangular work space. 
A compter software mDraw analyses and produces G codes from the interested image. 
The G codes containing coordinations to move the pen to, moving speed, etc. are then received 
by an arm-based processor LPCXpresso1549 through serial connection. The LPC controls the plotter 
based on the resulting information to produce high quality image.

