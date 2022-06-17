# Simulation / Dyanmic Optimization based Trajectory Planning and MPC Trajectory following control of a Planar QuadCopter (2D)

The UI for the code is written in p5py. Which is the python version of p5js which is a interactive drawing tool for java script. Unfortunately p5py is not as mature as p5js. It lacks some functionalities like buttons, and is specially laggy when drawing text on screen. I used it is because i am familiar with the lifecycle and usage of p5js and have used it in the past, so prototyping UI for the project is easier.



To account for the lack of features in p5py. I had to implement buttons and button clicks on my own, which is a fun task. the Buttons are implemented as Rectangles, with text inside them. The buttons can change colors, and have click listeners. 



p5py includes a setup and an draw loop see `Simulation/main.py` . setup loop is called when the program starts and the draw function is called once every frame. 

___
## QuadCopter
___

The quad copter is modelled as a simple bar with forces on either side and gravity acting downwards from the center of the bar. Air resistance and motor characteristics are ignored currently. They will be included in later versions. 

The Nonlinear state space model of the quadcopter can be implemented as 
![image](Image/CodeCogsEqn-4.svg)

where $F_0$ and $F_1$ are forces acting on the left corner and the right corner of the drone. 

The drone is simulated each step using euler integration with a $\Delta t = 0.01s$ 


___
## Dynamic Trajectory Optimization
___

The Dynamic Trajectory Optimization problem is used to get the optimal trajectory for the drone from a given start position to end position.

The problem that is solved is a trapezoidal quadrature approximation based method for approximating the system dynamics. In this optimization problem, total time to reach the end point is added as a cost in the problem that is to be minimized.
![image](Image/CodeCogsEqn.svg)


The Nonlinear state space model used in the dynamic trajectory optimization is the same as that used for the Quadcopter simulation.

To solve the above problem. The integral cost function and the system dynamics are approximated using the trapezoidal quadrature. 
![image](Image/CodeCogsEqn-2.svg)

where $h_k$ is the time gap between two grid points. 

The above problem is solved using Casadi in python.


___
## MPC Trajectory following
___

The MPC Trajectory following problem is used to obtain the control inputs to drive the drone along the Trajectory produced by the Trajectory optimization problem.

The problem that is solved is a trapezoidal quadrature approximation based method for approximating the system dynamics. In this optimization problem, total time to reach the end point is added as a cost in the problem that is to be minimized.

![image](Image/CodeCogsEqn-3.svg)

The Nonlinear state space model used for MPC is the same as that used for the Quadcopter simulation. The cost function also minimizes the distance between the quad copter and the trajectory at an given time.

The problem is solved using Multiple Shooting method and the Runge Kutta integrator. The above problem is solved using Casadi in python at each frame. Only the first control command used for the quad copter. 