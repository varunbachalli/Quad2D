# Simulation / Dyanmic Optimization based Trajectory Planning and MPC Trajectory following control of a Planar QuadCopter (2D)

The UI for the code is written in p5py. Which is the python version of p5js which is a interactive drawing tool for java script. Unfortunately p5py is not as mature as p5js. It lacks some functionalities like buttons, and is specially laggy when drawing text on screen. I used it is because i am familiar with the lifecycle and usage of p5js and have used it in the past, so prototyping UI for the project is easier.



To account for the lack of features in p5py. I had to implement buttons and button clicks on my own, which is a fun task. the Buttons are implemented as Rectangles, with text inside them. The buttons can change colors, and have click listeners. 



p5py includes a setup and an draw loop see `Simulation/main.py` . setup loop is called when the program starts and the draw function is called once every frame. 

___
## QuadCopter
___

The quad copter is modelled as a simple bar with forces on either side and gravity acting downwards from the center of the bar. Air resistance and motor characteristics are ignored currently. They will be included in later versions. 

The Nonlinear state space model of the quadcopter can be implemented as 

$$
\frac{d}{dt}
\begin{bmatrix}
  \dot{x}\\ \dot{y}\\ \dot{\theta}\\ x\\ y\\ \theta
\end{bmatrix}
=
\begin{bmatrix}
\frac{F_0 + F_1}{m} \sin(\theta)\\
\frac{F_0 + F_1}{m} \cos(\theta) - g\\
\frac{F_0 - F_1}{I*l}  \\
\dot{x}\\ \dot{y}\\ \dot{\theta}
\end{bmatrix}
$$

where $F_0$ and $F_1$ are forces acting on the left corner and the right corner of the drone. 

The drone is simulated each step using euler integration with a $\Delta t = 0.01s$ 


___
## Dynamic Trajectory Optimization
___

The Dynamic Trajectory Optimization problem is used to get the optimal trajectory for the drone from a given start position to end position.

The problem that is solved is a trapezoidal quadrature approximation based method for approximating the system dynamics. In this optimization problem, total time to reach the end point is added as a cost in the problem that is to be minimized.

$$
\begin{aligned}
 \underset{\boldsymbol{u}(t),\boldsymbol{x}(t),t_f}{min}&&&\int_{t_0}^{t_f} {\omega(\boldsymbol{x}(t) , \boldsymbol{u}(t))} + k*t_f^2\\
 s.t. &&&\frac{d}{dt}{\boldsymbol{x}(t)} = f(\boldsymbol{x}(t) , \boldsymbol{u}(t))\\
 \text{initial condition} &&&\boldsymbol{x}(t_{0}) = \boldsymbol{x}_{0}.\\
 \text{terminal constraints} &&& h(\boldsymbol{x}(t_{f})) \geq 0\\
 \text{equality constraints}&&& g_{e}(\boldsymbol{x}(t), \boldsymbol{u}(t)) = 0 \text{ }\forall \text{ }t\in [t_{0}, t_{f}] , (e \in \textit{E})\\
 \text{inequality constraints}&&& g_{i}(\boldsymbol{x}(t), \boldsymbol{u}(t)) \geq 0 \text{ }\forall \text{ }t\in [t_{0}, t_{f}] ,\text{ } (i \in \textit{I})\\
 \text{control variable boundary values}&&& \boldsymbol{u}_{min}  \leq \boldsymbol{u}(t) \leq \boldsymbol{u}_{max}\text{ }\forall \text{ }t\in [t_{0}, t_{f}]\\
 \text{state variable boundary values}&&& \boldsymbol{x}_{min}  \leq \boldsymbol{x}(t) \leq \boldsymbol{x}_{max}\text{ }\forall \text{ }t\in [t_{0}, t_{f}]
 \end{aligned}
$$


The Nonlinear state space model used in the dynamic trajectory optimization is the same as that used for the Quadcopter simulation.

To solve the above problem. The integral cost function and the system dynamics are approximated using the trapezoidal quadrature. 

$$
\int_{t_0}^{t_f} {\omega(\boldsymbol{x}(t) , \boldsymbol{u}(t))} = \sum_{k = 0}^{N-1} \frac{h_k}{2}  * (\omega_k  + \omega_{k+1}) 
\newline
\frac{d}{dt}{\boldsymbol{x}(t)} = f(\boldsymbol{x}(t) , \boldsymbol{u}(t)) \newline 
\implies  \boldsymbol{x}(t_{k+1}) = \boldsymbol{x}(t_{k}) + \int_{t_k}^{t_{k+1}}  f(\boldsymbol{x}(t) , \boldsymbol{u}(t))
\newline
\implies
\boldsymbol{x}(t_{k+1}) = \boldsymbol{x}(t_{k}) + \frac{h_k}{2} *( f(\boldsymbol{x}(t_{k+1}) , \boldsymbol{u}(t_{k+1}))  + f(\boldsymbol{x}(t_{k}) , \boldsymbol{u}(t_{k})) 

$$

where $h_k$ is the time gap between two grid points. 

The above problem is solved using Casadi in python.


___
## MPC Trajectory following
___

The MPC Trajectory following problem is used to obtain the control inputs to drive the drone along the Trajectory produced by the Trajectory optimization problem.

The problem that is solved is a trapezoidal quadrature approximation based method for approximating the system dynamics. In this optimization problem, total time to reach the end point is added as a cost in the problem that is to be minimized.

$$
\begin{aligned}
 \underset{\boldsymbol{u}(t),\boldsymbol{x}(t),t_f}{min}&&&\int_{t_0}^{t_f} {\omega(\boldsymbol{x}(t) , \boldsymbol{u}(t))}\\
 s.t. &&&\frac{d}{dt}{\boldsymbol{x}(t)} = f(\boldsymbol{x}(t) , \boldsymbol{u}(t))\\
 \text{initial condition} &&&\boldsymbol{x}(t_{0}) = \boldsymbol{x}_{0}.\\
 \text{terminal constraints} &&& h(\boldsymbol{x}(t_{f})) \geq 0\\
 \text{equality constraints}&&& g_{e}(\boldsymbol{x}(t), \boldsymbol{u}(t)) = 0 \text{ }\forall \text{ }t\in [t_{0}, t_{f}] , (e \in \textit{E})\\
 \text{inequality constraints}&&& g_{i}(\boldsymbol{x}(t), \boldsymbol{u}(t)) \geq 0 \text{ }\forall \text{ }t\in [t_{0}, t_{f}] ,\text{ } (i \in \textit{I})\\
 \text{control variable boundary values}&&& \boldsymbol{u}_{min}  \leq \boldsymbol{u}(t) \leq \boldsymbol{u}_{max}\text{ }\forall \text{ }t\in [t_{0}, t_{f}]\\
 \text{state variable boundary values}&&& \boldsymbol{x}_{min}  \leq \boldsymbol{x}(t) \leq \boldsymbol{x}_{max}\text{ }\forall \text{ }t\in [t_{0}, t_{f}]
 \end{aligned}
$$


The Nonlinear state space model used for MPC is the same as that used for the Quadcopter simulation. The cost function also minimizes the distance between the quad copter and the trajectory at an given time.

The problem is solved using Multiple Shooting method and the Runge Kutta integrator. The above problem is solved using Casadi in python at each frame. Only the first control command used for the quad copter. 