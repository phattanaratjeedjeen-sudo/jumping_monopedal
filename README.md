# FRA502 Project Jumping_Monopedal

## Table of Contents
- [System Architecture](#system-architecture)
  - [Phase 1: 1D Monopedal Robot Model With Deadbeat Controller (undamped case)](#phase-1-1d-monopedal-robot-model-with-deadbeat-controller-undamped-case)
  - [Phase 2: 2D Monopedal Robot Model With Reaction Wheel (undamped case)](#phase-2-2d-monopedal-robot-model-with-reaction-wheel-undamped-case)
- [Installation](#installation)
- [1D Monopedal Robot Model With Deadbeat Controller (undamped case)](#1d-monopedal-robot-model-with-deadbeat-controller-undamped-case)
  - [Usage](#usage)
  - [Model Description](#model-description)
  - [Hybrid Domain Cycle](#hybrid-domain-cycle)
  - [Controller Design (Undamped Case)](#controller-design-undamped-case)
  - [Demo Video](#demo-video)
  - [Simulation Results](#simulation-results)
  - [Key Observations](#key-observations)
- [2D Monopedal Robot Model With Reaction Wheel (undamped case)](#2d-monopedal-robot-model-with-reaction-wheel-undamped-case)
  - [Usage](#usage-1)
  - [Model Description](#model-description-1)
  - [Controller Design](#controller-design)
  - [Demo Video](#demo-video-1)
  - [Simulation Results](#simulation-results-1)
  - [Key Observations](#key-observations-1)
- [Members](#members)
- [References](#references)

## System Architecture

### Phase 1: 1D Monopedal Robot Model With Deadbeat Controller (undamped case)

```mermaid
graph TB
    subgraph Gazebo["Gazebo Simulation"]
        WORLD[Empty World]
        ROBOT[Monoped Robot]
        ALT[Altimeter Sensor]
        IMU[IMU Sensor]
    end

    subgraph ROS2["ROS2 Control"]
        RSP[Robot State Publisher]
        JSB[Joint State Broadcaster]
        EC[Effort Controller]
    end

    subgraph Controller["Deadbeat Controller Node"]
        SM[State Machine]
        ENERGY[Energy Calculator]
        SM --> |state| ENERGY
        ENERGY --> |effort| CMD[Command Publisher]
    end

    subgraph Bridge["ros_gz_bridge"]
        BRIDGE[Parameter Bridge]
    end

    RSP --> |/robot_description| ROBOT
    ROBOT --> ALT
    ROBOT --> IMU
    ALT --> |/altimeter_data| BRIDGE
    IMU --> |/imu_data| BRIDGE
    BRIDGE --> |/altimeter_data| SM
    JSB --> |/joint_states| SM
    CMD --> |/effort_controller/commands| EC
    EC --> ROBOT
```

**Nodes:**
- **Robot State Publisher**: Publishes robot URDF to `/robot_description`
- **Joint State Broadcaster**: Publishes joint positions to `/joint_states`
- **Effort Controller**: Receives effort commands and applies force to joints
- **Deadbeat Controller**: Main control logic with state machine and energy calculation
- **ros_gz_bridge**: Bridges Gazebo topics to ROS2

### Phase 2: 2D Monopedal Robot Model With Reaction Wheel (undamped case)

```mermaid
graph TB
    subgraph Gazebo["Gazebo Simulation"]
        WORLD[Empty World]
        ROBOT[2D Monoped Robot]
        ALT[Altimeter Sensor]
        IMU[IMU Sensor]
    end

    subgraph ROS2["ROS2 Control"]
        RSP[Robot State Publisher]
        JSB[Joint State Broadcaster]
        EC[Effort Controller]
    end

    subgraph Controller["2D Controller Node"]
        SM[State Machine]
        ENERGY[Energy Calculator]
        ATTITUDE[Attitude Controller]
        SM --> |state| ENERGY
        SM --> |state| ATTITUDE
        ENERGY --> |spring effort| CMD[Command Publisher]
        ATTITUDE --> |wheel torque| CMD
    end

    subgraph Bridge["ros_gz_bridge"]
        BRIDGE[Parameter Bridge]
    end

    RSP --> |/robot_description| ROBOT
    ROBOT --> ALT
    ROBOT --> IMU
    ALT --> |/altimeter_data| BRIDGE
    IMU --> |/imu_data| BRIDGE
    BRIDGE --> |/altimeter_data| SM
    BRIDGE --> |/imu_data| ATTITUDE
    JSB --> |/joint_states| SM
    CMD --> |/effort_controller/commands| EC
    EC --> ROBOT
```

**Nodes:**
- **Robot State Publisher**: Publishes robot URDF to `/robot_description`
- **Joint State Broadcaster**: Publishes joint positions to `/joint_states`
- **Effort Controller**: Receives effort commands and applies force/torque to joints
- **2D Controller**: Main control logic with state machine, energy calculation, and attitude control
- **ros_gz_bridge**: Bridges Gazebo topics (altimeter, IMU) to ROS2

## Installation
1. Clone the repository:

```bash
git clone https://github.com/phattanaratjeedjeen-sudo/jumping_monopedal.git
```

2. Navigate to the cloned repository and install dependencies:

```bash
cd jumping_monopedal
sudo apt update && sudo apt upgrade
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```
3. Build the workspace (please ensure you are in the workspace folder; if not, please cd to the workspace folder):

```bash
colcon build --symlink-install
```

4. Source the workspace

```bash
source install/setup.bash
```

> [!TIP]
> Alternatively, you can add this line to your `~/.bashrc` file to source the workspace automatically when you open a new terminal.
> ```bash
> echo "source ~/jumping_monopedal/install/setup.bash" >> ~/.bashrc
> source ~/.bashrc
> ```



## 1D Monopedal Robot Model With Deadbeat Controller (undamped case)

This implementation is based on the **Compress-Release Hopper (CRH)** model described in [[1]](#references).

Or You can read the full report [here](./Ambrose%20et%20al.%20-%202019%20-%20Design%20and%20Comparative%20Analysis%20of%201D%20Hopping%20Robots.pdf).


### Usage
To run the 1D monopedal robot simulation with the deadbeat controller, use the following command:

```bash
ros2 launch monoped_description sim.launch.py
```


### Model Description

![Model Diagram](images/model_v2.png)

Our implementation uses a modified CRH model with two springs:
- **Active Spring**: Controlled by effort controller (energy release)
- **Passive Spring**: Stores energy naturally during ground contact

#### TF Tree (URDF Structure)

<p align="center">
  <img src="images/tf.png" alt="TF Tree" width="400">
</p>

The robot consists of 5 links connected in series:
- **body_link** → **upper_leg_link** (fixed joint)
- **upper_leg_link** → **lower_leg_link** (prismatic joint + active spring)
- **lower_leg_link** → **passive_lower_leg** (prismatic joint + passive spring)
- **passive_lower_leg** → **foot_link** (fixed joint)

#### Coordinate Definitions

| Symbol | Description | Value/Formula |
|--------|-------------|---------------|
| $z_b$ | Body height from ground | Variable (measured by altimeter) |
| $z_f$ | Foot height from ground | $z_f = z_b - L_0$ |
| $L_0$ | leg length | 0.38 m (from body center to foot) |
| $z$ | Vertical axis | Positive upward from ground (z=0) |

Where $q_1$ and $q_2$ are the prismatic joint displacements (spring compressions).

#### Model Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Body mass | $M_b$ | 1.0 kg |
| Foot mass | $M_f$ | 0.1 kg |
| Active Spring stiffness | $k_s$ | 1000 N/m |
| Passive Spring stiffness | $k_s$ | 1000 N/m |
| Leg length | $L_0$ | 0.38 m |

### Hybrid Domain Cycle

<p align="center">
  <img src="images/cycle.png" alt="Hopping Cycle">
</p>

The hopping motion follows a hybrid cycle with multiple domains:

| Domain | Description | Transition |
|--------|-------------|------------|
| $D_{a1}$ | First air phase (hardstop active, spring uncompressed) | $t = t_s$ → $D_{a2}$ |
| $D_{a2}$ | Second air phase (spring compression before landing) | $\Delta_L$ (landing impact) → $D_g$ |
| $D_g$ | Ground phase (foot on ground, spring stores/releases energy) | $\Delta_T$ (takeoff impact) → $D_{a1}$ |

**Key Variables:**
- $H_k$ — Apex height of current hop
- $H_{k+1}$ — Apex height of next hop
- $t_s$ — Time to start spring compression
- $\Delta_L$ — Landing impact (reset map)
- $\Delta_T$ — Takeoff impact (reset map)

### Controller Design (Undamped Case)

The deadbeat controller calculates the required energy input to reach the desired hop height using:

$$u = EL + ET + EH$$

Where:
- $EL = Mf \times g \times (Hc - L0)$ — Energy lost at landing impact
- $ET = (Mb + Mf) \times g \times (Hd - L0) \times \frac{Mf}{Mb}$ — Energy lost at takeoff impact
- $EH = (Mb + Mf) \times g \times (Hd - Hc)$ — Potential energy for height change
- $u$ — Potential energy of active spring must be generated

The desired spring compression ($\delta^*$) is then:

$$
\delta^* = \sqrt{\frac{2u}{k_s}}
$$

### Demo Video

![demo_video](images/demo-phase1_v2.gif)

For better view please watch the [video here](images/demo-phase1_v2.mp4).

### Simulation Results

| Parameter | Value |
|-----------|-------|
| Duration | 50.95 seconds |
| Desired height (Hd) | 0.60 m |
| Mean apex height | 0.633 m |
| Std deviation | ±0.035 m |
| Number of hops | 81 |
| Convergence | ~5-10 hops |

#### Hopping Behavior Analysis

![Hopping Analysis](images/hopping_analysis_v2.png)

**Plot Description:**
1. **Top**: Body and foot height over time showing stable periodic hopping
2. **Second**: Apex height convergence from initial ~0.76m to target ~0.6m
3. **Third**: Phase plot (zb vs velocity) showing convergence to limit cycle
4. **Bottom**: Force commands applied during compress phase

### Key Observations

1. **Fast Convergence**: The controller achieves near-deadbeat convergence (~5-10 hops)
2. **Stable Periodic Orbit**: System converges to a stable limit cycle
3. **Small Steady-State Error**: Mean height (0.633m) slightly above target (0.60m) due to simulation dynamics

## 2D Monopedal Robot Model With Reaction Wheel (undamped case)

The quation of motion is based on the **Torque Driven Spring Loaded Inverted Pendulum (TD-SLIP)** model described in [[2]](#references). Or You can read the full report [here](./2407.12120%202.pdf).

This phase is developed based on Phase 1 [1D Monopedal Robot Model With Deadbeat Controller (undamped case)](#1d-monopedal-robot-model-with-deadbeat-controller-undamped-case). By adding 2 reaction wheel, full state feed controller to control the tilt angle of the robot and some robot's urdf modifiaction 

### Usage
To run the 2D monopedal robot simulation with the reaction wheel controller, use the following command:

```bash
ros2 launch monoped_2d_description sim.launch.py
```

### Model Description

![2D Model Diagram](images/model_phase2.png)

The 2D monopedal robot extends the 1D vertical hopper with planar motion and attitude control via reaction wheels. The system dynamics are governed by coupled angular and radial motion equations.

#### Coordinate Definitions

| Symbol | Unit | Description | Value/Formula |
|--------|------|-------------|---------------|
| $\theta$ | deg | Robot pitch angle reference from ground | $\theta = 90° - \text{angle from IMU}$ |
| $\ell$ | m | Leg length (radial distance) | Variable (measured) |
| $\ell_0$ | m | Operating leg length | 0.30 m |

#### Hopping Cycle Illustration

<p align="center">
  <img src="images/image.png" alt="2D Hopping Cycle" width="600">
</p>

The figure above illustrates the complete hopping cycle showing both stance and flight phases, with reaction wheels providing attitude control throughout the motion.

#### Dynamics Equations

**Stance Phase:**

The robot dynamics during ground contact are governed by:

$$\ddot{\theta} = -\frac{2\dot{\zeta}\dot{\theta}}{\zeta} - \frac{g\cos(\theta)}{\zeta} + \frac{\tau}{m\zeta^2}$$

$$\ddot{\zeta} = \zeta\dot{\theta}^2 - g\sin(\theta) - \frac{k_0}{m}(\zeta - \ell_0) - \frac{b_\ell}{m}\dot{\zeta}$$



**Flight Phase:**

During flight, only the reaction wheel provides control authority:

$$\tau = I_{robot} \cdot \ddot{\theta}$$

Where $I_{robot}$ is the robot's moment of inertia about the center of body.

#### TF Tree (URDF Structure)

<p align="center">
  <img src="images/tf_phase2.png" alt="TF Tree" width="400">
</p>

#### Model Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Body mass | $m$ | 0.5 kg |
| Reaction wheel mass (per wheel) | $M_{rw}$ | 0.1 kg |
| Thigh mass | $M_{thigh}$ | 0.01 kg |
| Shank mass | $M_{shank}$ | 0.01 kg |
| Foot mass | $M_{foot}$ | 0.1 kg |
| Spring stiffness | $k_0$ | 1000 N/m |
| Leg damping | $b_\ell$ | 0.0 N·s/m |
| Spring equilibrium length | $\ell_0$ | 0.30 m |
| Gravity acceleration | $g$ | 9.81 m/s² |
| Body dimensions | $b_x \times b_y \times b_z$ | 0.15 × 0.15 × 0.15 m |
| Reaction wheel dimensions | $r_{rw} \times l_{rw}$ | 0.15 × 0.04 m |

### Hybrid Domain Cycle

The 2D hopping motion follows a hybrid cycle similar to Phase 1:

| Domain | Description | Transition |
|--------|-------------|------------|
| **Flight** | Combination of air + compress domain from 1D model; robot in free fall with reaction wheel control | $z_f \leq 0$ → Ground |
| **Ground** | Foot in contact with ground; spring compression/extension + attitude control | Takeoff → Flight |

### Controller Design

The 2D controller combines vertical hopping control with pitch stabilization using full state-feedback control.

#### Vertical Control (Energy-Based Control)
The vertical hopping controller uses the same energy-based deadbeat approach from Phase 1 to regulate apex height through spring compression control.

#### Pitch Stabilization (State-Feedback Control)
There are 2 controller for each domain (ground and flight) due to equation of motion in flight and ground phase.

**Ground Phase**: The stance phase dynamics are linearized around an operating point to design an LQR controller.

**State Vector:** $\mathbf{x} = [\theta, \dot{\theta}, \ell, \dot{\ell}]^T$

**Operating Point:**
- Leg length: $\ell = 0.3$ m
- Leg velocity: $\dot{\ell} = 0$ m/s
- Pitch angle: $\theta = \pi/2$ rad (90°, upright)
- Angular velocity: $\dot{\theta} = 0$ rad/s

The linearized system takes the form:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\tau$$

Where $A$ and $B$ are obtained by linearizing the stance phase dynamics equations.

**LQR Design:**
State-feedback gains are computed using MATLAB's `lqrd` function with sampling period 10 ms and defined Q and R matrices by using **Bryson's Rule** with:
- Acceptable pitch angle error = 3° (0.0524 rad)
- Torque limitation = 1 Nm

Which yields:
- State weighting matrix: $Q = \text{diag}([365, 0, 0, 0])$ where $365 = (1/0.0524)^2$
- Control weighting: $R = 1$

This yields the stance phase gain: $K_g = [17.94, 1.26, 0.0, 0.0]$

**Feedforward Gain:**
The feedforward gain $N_g$ is computed using the `rscale` function to eliminate steady-state error for step reference tracking:

$$N_g = N_u + K_g \cdot N_x$$

Where $[N_x^T, N_u]^T = [A, B; C, D]^{-1} [0, 0, 0, 0, 1]^T$

For this system, $N_g = 17.94$ (equal to $K_g[0]$) 

**MATLAB Implementation:**
The controller gains are computed offline using MATLAB. The implementation can be found in the [matlab](matlab/) directory:
- [hop_2d.mlx](matlab/hop_2d.mlx) - Main script for computing LQR gains (ground phase) and pole placement gains (flight phase)
- [rscale.m](matlab/rscale.m) - Function to compute feedforward gain $N$ for step reference tracking
- [hop_2D_sim.slx](matlab/hop_2D_sim.slx) - Simulink model for system simulation



**Control Law (Ground Phase):**

$$\tau = N_g \theta_d - K_g \cdot \mathbf{x}$$

Where:
- $\theta_d = \pi/2$ rad (90°, upright position)
- $\mathbf{x} = [\theta, \dot{\theta}, \ell, \dot{\ell}]^T$ is the state vector

**Flight Phase:**
During flight, the system has reduced 2-state dynamics (no leg length states). The controller uses pole placement with similar structure.

**State Vector:** $\mathbf{x}_f = [\theta, \dot{\theta}]^T$

**Design Parameters:**
- State-feedback gains: $K_f = [15.82, 0.92]$
- Feedforward gain: $N_f = 15.82$

**Control Law (Flight Phase):**

$$\tau = N_f \theta_d - K_f \cdot \mathbf{x}_f$$

**Summary of Control Gains:**
| Phase | State-Feedback Gains $K$ | Feedforward Gain $N$ |
|-------|--------------------------|---------------------|
| Ground | $[17.94, 1.26, 0.0, 0.0]$ | $17.94$ |
| Flight | $[15.82, 0.92]$ | $15.82$ |


### Demo Video

![demo_video](images/demo-phase2_v2.gif)

For better view please watch the [video here](images/demo-phase2_v2.mp4).

### Simulation Results

| Parameter | Value |
|-----------|-------|
| Desired height (Hd) | 0.60 m |
| Mean apex height | 0.651 m |
| Std deviation (height) | ±0.120 m |
| Forward pitch angle | 96.11° |
| Backward pitch angle | 82.73° |


#### Hopping and Attitude Behavior Analysis

![Hopping Analysis](images/hopping_analysis_phase2_v2.png)

**Plot Description:**
1. **Top Left**: Body pitch angle showing oscillation (target alternates between 85° and 95° during ground phase for disturbance testing)
2. **Top Right**: Body and foot height showing periodic hopping behavior
3. **Middle Left**: Reaction wheel torque commands for attitude stabilization
4. **Middle Right**: Apex height per hop showing convergence to mean height
5. **Bottom Left**: Spring effort commands during stance phase
6. **Bottom Right**: Phase portrait (theta vs theta_dot) with color gradient representing time progression

#### Zoomed View (First 10 seconds)

![Zoomed Analysis](images/zoomed_0_10s_phase2_v2.png)

### Key Observations

1. **Attitude Stabilization**: Reaction wheels successfully maintain pitch angle near 90° (mean = 90.13°, std = 1.80°)
2. **Pitch Range**: Forward pitch angle reached 96.11° and backward pitch angle reached 82.73°, showing controlled oscillation within ±6° range
3. **Hopping Performance**: Robot achieved around 4 hops per seconds with mean apex height of 0.651m

## Members

- ศิวรุตม์ จุลพรหม, 67340800002
- พัฒนรัตน์ จีดจีน, 68430700410
- พีรดนย์ เรืองแก้ว, 67340700403

## References
- [1] Ambrose et al., "Design and Comparative Analysis of 1D Hopping Robots," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019. DOI: [10.1109/IROS40897.2019.8967692](https://doi.org/10.1109/IROS40897.2019.8967692)  
- [2] Optimizing Design and Control of Running Robots Abstracted as Torque Driven Spring Loaded Inverted Pendulum (TD-SLIP). DOI: [10.48550/arXiv.2407.12120](https://doi.org/10.48550/arXiv.2407.12120)
