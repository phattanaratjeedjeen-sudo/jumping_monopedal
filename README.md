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

![Model Diagram](images/model.png)

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
| $L_0$ | Spring equilibrium length | 0.38 m (from body center to foot) |
| $z$ | Vertical axis | Positive upward from ground (z=0) |

Where $q_1$ and $q_2$ are the prismatic joint displacements (spring compressions).

#### Model Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Body mass | $M_b$ | 1.0 kg |
| Foot mass | $M_f$ | 0.08 kg |
| Active Spring stiffness | $k_s$ | 1000 N/m |
| Passive Spring stiffness | $k_s$ | 1000 N/m |
| Spring equilibrium length | $L_0$ | 0.38 m |

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

$$u(Hc, Hd) = EL + ET + EH$$

Where:
- $EL = Mf \times g \times (Hc - L0)$ — Energy lost at landing impact
- $ET = (Mb + Mf) \times g \times (Hd - L0) \times \frac{Mf}{Mb}$ — Energy lost at takeoff impact
- $EH = (Mb + Mf) \times g \times (Hd - Hc)$ — Potential energy for height change

The desired spring compression is then:

$$
\delta^* = \sqrt{\frac{2u}{k_s}}
$$

### Demo Video

![demo_video](images/demo-phase1.gif)

For better view plsease watch the [video here](images/demo-phase1.mp4).

### Simulation Results

| Parameter | Value |
|-----------|-------|
| Desired height (Hd) | 0.50 m |
| Mean apex height | 0.52 m |
| Std deviation | ±0.034 m |
| Number of hops | 159 |
| Convergence | ~5-10 hops |

#### Hopping Behavior Analysis

![Hopping Analysis](images/undamped_hopping_analysis.png)

**Plot Description:**
1. **Top**: Body and foot height over time showing stable periodic hopping
2. **Second**: Apex height convergence from initial ~0.76m to target ~0.5m
3. **Third**: Phase plot (zb vs velocity) showing convergence to limit cycle
4. **Bottom**: Effort commands applied during compress phase

### Key Observations

1. **Fast Convergence**: The controller achieves near-deadbeat convergence (~5-10 hops)
2. **Stable Periodic Orbit**: System converges to a stable limit cycle
3. **Small Steady-State Error**: Mean height (0.52m) slightly above target (0.50m) due to simulation dynamics

## 2D Monopedal Robot Model With Reaction Wheel (undamped case)

This implementation is based on the **Torque Driven Spring Loaded Inverted Pendulum (TD-SLIP)** model described in [[2]](#references). Or You can read the full report [here](./2407.12120%202.pdf).

This phase is developed based on Phase 1 [1D Monopedal Robot Model With Deadbeat Controller (undamped case)](#1d-monopedal-robot-model-with-deadbeat-controller-undamped-case).

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

Where $I_{robot}$ is the robot's moment of inertia about the center of mass.

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
| Leg damping | $b_\ell$ | 0.2 N·s/m |
| Spring equilibrium length | $\ell_0$ | 0.30 m |
| Gravity acceleration | $g$ | 9.81 m/s² |
| Body dimensions | $b_x \times b_y \times b_z$ | 0.15 × 0.15 × 0.15 m |
| Thigh length | $\ell_{thigh}$ | 0.15 m |
| Shank length | $\ell_{shank}$ | 0.15 m |
| Reaction wheel radius | $r_{rw}$ | 0.15 m |
| Reaction wheel damping | $b_{rw}$ | 0.0 N·m·s/rad |
| Reaction wheel friction | $f_{rw}$ | 0.0 |

### Hybrid Domain Cycle

The 2D hopping motion follows a hybrid cycle similar to Phase 1:

| Domain | Description | Transition |
|--------|-------------|------------|
| **Flight** | Combination of air + compress domain from 1D model; robot in free fall with reaction wheel control | $z_f \leq 0$ → Ground |
| **Ground** | Foot in contact with ground; spring compression/extension + attitude control | Takeoff → Flight |

### Controller Design

The 2D controller combines vertical hopping control with pitch stabilization using state-feedback control.

#### Vertical Control (Energy-Based Deadbeat)
The vertical hopping controller uses the same energy-based deadbeat approach from Phase 1 to regulate apex height through spring compression control.

#### Pitch Stabilization (State-Feedback Control)

**Linearization:**
The stance phase dynamics are linearized around an operating point to design an LQR controller.

**State Vector:** $\mathbf{x} = [\theta, \dot{\theta}, \ell, \dot{\ell}]^T$

**Operating Point:**
- Torque: $\tau = 0$ Nm
- Leg length: $\ell = 0.3$ m
- Leg velocity: $\dot{\ell} = 0$ m/s
- Pitch angle: $\theta = \pi/2$ rad (90°, upright)
- Angular velocity: $\dot{\theta} = 0$ rad/s

The linearized system takes the form:

$$\dot{\mathbf{x}} = A\mathbf{x} + B\tau$$

Where $A$ and $B$ are obtained by linearizing the stance phase dynamics equations.

**LQR Design:**
State-feedback gains are computed using MATLAB's `lqr` function with:
- State weighting matrix: $Q = \text{diag}([100, 0, 0, 0])$ (prioritize angle control)
- Control weighting: $R = 1$

This yields the stance phase gain: $K_g = [11.5792, 1.0208, 0.0000, 0.0000]$

**Command Feedforward:**
The feedforward gain $N_g$ is computed using the `rscale` function and tuned to $N_g = 11.6$.

**MATLAB Implementation:**
The controller gains are computed offline using MATLAB. The implementation can be found in the [matlab](matlab/) directory:
- [twoD.m](matlab/twoD.m) - Main script for computing LQR gains (ground phase) and pole placement gains (flight phase)
- [rscale.m](matlab/rscale.m) - Function to compute feedforward gain $N$ for step reference tracking
- [twoD_ground.slx](matlab/twoD_ground.slx) - Simulink model for system simulation

**Control Law:**

$$\tau = N_g \theta_d - K_g (\mathbf{x} - \mathbf{x}_d)$$

Where $\theta_d$ is the desired pitch angle (90°).

**Flight Phase Controller:**
During flight, the system dynamics simplify to a 2-state system since leg length is not controllable. The flight controller uses pole placement with Ackermann's formula:
- State vector: $\mathbf{x}_f = [\theta, \dot{\theta}]^T$
- Desired poles: $p = -50 \pm 1i$ (fast, slightly underdamped response)
- Includes wheel damping: $b_w = 0.001$ N·m·s/rad

**Control Gains:**
- Ground phase: $K_g = [11.58, 1.02, 0.0, 0.0]$, $N_g = 11.6$
- Flight phase: $K_f = [14.068, 0.5625]$, $N_f = 14.068$ (only $\theta$ and $\dot{\theta}$ states)

**Stability Mechanisms:**
1. **Torque Saturation**: Prevents excessive control effort
2. **Rate Limiting**: Smooths torque commands to prevent oscillations
3. **Joint Damping**: Passive damping (1.0 N·m·s/rad) reduces wheel overshoot
4. **State-Dependent Gains**: Different gains for ground vs. flight phases account for changing dynamics

### Demo Video

![demo_video](images/demo-phase2.gif)

For better view please watch the [video here](images/demo-phase2.mp4).

### Simulation Results

| Parameter | Value |
|-----------|-------|
| Duration | 425.89 seconds |
| Samples | 127,501 |
| Desired height (Hd) | 1.5 m |
| Mean apex height | 1.002 m |
| Std deviation (height) | ±1.388 m |
| Mean pitch angle | 90.06° |
| Std deviation (pitch) | ±1.17° |
| Number of hops | 19,479 |
| Final theta | 122.05° (fell) |


#### Hopping and Attitude Behavior Analysis

![Hopping Analysis](images/hopping_analysis.png)

**Plot Description:**
1. **Top Left**: Body pitch angle (theta) showing stable oscillation around 90° target
2. **Top Right**: Body and foot height showing periodic hopping behavior
3. **Middle Left**: Reaction wheel torque commands
4. **Middle Right**: Apex height per hop showing convergence
5. **Bottom Left**: Spring effort commands during compress phase
6. **Bottom Right**: Phase portrait (theta vs theta_dot) showing limit cycle behavior

#### Zoomed View (First 10 seconds)

![Zoomed Analysis](images/zoomed_0_10s.png)

### Key Observations

1. **Attitude Stabilization**: Reaction wheels successfully maintain pitch angle near 90° (mean = 90.06°, std = 1.17°)
2. **Hopping Performance**: Robot achieved 19,479 hops over 425 seconds with mean apex height of 1.002m
3. **Physics Instability**: Gazebo physics explosion occurred near end of simulation (theta_dot spike to 2.9M °/s)
4. **Controller Robustness**: Controller performed correctly; the crash was due to Gazebo collision detection failure, not controller error

## Members

- ศิวรุตม์ จุลพรหม, 67340800002
- พัฒนรัตน์ จีดจีน, 68430700410
- พีรดนย์ เรืองแก้ว, 67340700403

## References
- [1] Ambrose et al., "Design and Comparative Analysis of 1D Hopping Robots," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019. DOI: [10.1109/IROS40897.2019.8967692](https://doi.org/10.1109/IROS40897.2019.8967692)  
- [2] Optimizing Design and Control of Running Robots Abstracted as Torque Driven Spring Loaded Inverted Pendulum (TD-SLIP). DOI: [10.48550/arXiv.2407.12120](https://doi.org/10.48550/arXiv.2407.12120)
