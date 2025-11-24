# FRA502 Project Jumping_Monopedal

## Table of Contents
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [1-d Monopedal Robot Model With Deadbeat Controller (undamped case)](#1-d-monopedal-robot-model-with-deadbeat-controller-undamped-case)
- [Members](#members)

## System Architecture

### Phase 1: 1-d Monopedal Robot Model With Deadbeat Controller (undamped case)

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

### Phase 2: 2-d Monopedal Robot Model With Reaction Wheel (damped case)
Comming soon!

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



## 1-d Monopedal Robot Model With Deadbeat Controller (undamped case)

This implementation is based on the **Compress-Release Hopper (CRH)** model described in [[1]](#references).

Or You can read the full report [here](./Ambrose%20et%20al.%20-%202019%20-%20Design%20and%20Comparative%20Analysis%20of%201D%20Hopping%20Robots.pdf).


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
| $z_f$ | Foot height from ground | $z_f = z_b - L_0 + q_1 + q_2$ |
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

#### Poincaré Map

![Poincare Map](images/undamped_poincare_map.png)

The Poincaré map shows the relationship between current apex height (Hk) and next apex height (Hk+1). Points clustering near the diagonal at Hd=0.5m indicate **orbital stability**.

### Key Observations

1. **Fast Convergence**: The controller achieves near-deadbeat convergence (~5-10 hops)
2. **Stable Periodic Orbit**: System converges to a stable limit cycle
3. **Small Steady-State Error**: Mean height (0.52m) slightly above target (0.50m) due to simulation dynamics


## Members

- ศิวรุตม์ จุลพรหม, 67340800002
- พัฒนรัตน์ จีดจีน, 68430700410
- พีรดนย์ เรืองแก้ว, 67340700403

## References
[1] Ambrose et al., "Design and Comparative Analysis of 1D Hopping Robots," IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019. DOI: [10.1109/IROS40897.2019.8967692](https://doi.org/10.1109/IROS40897.2019.8967692)  
[2] Optimizing Design and Control of Running Robots Abstracted as Torque Driven Spring Loaded Inverted Pendulum (TD-SLIP). DOI: [10.48550/arXiv.2407.12120](https://doi.org/10.48550/arXiv.2407.12120)