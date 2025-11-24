# FRA502 Project Jumping_Monopedal

## Table of Contents
- [System Architecture](#system-architecture)
- [Installation](#installation)
- [1-d Monopedal Robot Model With Deadbeat Controller (undampinged case)](#1-d-monopedal-robot-model-with-deadbeat-controller-undampinged-case)
- [Members](#members)

## System Architecture

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



## 1-d Monopedal Robot Model With Deadbeat Controller (undampinged case)

comming soon!


## Members

- ศิวรุตม์ จุลพรหม, 67340800002
- พัฒนรัตน์ จีดจีน, 68430700410
- พีรดนย์ เรืองแก้ว, 67340700403