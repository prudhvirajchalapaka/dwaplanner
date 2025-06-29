# Custom DWA Local Planner for TurtleBot3 (ROS 2 Humble)

This project implements a custom Dynamic Window Approach (DWA) local planner from scratch for a TurtleBot3 robot in the Gazebo simulation environment using ROS 2 Humble.

## Objective

The primary goal of this project is to develop and test a standalone DWA local planner without relying on existing libraries like nav2_dwb_controller. The implementation focuses on the core principles of the DWA algorithm: velocity sampling, trajectory prediction, and cost-based evaluation to achieve safe and efficient navigation in an environment with obstacles.

## Core Features

- **Custom DWA Algorithm**: Implemented entirely from scratch.
- **Dynamic Velocity Sampling**: Generates safe velocity commands within the robot's dynamic constraints.
- **Trajectory Prediction**: Forecasts the robot's path for each sampled velocity.
- **Multi-Objective Cost Function**: Evaluates trajectories based on distance to the goal, clearance from obstacles, and forward velocity.
- **ROS 2 Integration**: Subscribes to `/odom` and `/scan` topics and publishes velocity commands to `/cmd_vel`.
- **RViz Visualization**: Publishes markers to visualize the sampled trajectories, helping in debugging and visualization.

## Part 1: System Setup and Installation

This section will guide you through the complete setup of the operating system and all required ROS 2 dependencies.

### 1.1. Ubuntu 22.04 Installation

This project is built on Ubuntu 22.04 (Jammy Jellyfish).

**Command**: Follow the official tutorial to install Ubuntu Desktop 22.04.

**Link**: [Install Ubuntu Desktop Tutorial](https://ubuntu.com/tutorials/install-ubuntu-desktop)

![Ubuntu Installation](images/ubuntu_installation.png)
*Ubuntu 22.04 installation process*

### 1.2. Install ROS 2 Humble and Gazebo

These commands will set up the ROS 2 repository, install the full desktop version (which includes RViz2 and Gazebo Classic), and other essential tools.

**Command**: Open a terminal (`Ctrl+Alt+T`) and run the following commands one by one.

```bash
# Set up ROS 2 repositories
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2, Gazebo, and key dependencies
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-common-interfaces \
  ros-humble-tf2-ros \
  ros-humble-tf2-py \
  python3-colcon-common-extensions
```

### 1.3. Source the ROS 2 Environment

You must source the ROS 2 setup file to use ROS commands.

**Command**: Add the source command to your `.bashrc` file to make it available in every new terminal.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Part 2: Workspace Setup & Building

Now we will create a ROS 2 workspace, clone the necessary repositories, and build the project.

### 2.1. Create a ROS 2 Workspace

All our code will reside in this workspace.

**Command**:

```bash
mkdir -p ~/ros2_ws/src
```

### 2.2. Clone Repositories

We need to clone this custom DWA planner repository and the TurtleBot3 simulation packages into the src directory.

**Command**: Navigate to the src directory and clone the repositories.

```bash
cd ~/ros2_ws/src

# Clone this custom DWA planner repository
git clone https://github.com/prudhvirajchalapaka/dwaplanner.git

# Clone the required TurtleBot3 simulation packages for ROS 2 Humble
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
```

![Repository Clone](images/repository_clone.png)
*Cloning repositories into the workspace*

### 2.3. Build the Workspace

This command compiles all the packages in your workspace.

**Command**: Navigate back to the root of your workspace and run colcon build.

```bash
cd ~/ros2_ws/
colcon build
```

The build process may take a few minutes.
sometime it may occurs warnings due ti version conflits. please ignore it. 

![Workspace Build](images/workspace_build.png)
*Building the ROS 2 workspace*

## Part 3: Simulation & Visualization

With the project built, we can now launch the simulation and visualization tools.

### 3.1. A Note on Graphics Performance

Gazebo simulation can be resource-intensive. For a smoother experience and to avoid slow-loading issues, it is recommended to set your graphics card to performance mode if you have a dedicated GPU.

> **Note**: This planner will still work with lower-end graphics, but the simulation may run slowly.

### 3.2. Launch the Gazebo Simulation

This command starts a Gazebo world with a TurtleBot3 robot.

**Command**: Open a **NEW** terminal. Source the environment and set the TurtleBot3 model before launching.

```bash
# Step 1: Source the main ROS 2 environment in the new terminal
source /opt/ros/humble/setup.bash

# Step 2: Set the TurtleBot3 model to 'burger'
export TURTLEBOT3_MODEL=burger

# Step 3: Launch the simulation world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

Gazebo should open and you will see the TurtleBot3 in a simple world with obstacles.

![Gazebo Simulation](videos/video.mp4)
*TurtleBot3 in Gazebo simulation environment*

### 3.3. Launch and Configure RViz2

RViz is the primary tool for visualizing sensor data and planner output.

**Command**: Open another **NEW** terminal.

```bash
# Step 1: Source the main ROS 2 environment in the new terminal
source /opt/ros/humble/setup.bash

# Step 2: Launch RViz2
rviz2
```

#### Configuration:

1. Once RViz opens, change the **Fixed Frame** in the "Global Options" panel from `map` to `base_link`. This will center the view on the robot.

2. Click the **"Add"** button in the bottom-left.

3. In the **"By topic"** tab, find the `/scan` topic and select **LaserScan**. Click OK.

4. You should now see the red laser scan points from the robot's perspective.

![RViz Configuration](images/rviz_configuration.png)
*RViz2 configuration with laser scan visualization*

## Part 4: Running the Custom DWA Planner

This is the final step where we run the custom planner node.

**Command**: Open a **NEW** terminal.

```bash
# Step 1: Source the main ROS 2 environment
source /opt/ros/humble/setup.bash

# Step 2: Source your own workspace's setup file to find your package
source ~/ros2_ws/install/setup.bash

# Step 3: Run the custom DWA planner node
ros2 run dwaplanner dwa_solution
```

#### Execution:

1. The terminal will prompt you to enter the goal coordinates.
2. Enter the `x_goal`: (e.g., 5.0)
3. Enter the `y_goal`: (e.g., 1.0)
4. Observe the robot in Gazebo as it starts moving towards the goal, avoiding obstacles along the way.
5. You will see meaningful debug messages logged in this terminal.

![DWA Planner Execution](dwaplanner/videos/video.mp4)
*Robot navigating using the custom DWA planner*

### 4.1. Visualizing the Planner's Trajectories

To see the trajectories that the planner is considering in real-time:

1. In RViz, click the **"Add"** button again.
2. Go to the **"By topic"** tab.
3. Find the `/planned_paths` topic and select **MarkerArray**. Click OK.
4. You will now see lines radiating from the robot, representing the predicted paths. The chosen path is typically highlighted in a different color (e.g., green).

![Trajectory Visualization](images/trajectory_visualization.png)
*Real-time visualization of planned trajectories in RViz*

This visualization confirms that the robot is successfully navigating using the custom DWA planner, developed entirely from scratch.

## Repository Structure

```
dwaplanner/
├── src/
│   └── dwa_solution.cpp
├── include/
│   └── dwaplanner/
├── package.xml
├── CMakeLists.txt
└── README.md
```

## Contributing

Feel free to submit issues and enhancement requests!

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- ROS 2 Humble community
- TurtleBot3 simulation packages
- Gazebo simulation environment
