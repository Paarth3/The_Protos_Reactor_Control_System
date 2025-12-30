# Protos Reactor Control System

A distributed **ROS 2** simulation designed to model and monitor a volatile reactor core. This project implements a publisher-subscriber architecture to simulate reactor physics and enforce safety protocols through real-time telemetry analysis.

## 📖 Project Overview

The **Protos Reactor Control System** addresses a simulated hardware safety gap. The reactor core is inherently unstable, generating fluctuating heat and pressure readings that can lead to a "meltdown" if unchecked.

This project creates a software-based safety sentinel that:

1. **Simulates Physics:** Generates chaotic temperature and pressure data to mimic a live reactor.
2. **Monitors Telemetry:** Consumes data streams in real-time.
3. **Detects Anomalies:** utilizing moving averages to detect rapid temperature spikes.
4. **Triggers Alerts:** Logs warning or fatal error protocols based on defined safety thresholds.

It demonstrates the application of **distributed systems concepts** using the ROS 2 framework to decouple data generation (the hardware) from data analysis (the safety system).

## 🚀 Key Learnings

Building this project provided hands-on experience with **Systems Programming** and **Robotics Middleware**. Key technical competencies include:

- **ROS 2 Architecture:** Implemented a pure C++ Publisher/Subscriber model using `rclcpp` to pass messages between independent nodes.
- **Data Serialization:** Manually parsed and formatted complex telemetry data within standard `std_msgs::msg::String` topics to simulate low-level data handling.
- **Algorithmic Logic:** Implemented a sliding window algorithm (std::vector) to calculate moving averages for trend detection.
- **Concurrency & Timing:** Managed asynchronous callbacks and wall-timers (2 Hz frequency) to ensure consistent telemetry broadcasting every 500 milliseconds.
- **Memory Management:** Utilized C++ smart pointers (`std::shared_ptr`) for safe resource management within the ROS node structure.    

## ⚙️ Technical Details

### Architecture

The system consists of two distinct nodes running asynchronously:

1. **The Core Node (`reactor_core.cpp`)**:
    - Acts as the **Publisher**.
    - Simulates reactor physics by increasing Temperature and Pressure stochastically every 500ms.
    - Broadcasts telemetry to the topic `/hyperion/telemetry`.

2. **The Sentinel Node (`sentinel_monitor.cpp`)**:
    - Acts as the **Subscriber**.
    - Parses the incoming string telemetry (Format: `T:###|P:###`).
    - Maintains a history of the last 10 temperature readings to smooth noise.
    - **Logic:**
        - **Warning:** Triggers if the temperature rises > 5°C per cycle (based on moving average).
        - **Critical Failure:** Triggers if Pressure > 1500 Pa or Temperature > 2000°C.

### Built With

- **Language:** C++17
- **Middleware:** ROS 2 (Robot Operating System) - Jazzy Jalisco
- **Build System:** Colcon / CMake

## 💻 How to Run the Project

These instructions assume you are running a **Linux** environment (e.g., Ubuntu 24.04) with **ROS 2 Jazzy** installed.

### 1. Prerequisite: Install ROS 2

If you have not installed ROS 2 yet, follow the [official ROS 2 Installation Guide](https://docs.ros.org/en/jazzy/Installation.html).

### 2. Source the Environment and Create a Workspace

Source the environment:

```Bash
source /opt/ros/jazzy/setup.bash
```

Then, create a directory for your ROS 2 workspace and the source folder:

```Bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 3. Setup the Package

Create a new package (replace `reactor_control` with your desired package name) and compile the project.

1. **Create the package:**
```Bash
ros2 pkg create --build-type ament_cmake reactor_control
```

2. **Add the code:**
    - Copy `reactor_core.cpp` and `sentinel_monitor.cpp` into `~/ros2_ws/src/reactor_control/src/`.
    - Replace the contents of `package.xml` and `CMakeLists.txt` with the provided configuration files.
    - _Note: Ensure your CMakeLists.txt correctly links the executables to the C++ files._

### 4. Build the Project

_Note: If you are missing dependencies, run `rosdep install -i --from-path src --rosdistro jazzy -y` before building._

Navigate to the root of your workspace and build using `colcon`:

```Bash
cd ~/ros2_ws
colcon build --packages-select reactor_control
```

### 5. Run the System

You will need **two terminal windows** to run the distributed nodes.

**Terminal 1: Start the Reactor Core**

```Bash
cd ~/ros2_ws
source install/setup.bash
ros2 run reactor_control reactor_core
```

**Terminal 2: Start the Sentinel Monitor**

```Bash
cd ~/ros2_ws
source install/setup.bash
ros2 run reactor_control sentinel_monitor
```

### 6. Verify Output

- **Terminal 1** will show logs indicating it is publishing data.
- **Terminal 2** will display the formatted telemetry dashboard and alert messages when the simulated reactor becomes unstable.

---

## 🎥 Demo

A video demonstration (`Demo.mp4`) is included in this repository to visualize the runtime behaviour and console output of the system.
