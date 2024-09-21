# Baja

# Team
Aaren Wong

Rishi Biswas

# Project Description 
The University has an engineering team called Blue Jay Racing that competes in building an offroad racing vehicle every year. As the team has progressed, sensors have been added to the car, logging data as well as requiring more actuated components.

With some work on the data acquisition subteam, cameras, lidar mounts, actuator mounts, and feedback sensors have been and can be added to the car in order to outfit it with a full perception stack as well as rudimentary controls.

While there are many components that work independently, they have not been connected in a performant manner, or made to work at their full capacity, needed ros drivers in order to function well.
 
## Software
Although software already exists for the car, it comes in the form of low level HAL drivers, simple wrappers, and overall an incohesive set of libraries not currently bound with any central software element to keep all of the numbers, messages, communication methods, and threads standardized. Here is a big opportunity to build on existing software with ROS2 from RSP in order to make the perception and controls stack fully functionally cohesive.

 
## Hardware
As the car already is in the process of construction, there are many sensors that are physically hard mounted to the car with bolts and weld, allowing for much of the mechatronics to be already done or just needing the last bit of materials to mount the required actuators and potential depth sensor. The team has articulated a strong capacity to support any extraneous mechanical requirements in order to mount any necessary sensors or actuators or compute systems for an RSP project on the car as well as handle any manufacturing and design of needed components.

## Compute
The car currently has a raspberry pi5, accelerator module, and some microcontrollers connected over gigabit ethernet to provide “data access” over ethernet to any requests. As ROS supports ethernet transports, this interface can be used to extend any necessary compute such as with Jetsons or a NUC to do processing.

## Electrical
While there isn’t currently sufficient power for actuators to run fully on the car, listed below is the full specification of needed hardware and components that can work out the box as well as with some hardware assistance of the team to make the components work as plug and play as needed.

## Algorithms
Currently on-car, moderate software has been written for a radio driver and some device drivers to gather data in pure C++. These are planned to be adapted to ROS2 as part of the project as well as supplemented with drivers for components that do not exist in ROS2 as well as with utilization of existing ROS2 drivers for components.

# Hardware Requirements
While there is a solid requirement for the car to exist and functionally be able to mount all of the necessary components, a large part of the project will be to integrate packages from all of the sensors and peripherals into a cohesive set of software which is supported by simulation. The mechanical side will require physical components to be bought or found in order to actuate the car such as linear actuators, compute modules, batteries, and sensor mount hardware, but support for this is given from the University team for the physical side.


# Purchases
A comprehensive set of physical objects can be found [here](https://docs.google.com/spreadsheets/d/1bullgyg-n6dnWoXH10y2o3eHyXzcSN6-LDdDy9N1i8I/edit?usp=sharing).

Additionally a lidar would significantly improve the quality of the perception stack mapping data over the current RDBG system on the car from the Intel Realsense D455. Lidar quotes have been generated for a couple of lidar distributors such as Clearpath(Velodyne) and Cratus(Hesai), as well as some old modules have been acquired to test.

# Deliverables.
While there are many requirements that can be met for the project, the core of this endeavor is to write a solid ROS2 perception and controls base for the Baja SAE car as well as validate the function within simulation.

## Minimal
- Intel realsense image + depth map capture
- InertialSense Rug3-IMX5-Dual RTK localization
- ROS Control package (communicate with robot compatible with ROS control) HAL
- URDF of Robot package (including ROS control tags)
- Data logging for wheel speed, suspension measurements, orientation over ethernet
- Automatic scripts for install, setup
- Launch files for each individual component as well as launch files for entire project
- Configure Ackerman controller (ros control package)
### Packages
1. Intel Realsense Ros2 Package integration
2. InertialSense Ros1 -> Ros2 Migration
3. Data logging package
4. Car + sensor description package (enable/disable sensors)
5. Controls Package

## Expected
- SLAM running on a node, feeding in the RTK, IMU, camera, lidar information to produce a map and localization output
- Wireless microros integration with torque measurements
- Controls stack sending acceleration and brake commands to command longitudinal motion, and steering commands to command lateral motion
- Planning stack to command straight line nominal trajectories
### Packages
1. SLAM package(ORBSLAM 3)
2. Microros package
3. Simple Planning package (navigation stack)

## Maximum
- Autonomous longitudinal and lateral controls
- Simulation validation, putting ackerman steering into gazebo and running the software(w/o sensors)

### Packages
1. Full Stack package
2. Gazebo package and ROS control files 

# Timetable
Demo given on 05/13/2024

# Usage

## CMake version consideration (IMPORANT, PLEASE READ)
Most of the packages in this repository require `CMake` version `>=3.28`. However, you only need `CMake` version `3.8` to build all the packages required for the simulation. Assuming your system is `Ubuntu 22.04`, the default `CMake` version should be `3.22.1`, so your system's `CMake` will need to be updated to build ALL packages. But, you should not need to update your system's `CMake` to build/run ONLY the simulation. Please see the "**Simulation usage**" section below.

## Simulation usage
Usage instructions for the simulation can be found in the file [car_simulation/README.md](https://github.com/jhu-rsp/baja/blob/rsp24/car_simulation/README.md)

## Car development
While a large portion of the cars development can be done within repo, some of the source files would need to be pulled. Particularly the submodules for inertial sense sdk and pahomqtt which are accessible, need to be initialized to build everything.
