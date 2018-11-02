# RAIL Halloween

```
         \\                \\                \\
    .-'```^```'-.     .-'```^```'-.     .-'```^```'-.
   /   /\ __ /\  \   /   (\ __ /)  \   /   /) __ (\  \
   |   ^^ \/ ^^  |   |    ` \/ `   |   |   ^  \/  ^  |
   \   \_.__._/  /   \    \____/   /   \    `'=='`   /
    `'-.......-'`     `'-.......-'`     `'-.......-'`
```

Packages:

- [`task_executor`](task_executor/) - The main workhorse for this package. It executes the task specified in `config/tasks.yaml`.
- [`candy_manipulation`](candy_manipulation/) - Contains the action servers to:
    - move the arm to specified joint poses
    - move the arm through a pick motion
    - drop off the candy
    - verify that the arm has picked up candy based on vision and gripper position
- [`assistance_arbitrator`](assistance_arbitrator/) - Research code that uses context passed on from the task_executor (or another node that might be requesting assistance), and decides where to forward the request - local or remote.
- [`local_interfaces`](local_interfaces/) - If we decide to solicit help from a local person, then the code in this folder (main entrypoint: [`local_interfaces/local_strategy`](local_interfaces/local_strategy)) handles the presentation of the request to a human.
- [`assistance_msgs`](assistance_msgs/) - The primary interface between all the other modules in this folder.

## Prerequisites

The code in this repository requires the installation of the following prerequisites:

```yaml
# Prerequisites installed with `pip install <>`
pip:
- psutil
- pydub

# Prerequisite for tts. Instructions are in the speech_interface package in this repository
docker:
- marytts

# Prerequisite packages. Ensure these are in your ROS workspace
ros:
- GT-RAIL/fetch_demos
- GT-RAIL/fetch_gazebo
- GT-RAIL/fetch_ros

# optional
- GT-RAIL/rail_pose_estimation
```

In the near future, these will be packaged into a script(s) placed in the `install` directory.


## Quickstart

The bulk of the code in this repository is written in service of the `task_executor` that executes a specified task. To that end, there is only a single launch file in the `task_executor` package that brings up both the dependencies of `task_executor`, as well as the executor itself:

1. Start the Fetch. In the real world, simply turn on the power; in simulation
```bash
roslaunch fetch_gazebo playground.launch
```
2. Start the MoveIt, move_base, grasp planners, speech, etc
```bash
# Use the sim flag only in simulation
roslaunch task_executor fetch_deliver.launch sim:=true start_all:=true task_executor:=false
```
3. Start the task_executor
```bash
# Use the sim flag only in simulation
roslaunch task_executor fetch_deliver.launch sim:=true task_executor:=true
```
4. Run a task defined in `$(find task_executor)/config/tasks.yaml`
```bash
# For example, to run the task with the key `main` in the YAML file
rosrun task_executor run_task.py main
```
