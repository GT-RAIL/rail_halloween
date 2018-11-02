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

- [`task_executor`](task_executor/) - The main workhorse for this package. It executes the task specified in [`config/tasks.yaml`](task_executor/config/tasks.yaml).
- [`manipulation`](manipulation/) - Packages to assist in various manipulation tasks
    - [`candy_manipulation`](candy_manipulation/) - Contains the action servers to:
        - move the arm to specified joint poses
        - move the arm through a pick motion
        - drop off the candy
        - verify that the arm has picked up candy based on vision and gripper position
    - [`data_recorder`](data_recorder/) - Record and playback joint trajectories with some time scaling
- [`local_interfaces`](local_interfaces/) - Methods of interacting with a local human
    - [`bagposefromperson`](local_interfaces/bagposefromperson) - Use pose detection and/or depth thresholding to determine where to drop candy
    - [`hotword_detector`](local_interfaces/hotword_detector) - Detect a hotword "trick-or-treat" trigger
    - [`local_strategy`](local_interfaces/local_strategy) - In case the task executor fails, provide a joystick interface for recovery
- [`assistance_arbitration`](assistance_arbitration/) - Research code to provide assistance when the task fails.
    - [`assistance_arbitrator`](assistance_arbitrator/) - The server that serves assistance requests. Also includes fault detectors.
    - [`assistance_msgs`](assistance_msgs/) - The primary interface between the arbitration server and the strategy servers and the task executor.
- [`simulation_helpers`](simulation_helpers) - Stub interfaces to simulate real robot functionality in simulation

## Prerequisites

The code in this repository requires the installation of the following prerequisites:

```yaml
# Prerequisites installed with `pip install <>`
pip:
- psutil
- pydub

# Prerequisite for tts. Instructions are in the sound_interface package in this repository
docker:
- marytts

# Prerequisite packages. Ensure these are in your ROS workspace
ros:
- GT-RAIL/fetch_demos
- GT-RAIL/fetch_gazebo
- GT-RAIL/fetch_ros
- GT-RAIL/rail_pose_estimation_msgs

# optional
- GT-RAIL/rail_pose_estimator
```


## Quickstart

The bulk of the code in this repository is written in service of the `task_executor` that executes a specified task. To that end, there is only a single launch file in the `task_executor` package that brings up both the dependencies of `task_executor`, as well as the executor itself:

1. Start the Fetch. In the real world, simply turn on the power; in simulation
```bash
roslaunch fetch_gazebo playground.launch
```
2. Start the MoveIt, move_base, grasp planners, speech, etc
```bash
# Use the sim flag only in simulation
roslaunch task_executor trick_or_treat.launch sim:=true start_all:=true task_executor:=false
```
3. Start the task_executor
```bash
# Use the sim flag only in simulation
roslaunch task_executor fetch_deliver.launch sim:=true task_executor:=true
```
4. Setup to run the halloween demo
```bash
rosservice call /halloween/setup
```
5. Start the background thread to wait for the start triggers
```bash
rosservice call /halloween/start
```

When you are done, make sure to call the stop on the halloween-task runner: `rosservice call /halloween/stop`.

Extra helper commands:

1. Run a task defined in `$(find task_executor)/config/tasks.yaml`
```bash
# For example, to run the task with the key `main` in the YAML file
rosrun task_executor run_task.py main
```
2. Run an action defined at [`task_executor/actions`](task_executor/src/task_executor/actions/)
```bash
# Make sure you follow the exact quote syntax followed herefor the JSON
# params. Assuming you want to move to an arm position:
rosrun task_executor run_action.py arm '{"poses": [1,1,1,1,1,0,1], "look_at_gripper": true}'
```
