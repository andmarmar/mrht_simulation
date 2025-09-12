# mrht_simulation

## Multi-Robot Human Teaming Simulation (MRHT Simulation)

**mrht_simulation** is an advanced simulation environment for studying and developing collaboration between multiple robots and humans in hospital scenarios, using ROS 2 and Gazebo. The main package, **turtlebot3_hospital**, provides the nodes, configurations, and utilities needed to simulate mixed teams of TurtleBot3 robots and human agents in a virtual hospital.


## Purpose

- **Mixed team simulation**: Simulate interaction and collaboration between multiple robots and humans in hospital tasks.
- **Dynamic task allocation**: Includes a planner (auctioneer) that assigns tasks to robots and humans based on availability and estimated cost.
- **Robot agent navigation**: Uses Nav2 for TurtleBot3 robot navigation in the hospital environment.
- **Human agent management**: Simulates humans with realistic behaviors and planned routes.



## Package Structure

- **launch/**: Launch files to start the full simulation, robots, humans, planners, and RViz.
- **config/**: Configuration files for tasks, maps, and navigation parameters.
- **turtlebot3_hospital/**: Source code for ROS 2 nodes (planner, task generator, etc).
- **turtlebot3_hospital_msgs/**: Custom message definitions such as `Task.msg`.


## Main Nodes

- **ssi_auctioneer**: Node that manages task allocation to robots and humans using auctions.
- **task_generator**: Node to generate and publish tasks automatically or randomly.
- **human_planner**: Node that plans and publishes trajectories for human agents.



## Other Packages Used

This simulation relies on several additional packages to provide a complete multi-robot and human teaming environment:


- **hunav_sim**: Provides the core logic for human navigation behaviors, agent management, and RViz2 panels for configuring and visualizing human agents and metrics.
- **hunav_gazebo_wrapper**: Integrates the HuNavSim human navigation simulator with Gazebo, enabling realistic simulation of human agents and their interactions with robots in the environment.
- **aws-robomaker-hospital-world**: Supplies detailed hospital world models and assets for Gazebo, creating a realistic simulation environment.
- **turtlebot3_hospital_msgs**: Contains custom ROS 2 message definitions (such as `Task.msg`) used for communication between nodes in the hospital scenario.
- **tb3_multi_robot**: Provides multi-robot launch and configuration utilities for deploying several TurtleBot3 robots in the simulation.

These packages work together to enable realistic simulation, visualization, and coordination of robots and humans in complex hospital scenarios.

## Dependencies

In addition to the packages mentioned above (which are included in this repository), you need to install some extra dependencies for all packages to work correctly.

- **HuNavSim dependencies:**  
  For the human simulation to work, you must follow the instructions in the 'Dependencies' section in the [HuNavSim repository](https://github.com/robotics-upo/hunav_sim) to install all required libraries and packages.

- **Other ROS 2 dependencies:**  
  Make sure to install all ROS 2 dependencies

## Usage

### Launch the full simulation

```sh
ros2 launch turtlebot3_hospital hospital_humans_multirobot.launch.py
```

This will launch:
- The hospital world in Gazebo.
- The TurtleBot3 robots and their navigation systems.
- The simulated human agents with their route planner.
- The task planner and task generator.
- The marker nodes for RViz visualization.

### Publish tasks manually

You can send tasks to the system using:

```sh
ros2 topic pub /new_task turtlebot3_hospital_msgs/Task "{id: 't1', x: 1.0, y: 2.0, frame_id: 'map', duration: 10.0, exclusive: 0}"
```


## Customization

- Edit files in `config/` to change the tasks, parameters of the Turtlebot3 robots or the Rviz2 configuration.
- Edit the file `agents_hospital.yaml` in `hunav_gazebo_wrapper/scenarios` to change the parameters of the humans. 


## Credits and Acknowledgements

This package was created in collaboration with the Department of Systems Engineering and Automation, University of Seville.


## Contact

For questions, suggestions, or contributions, contact the repository maintainer or open an issue on GitHub.

