# SDCND Term 3 Path Planning Project

In this project a path planner for an autonomous driving car is implemented. A perfect controller is assumed where the simulator visits every waypoint provided, flawlessly.

Simulator returns the waypoints which haven't been visited in the previous timestep (`previous_path`). These points were used in order to generate smooth transitions. The last two points from `previous_path`were used as the first points for the next path to be provided to the simulator.

Next, the class `PathPlanner` is implemented in order to generate the desired behaviour. This class holds the vehicle state in Frenet coordinates and xy coordinates, uses the sensor input from simulator, and utilizes `VehicleState` class for a finite state machine implementation. The path planning is implemented differently for each state.

The state machine is a really simple one, with three states __GoStraight__, __ChangeLaneLeft__ and __ChangeLaneRight__. The latter two are implemented by using one class, `ChangeLane`. 

The default state is `GoStraight`. The state is changed when there is a car in less than 30m distance in ego lane. The passing of the car from left has a high priority. This is a design decision, but makes sense since legal overtaking is done from left hand side for right hand sided traffic. 

If there is no possible collisions on left lane, the state is switched to `ChangeLane`.

If overtaking from left side is not possible, same calculations are done for the right side. 

If it is not possible to overtake the leading car, the state remains at `GoStraight` and the speed is decreased, by taking the max acceleration/ decelaration into account.

In the state `ChangeLane`, the current position of the car is checked and the change lane state is preserved until the car is close to the center of the target lane. Target lane is constantly checked for collisions in this state. If a car comes up from another lane, or a too fast car appears, the state is changed to `GoStraight`, and the car forfeits changing the lane until the conditions allow. 