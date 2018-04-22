# PP_Cman

## Submission for Term3 Project 1: Path Planning

__Objective:__ 
Implement a path planner that safely navigates a simulator car around a virtual highway cycle of 4.32 miles. There are other cars on the road driving between +-10 MPH of the 50 MPH speed limit. The car shall run as close as possible to the speed limit, passing slower traffic when possible. The car must avoid collisions and must drive inside the lane marks, except when changing lanes. The car must not exceed acceleration over 10 m/s^2 and jerk greater than 50 m/s^3.

> This project integrates the following parts:

1. **Sensor Fusion** - Gather and process sensor data about the vehicle's state and the surrounding objects (for this project: vehicles)
2. **Prediction** - Estimate where the surrounding objects will appear over a finite time horizon (0.02ms)
3. **Behavior Planning** - Decide the vehicle's driving intent and target operating conditions
4. **Trajectory Generation** - Make a path trajectory that achieves the behavior target safely
5. **Control** - Actuate the vehicle to follow the planned path trajectory

Passing the project requires a successful simulator track run and documentation, pls see project rubics: https://review.udacity.com/#!/rubrics/1020/view. Boilerplate code provided by Udacity under https://github.com/udacity/CarND-Path-Planning-Project.

> Improvements made to:

* src/[main.cpp](./src/main.cpp)

## Implementation

Beside existing program code following steps have been implemented:

1. Set initial state of the simulator  
2. Set the initial state of the controller and read the sensor data
3. Detect objects (=vehicless) in same lane and predict their position/behavior
4. Evaluate potential maneuvers and simulate with safety thresholds
5. Actuate safe maneuver
6. Initialize path tangent using previous path waypoints
7. Construct path interpolating predicted waypoints with spline (incl. lane changes) 
8. Space & transform path points and send to simulator

### Implementation Details

#### Initial State
The assumption is that our car will always start in the center lane. It will also start from standstill and gradually accelerate to avoid exceeding the maximum acceleration limit.

#### Creating the Path
To create the path to follow the approach using splines described in the walkthrough is used. It essentially uses `MAX_POINTS` (50 in our case) waypoints starting at the end of the previous path (or, at the start of the simulation, with the car's current position). Then a number of additional (`PATH_NUMPOINTS`, or 3 in our case) waypoints spaced evenly apart (`POINT_SPACING` or 30m in our case) are added. A spline is then fitted to determine intermediate points and produce a smooth vector of waypoints for the car to follow.

#### Avoiding Collisions
The approach to avoiding collisions with cars driving in the same lane is to:

1. Look for cars travelling in the same lane that are deemed too close to our car (set using parameter `LANE_CHANGE_THRESHOLD`)
2. If our car gets too close to another car it tries changing lanes to the left. First it checks if there is a car in the lane to our left, allowing for additional room in front (using `LANE_CHANGE_DIST_FRONT`) and behind (using `LANE_CHANGE_DIST_REAR`) of our car to prevent changing lanes and ending up too close to the other car
3. If a lane change to the left is not possible because a car in that lane would end up too close to our car, or if our car is already in the leftmost lane, a lane change to the right is attempted using the same approach
4. If a lane change to the right is also not possible, the only other option is to slow down and stay in the current lane.

If there are no cars in front of our car, our car will always keep accelerating until it reaches the speed limit.

Three constants are used to determine when a lane change is required, and how much room there needs to be between our car and cars already in the other lane. Theses variables are `LANE_CHANGE_THRESHOLD`, `LANE_CHANGE_DIST_FRONT` and `LANE_CHANGE_DIST_REAR`. Their values were found using trial and error, observing lane change behaviour in the simulator and finding a balance between aggressively changing lanes and maintaining the maximum speed, while avoiding changing lanes into or cutting off other cars.

#### Possible Improvements
1. The car currently favours lane changes to the left because that is the first lane change it will consider when getting too close to another car in the same lane. Only if a lane change to the left is not possible a lane change to the right is considered
2. A lane change only checks for cars immediately to our left or right, with a little bit of space added for safety. It does not consider other traffic in that lane. E.g. it may decide to change lanes to a lane with lots of traffic, or slower traffic, instead of a lane with no other traffic
3. Our car does not check for any cars changing lanes into our lane. Occassionally another car will cause a collision because it is changing lanes into our lane.

## Compilation and Simulation

> build instructions: run shell-commands from the project directory

1. Clone this repo.
2. Make directory in project: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.
5. Run and start the MPC simulator (resolution: 600x800, Mode: fantastic)

## Data Structures 

> See Udacity https://github.com/udacity/CarND-Path-Planning-Project/blob/master/README.md for info about

* Car's localization Data (No Noise)
* Previous path data given to the Planner
* Previous path's end s and d values
* Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

## Environment
* Simulator: The project involves Term 3 Simulator which can be downloaded here: https://github.com/udacity/self-driving-car-sim/releases. A server package uWebSocketIO is setting up a connection from the C++ program to the simulator, which acts as the host.
* OS Setup: Ubuntu 16.4, for details pls see [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
* Eigen package: Eigen is already part of the repo, pls see: http://eigen.tuxfamily.org/index.php?title=Main_Page
