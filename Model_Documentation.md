# Path Planning Project &ndash; Model Documentation

#### Project Goal
The goal of this project was to build a path planner that can successfully drive the ego vehicle in the Udacity Simulator by preferably maintaining (but never exceeding) the highest speed allowed by the traffic rules and the traffic itself. Another important requirement was for the ego vehicle to execute lane changes in case being blocked by a slower car ahead of it. Excessive acceleration and jerk have to be avoided as well &mdash; just like any collision with another car on the road.

#### Implementation

My solution is simple and conservative. It is based on the skeleton code presented by Aaron Brown in the [walkthrough video](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/23add5c6-7004-47ad-b169-49a5d7b1c1cb/concepts/3bdfeb8c-8dd6-49a7-9d08-beff6703792d) in the classroom.

My solution does not implement a Final State Machine, nor does it calculate jerk minimization and/or path costs explicitly.

##### Behavior Planning

The basic driving or behavioral logic is very simple: if no (slower) car is behind us within a given gap, simply keep the lane and increase the speed until the predefined limit. If a slower car blocks the lane we're currently in (that is, this car is within the proximity gap), look for a neighboring lane to change to. If no lane change can be executed in a safe manner (there is no large enough gap to merge in), decrease the speed to keep a safe distance to the (slower) car ahead of us.

When looking for a gap in a neighboring lane, the program checks each vehicle's *S* coordinate difference from the ego vehicle's current *S* value in the sensor fusion data set. It looks for the information whether this *S* coordinate difference in the given lane is *outside* of the range \[-7.5, +30\] meters (with the ego vehicle being at zero). It also checks whether this condition holds in the foreseeable future based on the vehicle's current speed. In case all the vehicles are (and will be) outside this range in a given lane, a change to this lane becomes feasible.

The proximity gap is adaptive: it considers the current velocity of the ego vehicle and is thus smaller at lower speeds.

The program tries to steer back the ego vehicle into the middle lane if there's a larger gap available there. It also keeps track of whether a previously initiated lane change has been (almost) finished or not (see the variable ```lane_change_in_progress``` in line 137 in ```main.cpp```). This information helps to prevent the vehicle behaving erratically or "jumping" between lanes.

When calculating *S* coordinate differences, the program takes into account that the highway is a closed loop with the *S* values ranging from 0 to 6946.

##### Trajectory Generation

Using splines is a  convenient way to generate a trajectory. Instead of calculating cost and jerk manually for each path (segment) I eliminated the need for explicit jerk minimalization by using splines with slightly elongated control points. The multiplier for the control point "stretching" gets calculated in lines 210-211 in ```main.cpp```. The resulting spline is smooth enough to avoid exceeding the maximum allowed acceleration and jerk values. To ensure compliance with the rules I use a safe and small value for the reference velocity change as well.
