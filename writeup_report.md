## CarND Path Planning Project

The goals / steps of this project are the following:

* In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway.
* The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too.
* The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another.
* The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop.
* Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

[//]: # (Image References)
[image1]: ./output_images/compilation.png
[image2]: ./output_images/show_trajectory.png

[image3]: ./output_images/behavioral1.png
[image4]: ./output_images/behavioral2.png

[image5]: ./output_images/time_diff_cost.png
[image6]: ./output_images/s_diff_cost.png
[image7]: ./output_images/d_diff_cost.png
[image8]: ./output_images/efficiency_cost.png
[image9]: ./output_images/max_jerk_cost.png
[image10]: ./output_images/total_jerk_cost.png
[image11]: ./output_images/collision_cost.png
[image12]: ./output_images/buffer_cost.png
[image13]: ./output_images/max_accel_cost.png
[image14]: ./output_images/total_accel_cost.png
[image15]: ./output_images/exceeds_speed_limit_cost.png

[image16]: ./output_images/valid_trajectory.png
[image17]: ./output_images/invalid_trajectory.png
[image18]: ./output_images/planPath.png

[image19]: ./output_images/scenario1.png
[image20]: ./output_images/scenario2.png
[image21]: ./output_images/scenario3.png
[image22]: ./output_images/scenario4.png
[image23]: ./output_images/scenario5.png
[image24]: ./output_images/scenario6.png
[image25]: ./output_images/scenario7.png
[image26]: ./output_images/scenario8.png

[video1]: ./output_videos/Path_Planning_Final.mp4

## [Rubric](https://review.udacity.com/#!/rubrics/1020/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---

### Compilation

#### 1. The code compiles correctly.

![alt text][image1]

### Valid Trajectories

#### 1. The car is able to drive at least 4.32 miles without incident.

#### 2. The car drives according to the speed limit.

#### 3. Max Acceleration and Jerk are not Exceeded.

#### 4. Car does not have collisions.

#### 5. The car stays in its lane, except for the time between changing lanes.

#### 6. The car is able to change lanes

Here's a [link to my final video result][video1]

### Reflection

#### 1. There is a reflection on how to generate paths.

I choosed planning with:

* time: 2 s
* distance: 44 m
* max velocity: 49.5 mph ~ 22 m/s
* max accelerator: 5 m/s2 < 10 m/s2

Before implement path generation with Polynomial Trajectory Generation (PTG), I tested with `test_ptg.cpp`. I also create a VISUAL_DEBUG mode to show trajectory generation realtime with openCV for debugging easily. And the result as below:

![alt text][image2]

Please add definitions `-DVISUAL_DEBUG` in file `CMakeList.txt` if you want to use it.

It's look like ok after porting code from python to C++ ([Trajectoryexercise2](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/db4b83a1-b304-4355-92c7-66e973102079/concepts/16ed4c00-76c6-49f2-9d53-20ecd26644e0])).

I generated paths by 6 steps as below. In each step, I try to create a function for maintenance easier.

##### 1.1 Update state in the future

File `main.cpp` line `134-172`

I plan with the future state because the calculation will take a lot of time, and we always want to know our future, not our past.

##### 1.2 Make vehicle map

File `main.cpp` line `174-176` and file `algorithm.cpp` line `130-173`

To drive our car safety, I will check others car in 3 lanes with dangerous range is from -6 meters to +88 meters compare with our position.

The car is presented by a circle with radius 1.5m.

##### 1.3 Find the nearest vehicle in the front of us

File `main.cpp` line `186-188` and file `algorithm.cpp` line `175-214`

To decide how to drive, I check the nearest vehicle in the front of us in the same lane.

##### 1.4 Find the best trajectory, best ref_lane

File `main.cpp` line `190-205` and file `algorithm.cpp` line `216-376`

This is planning's heart.

I check the distance to the nearest vehicle in the front of us:
* Between +8m and +44m: find the best trajectory to switch lane or following. Just try to switch to the first next lane, not the second next lane. It's too far and have risk collision. Before switch lane, I check this next lane is safety or not. Safety is when the region -4m to +44m is free without any vehicle.
* Between 0m and +8m: just slow down speed and following
* Other case: just try to keep lane with max velocity

![alt text][image3]
![alt text][image4]

The main function is `ptg()` in file `ptg.cpp` line `3-153`

With each target I create 8 goals near the taget with delta t is 0.2 second.

After generate all goals, I create Jerk Minimizing Trajectory (JMT) for each goal and calculate the cost base on cost function in file `cost_functions.cpp`.

Finally, I choose the PTG have minimize cost.

**Cost functions**:

File `cost_functions.cpp`

* time_diff_cost: Penalizes trajectories that span a duration which is longer or shorter than the duration requested.

![alt text][image5]

* s_diff_cost: Penalizes trajectories whose s coordinate (and derivatives) differ from the goal.

![alt text][image6]

* d_diff_cost: Penalizes trajectories whose d coordinate (and derivatives) differ from the goal.

![alt text][image7]

* efficiency_cost: Rewards high average speeds.

![alt text][image8]

* max_jerk_cost: Penalizes max jerk.

![alt text][image9]

* total_jerk_cost: Penalizes total jerk.

![alt text][image10]

* collision_cost: Binary cost function which penalizes collisions.

![alt text][image11]

* buffer_cost: Penalizes getting close to other vehicles.

![alt text][image12]

* max_accel_cost: Penalizes max accel.

![alt text][image13]

* total_accel_cost: Penalizes total accel.

![alt text][image14]

* exceeds_speed_limit_cost: Penalizes exceeds speed limit.

![alt text][image15]

**Cost functions table**:

File `ptg.h` line `23-33`

| No  | Cost function name        | Weight  |
|-----|---------------------------|---------|
| 0   | s_diff_cost               | 2       |
| 1   | d_diff_cost               | 2       |
| 2   | efficiency_cost           | 100     |
| 3   | time_diff_cost            | 1       |
| 4   | exceeds_speed_limit_cost  | 10      |
| 5   | max_jerk_cost             | 1       |
| 6   | total_jerk_cost           | 1       |
| 7   | max_accel_cost            | 1       |
| 8   | total_accel_cost          | 1       |
| 9   | collision_cost            | 10      |
| 10  | buffer_cost               | 1       |

I try to test many time and get the latest table as above.

##### 1.5 Create path

File `main.cpp` line `207-270` and file `algorithm.cpp` line `378-600`

I try to create and keep a planning path with 50 points, alway use previous data.

It mean that my path will always be a continuous path.

###### 1.5.1 Create spline

File `main.cpp` line `378-554`

Firstly, I add 2 points:

* previous point
* current point

In case I get a valid trajectory as above step, I take 2 points in this trajectory (t=0.5T and t=T) and 1 extra point, that have same d value with trajectory (t=T) and add s value with 44m.

It should make the spline smoothy.

File `algorithm.cpp` line `464-554`

![alt text][image16]

In case I can not get a valid trajectory, I take 3 points in the future:

* 44 m
* 88 m
* 132 m

File `algorithm.cpp` line `378-462`

![alt text][image17]

Always transform the coordinates before create spline:

```cpp
  // Transformation to car coordinates
  for (uint8_t i = 0; i < pts_x.size(); i++)
  {
    // translation
    double shift_x = pts_x[i] - ref_x;
    double shift_y = pts_y[i] - ref_y;
    // rotation clockwise psi
    pts_x[i] = shift_x*cos(-ref_yaw) - shift_y*sin(-ref_yaw);
    pts_y[i] = shift_x*sin(-ref_yaw) + shift_y*cos(-ref_yaw);
  }
```

I have a implementation to fix crash spline in case all x value do not be sorted.

```cpp
  // Remove wrong data before apply spline
  for (uint8_t i = 1; i < pts_x.size();)
  {
    if (pts_x[i] > pts_x[i-1])
    {
      i++;
    }
    else
    {
      pts_x.erase(pts_x.begin()+i);
      pts_y.erase(pts_y.begin()+i);
    }
  }
```

###### 1.5.2 Update velocity

File `main.cpp` line `240-265`

Start speed value is 0 mph.

Max speed is 49.5 mph.

Speed increase/decrease with step value is 0.224 mph/0.02s (acceleration = 5m/s2)

* When start from 0, my velocity will increase until is equal max speed is 49.5. So my car can start smoothly.
* When close other vehicle, my car will decrease velocity slow down.
* With acceleration is 5m/s2. So driver will feel comfortable.

###### 1.5.3 Create planning path base on above spline

File `main.cpp` line `267-270` and file `algorithm.cpp` line `556-600`

I create 50 points for moving. Some of them come from the previous data and I will create the rest part.

![alt text][image18]

To make the car run with a constant velocity, I split the distance line (green line) into N part.

```cpp
  double ref_vel_step = ref_vel*0.02;
  double N = target_dist/ref_vel_step;
```

After that, I predict N points x and calculate N points y: `y = s(x)`.

When the target point near the start point, the distance line (green line) will approximate to the actual line (yellow line). So the actual velocity will approximate to the predict velocity.

But if it is too closed, we can miss some process cycles and maybe we lost controlling.

In here I choose the target distance is 22 m.

Always transform the coordinates before sending to simulator:

```cpp
    // Transformation to global coordinates
    // rotation counter-clockwise psi
    double x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
    double y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);
    // translation
    x_point += ref_x;
    y_point += ref_y;
```

### Regression Testing

Not only `test_ptg.cpp`, I created `test_main.cpp` to test some scenarios before testing with the simulator.

I took too much time for testing and debugging with console log and the simulator. I have too much stress also. This implementation save me many time. So regression testing tool is a great idea. I thing it's not a perfect tool, but it's very helpful for me.

Some scenarios:

* Scenario 1: Do not have vehicle in the dangerous range

![alt text][image19]

* Scenario 2: Do not have vehicle in the dangerous range

![alt text][image20]

* Scenario 3: Have vehicle in the dangerous range

![alt text][image21]

* Scenario 4: Have vehicle in the dangerous range and very near us

![alt text][image22]

* Scenario 5: The vehicle behide us is driving faster than us

![alt text][image23]

* Scenario 6: Still have vehicle in the left of us but we can change to this lane because this vehicle will be in front of us

![alt text][image24]

* Scenario 7: Have vehicle in the left of us and we cannot change to this lane because this vehicle maybe hit us. So we change to right lane

![alt text][image25]

* Scenario 8: Have vehicle in both of left and right lane. We should change to lane which have minimize cost.

![alt text][image26]

---

### References:

* https://www.youtube.com/watch?v=7sI3VHFPP0w
* http://kluge.in-chemnitz.de/opensource/spline/

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

##### Techniques:

* Jerk Minimizing Trajectory.
* Polynomial Trajectory Generation.
* Spline optimization.
* Frenet-Based Algorithm for Trajectory Prediction

##### Fail cases:

* Vehicle behide our car run too fast and cannot brake to avoid collision with us.
* Vehicle next to our car change lane and hit us.

##### Improve:

* Optimize cost function to find the better trajectory for driving.
* Optimize source code to minimize the processing time.