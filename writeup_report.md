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
[image3]: ./output_images/valid_trajectory.png
[image4]: ./output_images/invalid_trajectory.png
[image5]: ./output_images/planPath.png
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

Before implement path generation with Polynomial Trajectory Generation (PTG), I tested with `main_test.cpp`. I also create a VISUAL_DEBUG mode to show trajectory generation realtime with openCV for debugging easily. And the result as below:

![alt text][image2]

It's look like ok after porting code from python to C++ ([Trajectoryexercise2](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/27800789-bc8e-4adc-afe0-ec781e82ceae/lessons/db4b83a1-b304-4355-92c7-66e973102079/concepts/16ed4c00-76c6-49f2-9d53-20ecd26644e0])).

I generated paths by 6 steps as below. In each step, I try to create a function for maintenance easier.

##### 1.1 Update state in the future

File `main.cpp` line `675-713`

I plan with the future state because the calculation will take a lot of time, and we always want to know our future, not our past.

##### 1.2 Make vehicle map

File `main.cpp` line `715-717` and `164-207`

To drive our car safety, I will check others car in 3 lanes with dangerous range is from -6 meters to +88 meters compare with our position.

The car is presented by a circle with radius 1.5m.

##### 1.3 Find the nearest vehicle in the front of us

File `main.cpp` line `727-729` and `209-248`

To decide how to drive, I check the nearest vehicle in the front of us in the same lane.

##### 1.4 Find the best trajectory, best ref_lane

File `main.cpp` line `731-746` and `250-352`

This is planning's heart.

I check the distance to the nearest vehicle in the front of us:
* Between 3m and 44m: find the best trajectory to switch lane or following
* Other case: just try to keep lane with max velocity

The main function is `ptg()` in file `ptg.cpp` line `3-153`

With each target I create 8 goals near the taget with delta t is 0.2 second.

After generate all goals, I create Jerk Minimizing Trajectory (JMT) for each goal and calculate the cost base on cost function in file `cost_functions.cpp`.

Finally, I choose the PTG have minimize cost.

**Cost functions**:

* time_diff_cost
* s_diff_cost
* d_diff_cost
* efficiency_cost
* max_jerk_cost
* total_jerk_cost
* collision_cost
* buffer_cost
* max_accel_cost
* total_accel_cost

##### 1.5 Create path

File `main.cpp` line `748-811` and `354-575`

I try to create and keep a planning path with 50 points, alway use previous data.

It mean that my path will always be a continuous path.

###### 1.5.1 Create spline

File `main.cpp` line `755-779`

Firstly, I add 2 points:

* previous point
* current point

In case I get a valid trajectory as above step, I take 2 points in this trajectory (t=0.5T and t=T) and 1 extra point, that have same d value with trajectory (t=T) and add s value with 44m.

It should make the spline smoothy.

![alt text][image3]

In case I can not get a valid trajectory, I take 3 points in the future:

* 44 m
* 88 m
* 132 m

![alt text][image4]

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

File `main.cpp` line `781-806`

Start speed value is 0 mph.

Max speed is 49.5 mph.

Speed increase/decrease with step value is 0.224 mph/0.02s (acceleration = 5m/s2)

* When start from 0, my velocity will increase until is equal max speed is 49.5. So my car can start smoothly.
* When close other vehicle, my car will decrease velocity slow down.
* With acceleration is 5m/s2. So driver will feel comfortable.

###### 1.5.3 Create planning path base on above spline

File `main.cpp` line `808-811` and `532-575`

I create 50 points for moving. Some of them come from the previous data and I will create the rest part.

![alt text][image5]

To make the car run with a constant velocity, I split the distance line (green line) into N part.

```cpp
  double ref_vel_step = ref_vel*0.02;
  double N = target_dist/ref_vel_step;
```

After that, I predict N points x and calculate N points y: `y = s(x)`.

When the target point near the start point, the distance line (green line) will approximate to the actual line (yellow line). So the actual velocity will approximate to the predict velocity.

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

---

### References:

* https://www.youtube.com/watch?v=7sI3VHFPP0w
* http://kluge.in-chemnitz.de/opensource/spline/

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.

##### Techniques:

* .

##### Fail cases:

* .

##### Improve:

* .
