## CarND Path Planning Project

The goals / steps of this project are the following:

* In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

[//]: # (Image References)
[image1]: ./output_images/compilation.png
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

##### 1.1 Check near car in all lanes

To drive our car safety, I will check others car in all lanes with dangerous range is from -10 meters to +30 meters compare with our position.

##### 1.2 Choose the best lane

This is planning's heart.

##### 1.3 Control speed

Start speed value is 0 mph.

Max speed is 49.5 mph.

Speed increase/decrease with step value is 0.224 mph (acceleration = 5m/s2)

##### 1.4 Create spline

Choose 5 points to make our spline.

* 2 previous points
* 3 next points, far from us 30, 60, 90 meters

##### 1.5 Create planning path base on above spline

I create 50 points for moving.

---

### References:

* https://www.youtube.com/watch?v=7sI3VHFPP0w

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
