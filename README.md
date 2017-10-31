# Self-Driving Car Engineer Nanodegree Program
## Path-Planning - Project 1

<p align="center">
    <img src="./imgs/img.png" width="960">
</p>

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

## Project Description

In this project, your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also, the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

## The car is able to drive at least 4.32 miles without incident.

The car exceeds expectations by achieving **`11.53 miles`** as shown in the screenshot above. 

## The car drives according to the speed limit.

To meet these criteria I have implemented a speed controller. Whenever the distance between the ego car and the car in front of it drops below the acceptable value the reference velocity would be decreased slowing down the ego car.
```
    if ((check_car_speed > car_s) && (car_gap < 27)) {
        too_close = true;
        ref_vel -= 0.184; // slowing down if too close
        std::cout << " Too Close: " << car_gap 
        << " Ref_Vel: " << ref_vel
        << std::endl;
```  
## Max Acceleration and Jerk are not Exceeded.

To counter the loss of speed caused by slow traffic ahead and to maintain the overall speed around the track I have added an algorithm that would gradually increase the speed as the reference velocity drops below a certain value (spaced out in small increments to prevent extreme acceleration).

```
    } else if (ref_vel < 10) { 
    ref_vel += 0.682;
    } else if (ref_vel < 20) {
    ref_vel += 0.442;
    } else if (ref_vel < 30) {
    ref_vel += 0.210;
    } else if (ref_vel < 40) {
    ref_vel += 0.185;
    } else if (ref_vel < 45) {
    ref_vel += 0.105;
    } else if (ref_vel < 49.4) {
    ref_vel += 0.098;
    }
```
## Car does not have collisions.

This very important criterion is met by implementing a lane safety value. Only if specific conditions are met will the ego car choose to merge. To make this feature more efficient I plan to add a cost function in the future. Calculating the cost of merging versus staying behind a slow vehicle.
```
    if (too_close) { 
        int left_lane = lane - 1;
        if (lane>0 && merge_left_possible && d<(2+4*left_lane+2) && d>(2+4*left_lane-2)) {
            if ((check_car_speed > car_s) && ((car_gap < 30)) || // if either of these conditions are not met
            (check_car_speed < car_s) && (car_gap_rear < 40)) {  // merging is not possible    
                    merge_left_possible = false;

        int right_lane = lane + 1;
    if (lane<2 && merge_right_possible && d<(2+4*right_lane+2) && d>(2+4*right_lane-2)) {
        if ((check_car_speed > car_s) && ((car_gap < 30)) || // if either of these conditions are not met
        (check_car_speed < car_s) && (car_gap_rear < 40)) {     // merging is not possible    
                merge_right_possible = false;
```
## The car stays in its lane, except for the time between changing lanes.

This criterion is met by controlling a number of lanes the ego car can change per turn. 
```
    if(lane>0 && merge_left_possible && (switch_lanes % 1 == 0)){            
        std::cout << " Merging <-<-< "
         << " Ref_Vel: " << ref_vel
         << " Speed: " << car_speed
         << std::endl;                
        lane -= 1;
        switch_lanes += 1;

    } else if(lane<2 && merge_right_possible && (switch_lanes % 1 == 0)){
        std::cout << " Merging >->-> "
        <<  " Ref_Vel: " << ref_vel
        << " Speed: " << car_speed
        << std::endl;
        lane += 1;
        switch_lanes += 1;
```
Every cycle the ego car will decide if its speed will be greater then the speed of the car in fornt of it. this allows for quick decision making and ease of use.

## The car is able to change lanes.

This criterion is met by the same algorithms mentioned above. 
