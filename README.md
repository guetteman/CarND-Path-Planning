# Self-Driving Car Path Planning

![alt text][image1]

---

[//]: # (Image References)

[image1]: ./images/main.png "Path planning"

## This is the first project of term 3 of self-driving cars engineer nanodegree.
In this project we will safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. It will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway.
   
### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Reflection

Based on the provided code, we will explain with detail the process. The project has three parts:

- **Prediction:** In this part we have to gather data from **sensor fusion** and **telemetry** to know if We have:

    - A car in front of us.
    - A car on our right.
    - A car on our left.

```cpp
for (int i = 0; i < sensor_fusion.size(); i++) {
            
            float d = sensor_fusion[i][6];
            

            if (d > 0 && d < 4) {
                car_lane = 0;
            } else if (d > 4 && d < 8) {
                car_lane = 1;
            } else if (d > 8 && d < 12) {
                car_lane = 2;
            } else {
                continue;
            }

            double vx = sensor_fusion[i][3];
            double vy = sensor_fusion[i][4];
            double check_speed = sqrt(vx*vx + vy*vy);
            double check_car_s = sensor_fusion[i][5];
            check_car_s += ((double)prev_size*0.02*check_speed);

            if (car_lane == lane) {
                if ((check_car_s > car_s) && ((check_car_s - car_s) < 30)) {
                    is_car_ahead = true;
                }
            } else if (car_lane - lane == -1) {
                if (car_s - 10 < check_car_s && car_s + 40 > check_car_s) {
                    is_car_left = true;
                }
            } else if (car_lane - lane == 1) {
                if (car_s - 10 < check_car_s && car_s + 40 > check_car_s) {
                    is_car_right = true;
                }
            } 
        }
```

We define a safe distance of **30 meters** for cars ahead of us, for cars on the sides, we define **40 meters** (this will help to check what could the best option to make a lane change) on the front and **10 meters** behind of us.

Based on `is_car_ahead`,  `is_car_left`, `is_car_right` we decide what to do in the next phase.

- **Behavior:** Once We know more about our environment, we take decisions so our car can move through the highway as fast as it can without break the rules.

```cpp
if (is_car_ahead) {
                
    ref_vel -= 0.224;

    if (!is_car_left && lane > 0) {
        lane -= 1;
    } else if (!is_car_right && lane < 2) {
        lane += 1;
    }

} else if (ref_vel < 49.5) {
    ref_vel += 0.224;
}

if (!is_car_ahead && car_lane != 1) {
    if (!is_car_left && lane == 2) {
        lane -= 1;
    } else if (!is_car_right && lane == 0) {
        lane += 1;
    }
}
```

- When the car is ahead we reduce the velocity and check if we have cars on both sides to see if we change lane.
- If we don't have cars ahead, we increase velocity until 49.4MPH which is almost the max velocity allowed.
- Finally, We try to maintain our car on the center lane as long as we can, because we will have more options to move to another lane if we find a car ahead.

This will take us to the last phase.

- **Trayectory:** In this phase, based on the behavior that we want, we define a trayectory which will be followed by the car.

```cpp
vector<double> ptsx;
vector<double> ptsy;

double ref_x = car_x;
double ref_y = car_y;
double ref_yaw = deg2rad(car_yaw);

if (prev_size < 2) {

    double prev_car_x = car_x - cos(car_yaw);
    double prev_car_y = car_y - sin(car_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(car_x);
    
    ptsy.push_back(prev_car_y);
    ptsy.push_back(car_y);

} else {

    ref_x = previous_path_x[prev_size - 1];
    ref_y = previous_path_y[prev_size - 1];

    double ref_x_prev = previous_path_x[prev_size - 2];
    double ref_y_prev = previous_path_y[prev_size - 2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);
    
    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);

}

vector<double> next_wp0 = getXY(car_s + 30, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp1 = getXY(car_s + 60, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
vector<double> next_wp2 = getXY(car_s + 90, (2 + 4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

ptsx.push_back(next_wp0[0]);
ptsx.push_back(next_wp1[0]);
ptsx.push_back(next_wp2[0]);

ptsy.push_back(next_wp0[1]);
ptsy.push_back(next_wp1[1]);
ptsy.push_back(next_wp2[1]);

for (int i = 0; i < ptsx.size(); i ++) {
    
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
    ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));

}

tk::spline s;

s.set_points(ptsx, ptsy);

vector<double> next_x_vals;
vector<double> next_y_vals;

for (int i = 0; i < previous_path_x.size(); i++) {
    
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);

}

double target_x = 30.0;
double target_y = s(target_x);
double target_dist = sqrt(target_x*target_x + target_y*target_y);

double x_add_on = 0;

for (int i = 0; i < 50 - previous_path_x.size(); i++) {
    
    double N = (target_dist/(0.02*ref_vel/2.24));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);

}
```

- First, we check the previous path size, and if it is almost empty, we will use the current car state. If we have more points, we get the car last state and the previous one. 
- In Frenet we add three spaced points each 30m as an starting point. 
- We shift car reference angle to 0 degrees.
- We define a trayectory function with `spline` library with the current state.
- If we have previous points, we add those points to the path planner. Then, we just add `50 - previous_path_x.size()` new points which will define the future path.

## Finally, the car will follow the route that we define!