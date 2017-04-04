# Extended Kalman Filter Project Starter Code
Self-Driving Car Engineer Nanodegree Program

---

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

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./ExtendedKF path/to/input.txt path/to/output.txt`. You can find
   some sample inputs in 'data/'.
    - eg. `./ExtendedKF ../data/sample-laser-radar-measurement-data-1.txt output.txt`

## Results




## Simulator
Output of the program was changed so that it outputs RMSE in the format used by the simulator.
The `kalman-tracker.py` which can be used together with the simulator is included.

These are the results as seen on the simulator, with lidar and radar data first:

![Lidar and Radar](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sim_lidar_and_radar.png)

While these are the results using only lidar:

![Lidar Only](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sim_lidar.png)

And only radar:

![Radar Only](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sim_radar.png)

