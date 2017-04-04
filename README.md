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

Results of running the program on the `sample-laser-radar-measurement-data-1.txt` file should give the following output:

```
 RMSE
 0.0651649
 0.0605378
   0.54319
  0.544191
``` 


Results for `sample-laser-radar-measurement-data-2.txt`:

```
RMSE
0.185496
0.190302
0.476754
0.804469
```

## Visualization

A Jupyter notebook for visualization of results is included together with the project. It's going to plot
ground values vs. predicted values for selected output files.

![Sample 1](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sample-laser-radar-measurement-data-1.png)

It can be noticed there is some overestimation in the shown section:

![Sample 1 Zoom](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sample-laser-radar-measurement-data-1_zoom.png)

For sample 2, these are the results:

![Sample 2](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sample-laser-radar-measurement-data-2.png)


## Tweaks 

After changing the x and y noise components from 9 to 25 each, the RMSE for the first sample is significantly lower:

```
RMSE
0.0416684
0.0393652
 0.444191
 0.471881
```

The improvement can also be noted in the same highlighted section, where the overestimation is also lower: 
![Second noise value on sample 1 zoom](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sample-laser-radar-measurement-data-1_2nd_noise.png)

At the same time, using this value RMSE becomes worse for sample 2, especially when regarding the velocity components:

```
RMSE
0.186466
 0.19127
0.523013
 1.07051
```

In practice, the estimation still seems to be very good but given the accuracy requirements I went back to the original
 values of 9, 9 for the noise in x and y components.

## Simulator
Output of the program was changed so that it outputs RMSE in the format used by the simulator.
The `kalman-tracker.py` which can be used together with the simulator is included.

These are the graphed measurements and estimates vs ground truth with Lidar and Radar enabled:

![Simulator](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/newplot.png)

These are the results as seen on the simulator, with lidar and radar data first:

![Lidar and Radar](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sim_lidar_and_radar.png)

While these are the results using only lidar:

![Lidar Only](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sim_lidar.png)

And only radar:

![Radar Only](https://github.com/ottonello/CarND-Extended-Kalman-Filter-Project/raw/master/out/sim_radar.png)

