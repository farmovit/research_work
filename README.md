# Description

This research is devoted to the development of a bearings-only tracking algorithm for a system whose bearings dynamic has the following form:

![bearings_dynamic](https://)

There is an example in the folder [test_data](http://FIXME) of the data shown in the picture. The peculiarity is that these data are going into the input of the algorithm in real time and the goal of the algorithm is to smooth out noised data in real time.

To do it I use linear filtering algorithms like alpha-beta, alpha-beta-gamma, Kalman filter. You can see how, for example, Kalman filter works:

![example](https://)

In this repository I rendered only a small chunk of filtering algorithms that I tried.

# Building

For build this program run script build.bat in Windows and ./build.sh in Linux.

# Running

For run an example run script run_single.bat

