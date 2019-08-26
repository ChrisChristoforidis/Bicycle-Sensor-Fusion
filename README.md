# Bicycle-Sensor-Fusion
Compute orientation of bicycle from inertial measurements.

The code used to provide an estimation of the roll and yaw angle for the steer by wire bicycle of TU Delft.
The output from euler integration is fused with a set of pseudo absolute measurments to account for the integration drift.
For the roll angle the absolute measurment  taken used the yaw rate and the forward velocity, while for the yaw the magnetometer is used to provide an estimate  relative to the Magnetic north. It also it provides an option to sensor fuse using a much more computationally cheap complimentary filter and a more robust but computationally expensive kalman  filter. 
