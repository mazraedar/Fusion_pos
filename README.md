Function for sensors fusion using KalmanFilter class by Hayk Martirosyan
========================================================================

Fusion define/update the state system of a free object (3-DOM) from the time from the last measurement, the measurements and the measurment matrix C, and the former state.

The state is x = x ; y ; z;

define_measurement_matrix define two measurements matrixes C_hl and C_IMU.

The angle is given by the IMU fusion algorithm.
