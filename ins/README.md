# pony-ins-plugin-set

## Overview

Plug-ins for [pony](https://github.com/p-o-n-y/pony) core library, that target inertial navigation aspects:

| Lib name              | Plugin list                                                                 | Short description                                       |
| -------------------   | --------------------------------------------------------------------------- | ------------------------------------------------------- |
| `pony_ins_gravity`    |                                                                             | Gravity model calculations:                             |
|                       | [`pony_ins_gravity_constant`           ](#pony_ins_gravity_constant)        | constant gravity value from accelerometers on alignment |
|                       | [`pony_ins_gravity_normal`             ](#pony_ins_gravity_normal)          | conventional Earth normal gravity model                 |
|                       | [(planned) `pony_ins_gravity_egm08`    ](#pony_ins_gravity_egm08)           | planned for future development                          |
| `pony_ins_alignment`  |                                                                             | INS initial alignment:                                  |
|                       | [`pony_ins_alignment_static`           ](#pony_ins_alignment_static)        | sensor averaging on a static base                       |
|                       | [`pony_ins_alignment_rotating`         ](#pony_ins_alignment_rotating)      | gravity vector approximation in inertial reference      |
|                       | [`pony_ins_alignment_rotating_rpy`     ](#pony_ins_alignment_rotating_rpy)  | same, but matrix is calculated via roll, pitch and yaw  |
| `pony_ins_attitude`   |                                                                             | Angular rate integration:                               |
|                       | [`pony_ins_attitude_rodrigues`         ](#pony_ins_attitude_rodrigues)      | via Euler vector using Rodrigues' formula               |
|                       | [(planned) `pony_ins_attitude_madgwick`](#pony_ins_attitude_madgwick)       | planned for future development                          |
| `pony_ins_motion`     |                                                                             | Position and velocity algorithms:                       |
|                       | [`pony_ins_motion_euler`               ](#pony_ins_motion_euler)            | modified Euler integration                              |
|                       | [(planned) `pony_ins_motion_sculling`  ](#pony_ins_motion_sculling)         | planned for future development                          |
|                       | [`pony_ins_motion_vertical_damping`    ](#pony_ins_motion_vertical_damping) | vertical error buildup damping                          |
|                       | [`pony_ins_motion_size_effect`         ](#pony_ins_motion_size_effect)      | accelerometer separation compensation                   |


## Detailed descriptions

### `pony_ins_gravity_constant`

Takes the magnitude of average accelerometer output vector and puts it into vertical gravity component, 
which then remains constant. Performs calculations for each initialized IMU device.

Recommended for low-grade systems, especially when no reference coordinates available.

### `pony_ins_gravity_normal`

Computes conventional Earth normal gravity model as in GRS80, etc., 
but takes Earth model constants from pony->imu_const variables.  
Accounts for both latitude and altitude, as well as for plumb line curvature above ellipsoid.  
Performs calculations for each initialized IMU device.

Recommended for conventional navigation grade systems.

### (planned) `pony_ins_gravity_egm08`

Planned for future development.


### `pony_ins_alignment_static`

Conventional averaging of accelerometer and gyroscope outputs.
Then constructing the attitude matrix out of those averages.  
Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.  
Sets velocity vector to zero, as it is assumed for this type of initial alignment.  
Performs the alignment for all initialized IMU devices.

Recommended for navigation/tactical-grade systems on a highly stable static base,
e.g. turntable or stabilized plate.

### `pony_ins_alignment_rotating`

Approximation of ground reaction force rotating along with the Earth in an inertial reference frame.
Then estimating northern direction via ground reaction force displacement.  
Also calculates attitude angles (roll, pitch, yaw=true heading), and quaternion.  
Performs the alignment for all initialized IMU devices.  
Sets velocity vector to zero and its validity flag up.

Recommended for navigation-grade systems on a rotating base, 
allowing vibrations with zero average acceleration,
e.g. an airplane standing still on the ground with engine(s) running.

### `pony_ins_alignment_rotating_rpy`

The same as `pony_ins_alignment_rotating`, but attitude matrix is calculated using attitude angles
(roll, pitch and yaw=true heading). Does not allow the first instrumental axis to point upwards.  
This plugin is a legacy one, not worked on anymore.  
Sets velocity vector to zero.

Recommended for navigation-grade systems on a rotating base, 
allowing vibrations with zero average acceleration,
with pitch angle not exceeding 80°.


### `pony_ins_attitude_rodrigues`

Combines angular rate components or their integrals into Euler rotation vector
for both instrumental frame and navigation frame (according to rate validity). 
Then uses Rodrigues' rotation formula for the transition matrix between them.  
For proper rotation, compensates for coning by using previous rotation vector in cross-product
with the current one, implying linear angular velocity approximation in time.  
Quaternion and angles are also updated.
Performs attitude integration step for each initialized IMU device with valid current attitude matrix.

Recommended for navigation/tactical grade systems.

### (planned) `pony_ins_attitude_madgwick`

Planned for future development.


### `pony_ins_motion_euler`

Numerically integrates Newton's second Law in local level navigation frame 
using modified Euler's method (with midpoint for attitude matrix) over the Earth reference ellipsoid.  
Updates velocity and geographical coordinates for each initialized IMU device 
with valid accelerometer measurements, gravity vector, and current position, velocity and attitude matrix.  
At the Earth's poles, transforms to azimuth wandering reference frame.

Not suitable at outer-space altitudes, and/or over-Mach velocities. Note that the plugin initializes
coordinates and velocities, and raises their validity even if default values are used.


### `pony_ins_motion_vertical_damping`

Restrains vertical error exponential growth by either damping
vertical velocity to zero, or to an external reference like
altitude/rate derived from air data system and/or gnss, etc.  
Preforms the damping for all initialized IMU devices.

Recommended when using normal gravity model and/or long navigation timeframe.

### `pony_ins_motion_size_effect`

Compensates for accelerometer cluster size effect, i.e. the spatial separation 
between accelerometer proof masses, also known as "accelerometer lever arm compensation".
Uses angular rate and computed angular acceleration, taken as rate increment over the time step.  
Preforms the compensation for all initialized IMU devices with accelerometer positions specified.

Recommended only for high-grade systems on dynamically maneuvering carriers.