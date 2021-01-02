# Robotics-Kinematics

## Forward Kinematics

The following figure is the forward kinematics

![Sketch](https://user-images.githubusercontent.com/21059011/103450870-6514ec80-4c8a-11eb-92f9-6359a1cd9598.png)

## Inverse Kinematics

The index of function *InverseKinematics(O_LeftLeg, K_LeftLeg)* can decide which arm (or leg) to approach the target.

The following table is the result of inverse kinematics for the right arm.

The origin positions are [ 7.75  0  -10  1 ], and the targets are [ 5  3  -4  1 ].

Learning rate is 0.001.

Iteration | x | y | z
----------|---|---|-----------------
1 | 4.79785 | 5.19368 | -9.05842
100 | 4.07364 | 4.29091 | -8.53099
200 | 4.96027 | 3.50629 | -5.25026
