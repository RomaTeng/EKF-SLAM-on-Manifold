# EKF-SLAM-on-Manifold

------------------------
This is a joint work by Teng Zhang, [Kanzhi Wu](kanzhi.me), Shoudong Huang and Gamini Dissanayake.

The document is the paper "Convergence and Consistency for An Invariant-EKF 3D SLAM".
Fortunately, this paper has been accepted by Robotics and Automation Letters.



__This repository is for code release only!__
__Under construction__
__This repository is for R-EKF(right_ekf_3d), FEJ-EKF(f_ekf_3d) and T-EKF(ekf_3d) code ONLY__


Simulation settings can be tuned into "./datagen_3d":
(1) simulated trajectory:       gen_trajectory.m
(2) noise level, number of landmarks, FOV: config.m




If you want to test a simple case:
for example, 
(1)   run "./datagen_3d/gen_data.m" to generate "data.mat"(all measurements and ground truth) 
(2)   copy this "data.mat" into the foler "./right_ekf_3d.m" or "./f_ekf_3d.m" or "./ekf_3d.m"
(3)   run "REKF_SLAM.m" or "FEKF_SLAM.m" or "EKF_SLAM.m"    and get the simulation test


If you want to test a simple test for three filters:
for example, 
(1)   run "./comparison_3d.m" and the comparsion will be displayed


If you want to perform Monte Carlo test for three filters:
for example, 
(1)   run "./multi_comparison_3d.m" and the comparsion will be displayed
