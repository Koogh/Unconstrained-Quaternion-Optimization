# Simple Example for 3D Relative Pose Estimation Using Two 3D Points

This repository contains an example source code for estimating the 3D relative pose between two 3D point sets.
The code is developed to help understanding the study (VMV, 2001) by  J.Schmidt and H. Niemann.

Generally, the 3D pose is represented as 1x7 vector, [tx, ty, tz, qw, qx, qy, qz]. And the quaternion elements (4 elements) in the 1x7 vector is estimated not only by the LM method but also the additional normalization. However, J.Schmidt and H. Niemann have proposed the unconstrained optimization method for quaternion estimation only using 3 elements. The quaternion (1x4 vector) is converted into a vector (1x3 vector) on its hyperplane, and the target pose could be represented as 1x6 vector, [tx, ty, tz, v1,v2, v3]. This would result in the compuational effectiveness in various robotics and computer vision applications.  


## Requirement and Dependency

- Matlab 
- Optimization Toolbox (by Matlab)
(cf. I have tested Matlab 2019 and  Matlab 2020 )


## How to Use

- Run (Press 'F5') the 'Example_3d_pose_estimation.m'


## Reference Paper

@inproceedings{schmidt2001using,
  title={Using Quaternions for Parametrizing 3-D Rotations in Unconstrained Nonlinear Optimization.},
  author={Schmidt, Jochen and Niemann, Heinrich},
  booktitle={Vmv},
  volume={1},
  pages={399--406},
  year={2001},
  organization={Citeseer}
}