# Dense 3D Reconstruction from Stereo

This is a ROS package for real-time 3D reconstruction from stereo images. Currently this version uses [LIBELAS](http://www.cvlibs.net/software/libelas/) for generating dense disparity maps as a baseline. The method for generation of disparity maps can be changed based on user preferences.

This package serves as a visualization tool for dense disparity maps and point clouds. Additionally, a tool for transforming point clouds to a different reference frame is also included. 

Usually, the point clouds are formed in the reference frame of the left camera. For ground robots, often the point clouds need to be transformed to a different frame *e.g.*, a reference frame with the origin at the centre of rotation of the robot projected on to the ground plane. These transformations are hard to calculate mathematically - this tool can be used to find the transformations visually.

## Dependencies

- OpenCV 2.4.8+ [(https://github.com/opencv/opencv)](https://github.com/opencv/opencv)
- ROS (Indigo or later)

## Stereo Calibration

A calibrated pair of cameras is required for stereo rectification and calibration files should be stored in a `.yml` file. [This repository](https://github.com/sourishg/stereo-calibration) contains all the tools and instructions to calibrate stereo cameras.

The rotation and translation matrices for the point cloud transformation should be named as `XR` and `XT` in the calibration file. `XR` should be a **3 x 3** matrix and `XT` should be a **3 x 1** matrix. Please see a sample calibration file in the `calibration_files/` folder.

## Stereo Rectification

Run this node to rectify stereo images

```bash
$ rosrun stereo_dense_reconstruction stereo_rectify [/camera/left/topic] [/camera/right/topic] [path/to/calib/file.yml]
```

This node produces rectified stereo images on the following topics:

- `/camera_left_rect/image_color`
- `/camera_right_rect/image_color`

## Dense 3D Reconstruction

```bash
$ rosrun stereo_dense_reconstruction dense_reconstruction [path/to/calib/file.yml]
```

This node outputs the dense disparity map as a grayscale image on the topic `/camera_left_rect/disparity_map` and the corresponding point cloud on the topic `/camera_left_rect/point_cloud`.

## Point Cloud Transformation

The point cloud can be viewed on `rviz` by running

```bash
$ rosrun rviz rviz
```

To transform the point cloud to a different reference frame, the `XR` and `XT` matrices (rotation and translation) in the calibration file need to be changed. This can be done real-time by the running

```bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

Modify the euler angles and the translation vector to get the desired transformation.