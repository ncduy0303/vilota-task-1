# Vilota Task 1

## Setup

Install the required packages using the following command:

```bash
pip install -r requirements.txt
```

## Usage

### Part 1 (Warm-up): Expressing Spatial Information Mathematically

In the notebook `spatial_math.ipynb`, I have used the `spatialmath-python` package to visualize some simple transformation matrices in 3D space. The matrices of interest are in the group `S03` and `SE3`, which are used to represent rigid body motions.

Some mathematical understanding can be helpful to understand the transformations, which can be learnt from this YouTube playlist: [Modern Robotics, Chapter 3: Rigid-Body Motions](https://youtube.com/playlist?list=PLggLP4f-rq01NLHOh2vVPPJZ0rxkbVFNc&si=LNpQ8w3zCcSAEbCw).

To learn more about the `spatialmath-python` package, their [GitHub repository](https://github.com/bdaiinstitute/spatialmath-python/tree/master) has a very good introductory notebook: [notebooks/gentle-introduction.ipynb](https://github.com/bdaiinstitute/spatialmath-python/blob/master/notebooks/introduction.ipynb).

Below is a recording of running the notebook:

A recording of running the notebook is available as `spatial_math_usage.mov`

### Part 2: Visualizing a Camera Wireframe in Open3D

The main goal is to create an animation similar to this YouTube video: [SenseTime B4](https://youtu.be/fg_7IcZ39Oc?si=zOnXdubGaUe5RFMw).

The script `open3d_animation.py` creates a pyramid wireframe to represent a camera. The animation is created by rotating the camera around the a target object in space. It uses the `open3d` package to create the 3D visualization and animation. The animation is interactive so you can rotate, zoom and pan the camera using the mouse while the animation is paused.

To learn more about the `open3d` package, their [GitHub repository](https://github.com/isl-org/Open3D/tree/main) has a lot of examples: [examples/python](https://github.com/isl-org/Open3D/tree/main/examples/python).

A recording of running the script is available as `open3d_animation_usage.mov`
