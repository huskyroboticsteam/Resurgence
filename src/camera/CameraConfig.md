# Camera Configuration {#cameraconfig}
<!--   File is intended to be processed by Doxygen and may contain   -->
<!--   Doxygen-specific syntax that is not valid Markdown.           -->

Cameras are configured using the following configuration file format.  The
specification given is in general terms, and the [example](@ref example) is
given in YAML; however, XML and JSON are also supported. Please see the OpenCV
documentation on [persistent file
storage](https://docs.opencv.org/4.2.0/d4/da4/group__core__xml.html) for more
details.

### Top-Level Format {#toplevel}

The top level of the configuration file should be a key-value mapping, with the
following top-level keys:
  * `name`: The name of the camera, e.g. "mast" or "front_left". No restrictions
    on naming or uniqueness are currently enforced; ideally this should be
    unique but it isn't strictly necessary at this time.
  * `description`: _Optional._ A longer description of the camera, including
    details like its location on the rover.
  * *One* of the following. **If both are present, `filename` will take precedence.**
   	  * `filename`: The file path or URI to a video device or stream that should
      be opened.
	  * `camera_id`: The ID of a video device that will be opened. Given ID N, the
      device at `/dev/videoN` will be opened.
  * `intrinsic_params`: _Optional._ Value should be a nested key-value mapping,
   defining a set of intrinsic camera parameters as defined in the section
   [Intrinsic Parameters](@ref intrinsic) below.
  * `extrinsic_params`: _Optional._ Should be a 4x4 matrix defining a
    transformation from 3-dimensional coordinates in the camera's
    frame of reference to the rover's frame of reference. This matrix
    should ideally have a 3x3 [rotation
    matrix](https://en.wikipedia.org/wiki/Rotation_matrix#In_three_dimensions)
    in the first three rows and columns, and the first three rows in
    the fourth column should be a translation vector.  The fourth row
    should be `0, 0, 0, 1` so that [homogeneous
    coordinates](https://en.wikipedia.org/wiki/Homogeneous_coordinates)
    (4-dimensional coordinates where the 4th coordinate is `1`) can be
    used.

	Note that in the rover's frame of reference, the positive
    x-axis is towards the front, the positive y-axis is towards the
    left, and the positive z-axis is towards the top; in the camera's
    frame of reference, the positive x-axis is to the right (parallel
    to the image plane), the positive y-axis is down, and the positive
    z-axis is in the direction the camera is pointing.
  * `calib_info`: _Optional._ Information from the calibration process, generated
   by the calibration program. See [Calibration Info](@ref calibinfo) below.
   
### Intrinsic Parameters {#intrinsic}

Intrinsic parameters are represented as a key-value mapping, with the following
keys: 
  * `camera_matrix`: A 3x3 matrix defining the projection of a 3D scene onto the
    2D image plane of the camera.
  * `distortion_coefficients`: A column vector (i.e. matrix with one column) of
    4, 5, 8, 12, or 14 distortion coefficients, which define the lens distortion
    of the camera.
  * `image_width`: The width of the image in pixels. This (along with
    `image_height`) will define the resolution the camera is set to, as
    intrinsic parameters are only valid for the resolution at which they are
    calibrated.
  * `image_height`: The height of the image in pixels.
  
### Calibration Info {#calibinfo}

Calibration information is saved by the calibration program, and provides some
data relevant to the calibration process, such as the time and date of
calibration, the size of the board used, and the reprojection error. You should
not need to fill this section of the configuration file out yourself.

Probably the most important part of this section if it is present is
`avg_reprojection_error`. The calibration process computes a projection matrix
for the camera; multiplying the coordinates of a point in space as a column
vector by this matrix will give the location they should appear in the
image. The reprojection error is the distance between the computed projection
and the actual location of the point in the image, and is measured in pixels;
the lower this number, the better. A general rule of thumb is that you should
try to aim for less than 1 pixel.

### Example {#example}

```yaml
%YAML:1.0
---
name: "example_camera"
description: "An example camera, not actually located on the rover."
# Camera ID: given ID N, will attempt to use /dev/videoN.
camera_id: 0
# Information about the calibration process, such as the
# reprojection error and board size.
calib_info:
   calibration_time: "Sat 08 May 2021 12:19:35 PM PDT"
   board_width: 10
   board_height: 7
   square_size: 21.
   avg_reprojection_error: 1.8641131183947526e-01
   flags: 0
# The actual intrinsic parameters of the camera.
intrinsic_params:
   camera_matrix: !!opencv-matrix
      rows: 3
      cols: 3
      dt: d
      data: [ 6.4859611612740241e+02, 0., 3.2923686348750090e+02, 0.,
          6.5023092591714169e+02, 2.4355655685247874e+02, 0., 0., 1. ]
   distortion_coefficients: !!opencv-matrix
      rows: 5
      cols: 1
      dt: d
      data: [ -4.1887226549228418e-01, 1.6973679483281634e-01,
          8.8881372228211163e-04, 9.6518575595955756e-04, 0. ]
   image_width: 640
   image_height: 480
```
