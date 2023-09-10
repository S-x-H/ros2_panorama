# ros2_panorama

ros2 iron python package to stitch 3 images into a single panorama


## BUILDING

Create the top level workspace folder.

`mkdir ros2_panorama`

Within the workspace, create the src folder.

`cd ros2_panorama`

`mkdir src`

Within the src folder, clone the repo.

`cd src`

`git clone https://github.com/S-x-H/ros2_panorama.git`

Return to src folder and install all requirements.

`cd ..`

`rosdep install --from-paths ros2_panorama -y --ignore-src`

`pip install -r ros2_panorama/requirements.txt`

Return to top level workspace folder and build.

`colcon build`

Source setup.

`source install/setup.sh`


## CONFIGURING

The image_merger node has the following configuration parameters:

| Name                     | Type   | Description                                                                                                                 | Default | Dynamically Reconfigurable? |
| ------------------------ | ------ | --------------------------------------------------------------------------------------------------------------------------- | ------- | --------------------------- |
| display_img_window       | bool   | True to display panorama image being published in window.                                                                   | False   | yes                         |
| camera_sync_tolerance    | bool   | Maximum time difference (s) allowed when matching images from different cameras to be considered a single frame.            | 0.01    | no                          |
| camera_calibrations_path | string | Full filepath to .json containing all camera calibration values. Can be left blank to skip undistortion.                    | ""      | no                          |
| left_camera_name         | string | When doing undistortion: name of left camera in camera calibration json. Can be left blank if calibrations path is blank.   | ""      | no                          |
| centre_camera_name       | string | When doing undistortion: name of centre camera in camera calibration json. Can be left blank if calibrations path is blank. | ""      | no                          |
| right_camera_name        | string | When doing undistortion: name of right camera in camera calibration json. Can be left blank if calibrations path is blank.  | ""      | no                          |
| attempt_stitching        | bool   | True to attempt image stitching instead of mere stacking, will still stack if stitching fails.                              | True    | yes                         |
| stitching_detector       | string | When doing stitching: 'orb', 'sift', 'brisk', or 'akaze'.                                                                   | "orb"   | yes                         |
| stitching_threshold      | float  | When doing stitching: feature matching threshold between 0 - 1.                                                             | 0.2     | yes                         |
| stitching_timeout        | float  | To continue running in real-time, length(s) to attempt stitching per frame before giving up, if stitching is slow.          | 0.5     | yes                         |

Note that stitching will perform poorly or fail completely if images have little overlapping area, or a low number of features. Stacking will be performed anyway if stitching fails. It may be desirable to disable stitching if it succeeds but returns incorrect results, or if it repeatedly fails and slows down processing.

Parameters can be edited by changing their defaults in the [launch file](launch/launch.py), or via the command line upon [launch](#launching). The dynamically reconfigurable parameters can be changed at runtime using rqt or by command line.

`ros2 param set /image_merger <parameter_name> <parameter_value>`

The node also has the following topics for remapping:

| Name            | Type              | Publisher/Subscriber |
| --------------- | ----------------- | -------------------- |
| "camera_left"   | sensor_msgs/Image | Subscriber           |
| "camera_centre" | sensor_msgs/Image | Subscriber           |
| "camera_right"  | sensor_msgs/Image | Subscriber           |
| "panorama"      | sensor_msgs/Image | Publisher            |


## LAUNCHING

`ros2 launch ros2_panorama launch.py`

Or with parameters:

`ros2 launch ros2_panorama launch.py attempt_stitching:=True stitching_timeout:=0.3`

To test/view results, open rviz2 and subscribe to the "panorama" topic, or set display_img_window:=True.