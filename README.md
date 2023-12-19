# seam_detection
This is a ROS package for weld seam detection from pointclouds using the point cloud library [PCL](https://pointclouds.org/).

## Publications:
```
Automated Weld Path Generation Using Random Sample Consensus and Iterative Closest Point Workpiece Localization
Proceedings of the ASME 2022
International Design Engineering Technical Conferences and
Computers and Information in Engineering Conference
IDETC/CIE2022 August 14-17, 2022, St. Louis, MO
```

### Installing seam_detection

#### Requirements:
##### Operating System:
- Ubuntu 20.04 - current 
- Ubuntu 18.04 - previous (see tag v1.x)
##### ROS Version (linked to OS):
- ROS Noetic - current 
- ROS Melodic - previous
##### Hardware
- no hardware requirements identified currently
- RTX 3070 - GPU required to run seam_detection in Docker  
##### Graphics
- `nvidia-525` current 
- `nvidia-4xx` previous


#### Step 1 - Setup ROS workspace
Create and build a catkin workspace before proceeding. Choose a location and insert a name for the workpace. Typically this is somewhere in `~/`.

```
mkdir -p ~/<workspace-name>/src
cd ~/<workspace-name>
catkin_make
```

Add the workspace path to `/.bashrc` so that ros can find your packages
```
echo "source ~/<workspace-name>/devel/setup.bash" >> ~/.bashrc
```

To use a catkin workspace that is already setup, skip **Step 1** (the workspace must compile with `catkin_make`).


#### Step 2 - Download seam_detection Package
Change to the source directory of the workapce and clone the package using git.

```
cd ~/<workspace-name>/src
git clone https://github.com/thillRobot/seam_detection.git
```

or using SSH (used by thillrobot)
```
git clone git@github.com:thillrobot/seam_detection.git
```

#### Step 3 - Compile seam_detection in catkin workspace
Change to top of workspace and compile.

```
cd ..
catkin_make
```
The workspace and package should compile without errors.


### Using seam_detection

#### Primary Nodes 
 (Early work 2020-2021, needs testing)
 - `ransac_plane` - use RANSAC to fit models to a pointcloud
 - `segment_plane` - use RANSAC to separate a pointcloud into multiple planes 
 - `seam_detection_RANSAC` - use RANSAC and the intersection of planes to locate a seam 
 - `seam_detection_icp` - ! this node has been removed and replaced with `seam_detection` !

 (Current work 2022-present)
 - `seam_detection` - use filtering, voxel downsamplings, RANSAC segmentation to prepare pointclouds for iterative closest point registration 

#### Supporting Nodes 

 - `scan2cloud` - generate pointclouds from lidar scans and poses (node missing from repo, coming back soon)   
 - `cad_cloud` - convert .ply file into .pcd file using pcl
 - `cad_cloud_bulk` - convert directory of .ply files into .pcd files using pcl
 - `rotate_cloud` - apply homogenous transformation to pointcloud using pcl
  
#### Development Nodes
  
 - `register_clouds` - test different registration algorithms including icp and Teaser
 (Summer 2023 - present)  
 - `get_cloud` - get pointcloud from aubo robot system
 - `get_target` - get pointcloud from aubo robot system and save as registration target
 - `get_source` - get pointcloud from aubo robot system and save as registration source
 - `register_target_source` - register target and source from aubo system (or file)

Note: This project has been running for years, and it has seen a lot of recent changes. Some of the older methods may not run currently, but effort is being made to bring everthing up to date and improve documentation. Contact thillRobot if you have questions or run into problems. 


#### Examples: 

These examples require a pre-recorded pointcloud from a 3D Lidar and/or CAD. There are example scans here.
This code is based on **PCL - Sample Consensus and RANSAC (SEGMENTATION)**
Note: **.pcd** files are currently in the **.gitignore** so you have to generate them locally which is explained below.

<!-- ##### try this one pcd_to_pointcloud - THIS WORKS (02/04/2020)-->
<!--`roslaunch seam_detection segment_plane_line.launch in_file:="test_cloud11.pcd" thresh:=0.01`-->
#### Random Sample Consensus - RANSAC
##### Use RANSAC to fit a models to a pointcloud.
The library supports planes, cylinders, spheres and more.
```
roslaunch seam_detection ransac_plane.launch in_file:="lidar_cad_scenes/test_cloud8.pcd" thresh:=0.01
```

##### Use RANSAC to segment, or separate, pointclouds.
```
roslaunch seam_detection segment_plane.launch in_file:="lidar_cad_scenes/test_cloud8.pcd" thresh:=0.01
```

##### Use RANSAC for weld seam detection. Each plane is found with RANSAC, then the origin is calculated as the intersection of the three planes.
```
roslaunch seam_detection seam_detection_RANSAC.launch in_file:="lidar_cad_scenes/lidar_scene1.pcd" out_file:="scene1.txt" thresh1:=0.01 thresh2:=0.001
```

#### Iterative Closest Point - ICP
These demos require two points clouds to be saved as `.pcd` files. Use the default images or make you own with the following procedure.
##### Import a cloud from a CAD model: Solidworks(.stl)-->meshlab(.ply)-->cad2cloud(.pcd)-->ROS(pointcloud)

###### Step 1)
Make a part in your CAD program of choice. This part will become the 'reference cloud'. Save the file as a '.stl' file. If your CAD program can create a '.ply' or '.pcd' file you can skip steps 2 or 3 respectively. Use units of meters for the part and stl export.

Note: In Solidworks: Save As -> select .ply then click Option and set the output units to meters.   

###### Step 2)
Convert the '.stl' to a '.ply' file using meshlab. Open the '.stl' using 'import mesh' and save it as a '.ply' file. Step2 can be done in solidworks. This could combine Step 1 and Step 2.

###### Step 3 - Option 1)
Convert a single '.ply' file to a '.pcd' file using `cad_cloud.cpp`. This is a I wrote based on sample code from PCL. The input arguments determine the resolution of the resulting files, and you must pass in the input and output file names. The parameters `-n_samples` and `-leaf_size` determine the resolution of the conversion.

```
cd seam_detection

rosrun seam_detection cad_cloud -n_samples 100000 -leaf_size 0.00025 -write_normals 1 ply_images/input.ply pcd_images/output.pcd

pcl_viewer -multiview 1 output.pcd
```
###### Step 3 - Option 2)
Alternatively, you can use `cad_cloud_bulk.cpp` to convert an entire directory of **.ply** files to **.pcd** files. This is based on the same sample code from PCL, but it iterates through the input directory with `boost`. The conversion parameter arguments are the same, but you must include the input and output directories instead of the file names.

```
rosrun seam_detection cad_cloud_bulk -n_samples 100000 -leaf_size .00025 -write_normals 1 -input_dir "ply_images/" -output_dir "pcd_images/"
```

Sometimes the extensions get changed from .ply to .PLY and I do not know why. This causes errors and confusion because there can be duplicate files in the same location. To fix this issue delete any duplicates before running the following command.

```
find . -name '*.*' -exec sh -c '
  a=$(echo "$0" | sed -r "s/([^.]*)\$/\L\1/");
  [ "$a" != "$0" ] && mv "$0" "$a" ' {} \;
```

This comes from [stackoverflow](https://stackoverflow.com/questions/11818408/convert-all-file-extensions-to-lower-case/11824856)


###### Step 4)
Use ICP to compare the CAD/reference image to the LIDAR/source image.
The LIDAR '.pcd' file must also be in the image directory. There are four numbered scenes choose from. There appears to be one directory level hidden in the launch file. I am not sure if this is a good idea or not. 


```
roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_cad_scenes/plate_cylinder.pcd" cad_file:="lidar_cad_scenes/cylinder.pcd"  thresh:=0.0001

```

Note: The code for seam_detection_icp.cpp was removed for clarity. Use the updated seam_detection.cpp node instead. 

#### RANSAC + ICP SEAM DETECTION 
##### Use RANSAC + ICP for weld seam detection. First segmen t with RANSAC(or other) then use ICP to locate the origin of the part.

These examples have the `round_tube` or a `square_tube` and the `plate`. There can be variations in part1 one but you must choose `round_tube` or a `square_tube`for the segmentation to work properly. These work well, but there is some discrepancy along the length of the cylinder. All other dimensions match very well. This seems to be related to the amount of data that is avaialable about this dimension.


Now define all the file names and other parameters in a <config>.yaml file. The .yaml files are saved in `config/`
**Note:** the way config files are handled is being improved to streamline the process, see `filter_cloud.yaml` and `register_clouds.yaml`
the launch arg has been renamed from `scene` to `config`, see `filter_cloud.launch` etc. The configs and docs are slowly being updated with the changes.

```
---
scene_name: "table_8in10in_tee_longclamps_x4y24_45"
lidar_src_file: "pcd_images/table_tee/table_8in10in_tee_longclamps_x4y24_45.pcd"
part1_name: "8in10in_tee_01"
cad_ref_file: "pcd_images/table_tee/8in10in_tee_01_blndrB.pcd"
part1_type: "square_tube"
part2_name: "none"
part2_type: "none"

filter_box: [-0.05, 0.30, -1.0, -0.4, -0.02, 0.3] # bounding box limits
voxel_leaf_size: 0.0005                           # voxel leaf size

ransac_norm_dist_wt: 0.1 # RANSAC Normal Distance Weight
ransac_max_iter: 100     # RANSAC Maximum Iterations
ransac_dist_thrsh: 0.03  # RANSAC Distance Threshold
ransac_k_srch: 50        # RANSAC KD Tree Parmeter(?)
ransac_init_norm: [-1.0, 1.0, 0.0] # RANSAC init perpendiculr vector

icp_max_corr_dist: 0.5            # ICP Maximum Correspondence Distance
icp_max_iter: 1000000000          # ICP Maximum Iterations
icp_trns_epsl: 0.000000001        # ICP Transformation Epsilon
icp_ecld_fitn_epsl: 0.000000001   # ICP Euclidean Distance Fitness Epsilon          

expected_results: [0.1016, -0.6096, 0.0254,0.0,0.0,0.7854] # [4.0in, -24.0in, 1.0in]*(25.4e-3) #[0.0,0.0,45.0]*(pi/180)
calibration_offset: [-0.00893203,-0.000860624,0.00537355,-0.00493333,-0.000708936,0.019938]

seam1_length: 1.0                      # weld seam length
seam1_points_x: [10, 11, 12, 13]       # weld seam control points
seam1_points_y: [20, 55, 22, 23]
seam1_points_z: [30, 31, 54, 12]

```

Pass the name of config when using seam_detection as shown below. This is much more convenient and allows for all nodes in the system access to parameters. Many of the old scene configs can be found in `config/scenes`


##### Simulated Application - source and target clouds from CAD (simulated LiDAR)

**plate_rect_block**

```
roslaunch seam_detection seam_detection.launch config:="scenes/plate_rect_block_02"
```
```
roslaunch seam_detection seam_detection.launch config:="scenes/plate_rect_block_02_blndr"
```
```
roslaunch seam_detection seam_detection.launch config:="scenes/plate_rect_block_02_blndr"
```
```
roslaunch seam_detection seam_detection.launch config:="scenes/plate_rect_block_02_dx100_rz30_blndr"
```

**table_tee**

add these here, this scene was used in the 2022 pub
(table_tee simulated application is missing or was never completed, look into this)

##### Experimental Application A - LiDAR scans from RPLiDAR A2 on Aubo i5

**table_8in10in_tee**

```
roslaunch seam_detection seam_detection.launch config:="scenes/table_8in10in_tee_x4y24_45"
```

```
roslaunch seam_detection seam_detection.launch config:="scenes/table_8in10in_tee_x0y24"
```


**table_8in10in_tee_longclamps**


```
roslaunch seam_detection seam_detection.launch config:="scenes/table_8in10in_tee_longclamps_x4y24_45"
```

##### Experimental Application B - LiDAR scans from RPLiDAR A2 on Aubo i10 - recorded 06/21/2023

add this 




##### archived examples 
These examples will not run because the config files need to be updated to the new format, do this soon

**plate_round_tube_01** (not working)
```
roslaunch seam_detection seam_detection.launch scene:="plate_round_tube_01"
```

**plate_square_tube_01** (not working)
```
roslaunch seam_detection seam_detection.launch scene:="plate_square_tube_01"
```

**table_plate_tee_clamps** (not working)

```
roslaunch seam_detection seam_detection.launch scene:="table_plate_tee_clamps_c1_blndr"
```

###### simualated test scenes

   - [x] fillet weld: lidarfile= `plate_round_tube_01.ply(.pcd)`, cadfile=`round_tube_01.ply(.pcd)` tested and working
   - [x] fillet weld: lidarfile= `plate_round_tube_02.ply(.pcd)`, cadfile=`round_tube_02.ply(.pcd)` tested and working

   - [x] fillet weld: lidarfile= `plate_square_tube_01.ply(.pcd)`, cadfile=`square_tube_01.ply(.pcd)` tested and working
   - [x] fillet weld: lidarfile= `plate_square_tube_02.ply(.pcd)`, cadfile=`square_tube_02.ply(.pcd)` tested and working
   - [x] fillet weld: lidarfile= `plate_square_tube_03.ply(.pcd)`, cadfile=`square_tube_03.ply(.pcd)` tested and working

   - [ ] fillet weld: `table_plate_cylinder`- does not work - RANSAC segmentation fails

   - [ ] fillet weld: `square tube to plate` - designed by RS - initial tests now
   - [ ] fillet weld: `round tube to plate` - designed by RS - initial tests





##### Testing TEASER

##### Testing Model Recognition from PCL

```
rosrun seam_detection correspondence_grouping pcd_images/plate_rect_block/rect_block_02.pcd pcd_images/plate_rect_block/rect_block_02.pcd -c -k

```

the correspondence grouping sample code compile and runs, but it only rfinds a model instance if I give it the same clouds...

```
$ rosrun seam_detection correspondence_grouping pcd_images/table_tee/offset_tee_01.pcd pcd_images/table_tee/offset_tee_c1.pcd -c -k
Failed to find match for field 'rgba'.
Failed to find match for field 'rgba'.
Model total points: 98060; Selected Keypoints: 1205
Scene total points: 98121; Selected Keypoints: 72
Correspondences found: 60
Model instances found: 0

$ rosrun seam_detection correspondence_grouping pcd_images/table_tee/offset_tee_c1.pcd pcd_images/table_tee/offset_tee_c1.pcd -c -k
Failed to find match for field 'rgba'.
Failed to find match for field 'rgba'.
Model total points: 98121; Selected Keypoints: 1217
Scene total points: 98121; Selected Keypoints: 72
Correspondences found: 70
Model instances found: 1

    Instance 1:
        Correspondences belonging to this instance: 56

            |  1.000 -0.000 -0.000 | 
        R = |  0.000  1.000  0.000 | 
            |  0.000  0.000  1.000 | 

        t = < 0.000, -0.000, -0.000 >


```

```
BEGINNING RANSAC SEGMENTATION
Plane coefficients: header:
seq: 0 stamp: 0 frame_id:
values[]
  values[0]:   9.70263e-06
  values[1]:   9.7027e-06
  values[2]:   1g
  values[3]:   -0.0250056

PointCloud representing the planar component: 2993 data points.
[pcl::SampleConsensusModel::getSamples] Can not select 0 unique points out of 0!
[pcl::RandomSampleConsensus::computeModel] No samples could be selected!
[pcl::SACSegmentationFromNormals::segment] Error segmenting the model! No solution found.
```


###### new test scenes - generated 06/21/2023 - rplidar a2 on aubo i10

two new steel objects: `shape1` and `shape2` (best names ever)

testing with `register_clouds.cpp` to compare ICP vs TEASER vs TEASER_FPFH (fast point feature histogram)

test mode 1: CAD model based target cloud (fixed) and a LiDAR based source cloud (transformed) - (what we previously have done)        

example: 

```
roslaunch seam_detection register_clouds.launch config:="scenes/shape2_60deg"
```

NEW! test mode 2: LiDAR based taget cloud and different LiDAR scan based source cloud - as suggested by SC 

example:

```
roslaunch seam_detection registration_examples.launch config:="scenes/shape2_45deg_60deg"
```



#### Notes about new tests

- Being too 'far away' can cause ICP to fail. This may seem obvious, but I have not thought about the scale of initial translation until now. This is at least one thing I have  learned from the new shape1_shape2 dataset.

- 8 new scans have been recorded and saved with the project. These have all been tested with ICP registration and TEASER_FPFH registration. 

- registration using a CAD model based target cloud (fixed) and a LiDAR based source cloud (transformed) not always successful. Shape 1 is not successful in this mode, but shape 2 is successful with ICP in several examples.  Some examples have minor alignment error. TEASER and TEASER_FPFH are not successful in this mode.

- shape 1 is not successful in the examples in which alignment requires more than 180 of z rotation. This may be because the alignment must pass through a local minimum where both planes are parallel but offset which occurs at a 180 offset because the object is rectangular. This is somewhat confirmed by the fact that registration is successful if the source orientation is within X degs of target - X needs to be determined 

- Dr. Canfield suggested comparing LiDAR scan clouds to different LiDAR scan clouds. I do not know why we have not tried this yet. This mode seems to be more successful. Registration is successful using ICP and TEASER_FPFH (needs more testing) in several shape1 and shape2 examples. 

- re-registration with ICP does not produce improved results - this is not suprising because iteration is built into the routine, max iterations is a parameter



#### TEASER Notes

Teaser is running on the the current data sets, however the results are not correct or usable. 

The translation component of the solution is mostly correct. It appears that the algorithm is successfully locating the centroid of the workpiece based on the ROS  visualization.

The rotation portion of the solution is not correct. It is off by 30+ deg. 

##### BIG IDEA
It appears that registration requires correspondence and/or overlapping point clouds. However, most sample code and algorithm testing is done on standard data sets, and the clouds in these sets have correspondance by nature of the test. For example, it is common to test registration on a cloud and a modified version of the same cloud. This test represents an ideal situation and best case inputs to the registration problem in which correspondence within a tolerance is expected. 

The applied registration problem for workpiece localization provides no guarantee that of correpondence between cloud points can be found. Check for this idea in the literature.


#### More TEASER Notes

If the number of source points (Ns) is less that the number of target points (Nt), seam_detection_teaser.cpp crashes during TEASER registration. This has been double checked, Ns > Nt or TEASER will crash.

If the number of source points is close to the number of target points, the algorithm converges quickly. Currently testing will N = ~1000

Some of the data sets we are testing are not good because they have low overlap. This is partially due to the segmentation process and partly due to the lidar shadow concept. Next, setup/find a ideal case for algorithm testing. I have a hunch that TEASER is working, but the data we are giving it is not great. CHECKTHISSOON!

It seems that it is all about the input data, duh!

it appears the python example may just be a wrapper on what we already have access to

on the other hand it seems like the python_3dsmooth.py example is different, uses KNN 3DSmoothNet 

3DsmoothNet seemed interesting and the demo results are compeling, the downside is that the example is outdated in python 3.5,3.6, this is not what we want but we could use docker to make an old environment just for the sake of testing, it would be worth it I think

New Stuff! - While invesigating the 3DsmoothNet author Zan Gojcic(zgojcic@github) I found something very interesting! Guess what it also has a catchy name: OverlapPredator [prs-eth/OverlapPredator](https://github.com/prs-eth/OverlapPredator). It seems like the main idea is "Registration of 3D Point Clouds with Low Overlap". This may be what we need. Also, the tested example is for Python 3.8.5 which is much more acceptable than 3.5/3.6 as in previous example.

Predator comes from the Photogrammetry and Remote Sensing Lab: https://github.com/prs-eth


### running in Docker

It is possible to stand up the entire application in a single line using docker and docker compose. This is not required. 

Create a source directory and set the environment variable $CATKIN_WS_PATH 

```
mkdir -p ~/catkin_ws/src
export CATKIN_WS_PATH=~/catkin_ws
```

Clone this repository into $CATKIN_WS_PATH/src

```
cd $CATKIN_WS_PATH/src
git clone git@github.com:thillrobot/seam_detection
```


Modify xauth access control 
```
xhost local:root
```

Build the container and start the application

```
cd seam_detection/docker
docker compose up --build 
```

Run a simple test, this calls one of the launch files from above.

```
cd seam_detection/docker
docker compose run seam_detection
```

Note: if you want to generate files in the container, for example when using `cad_cloud` to make PCDs, you must set the permissions of the workspace directory or sub directory on the host to give the docker container write access which would be an 'other' in this case.

```
chmod o+w seam_detection/<SUBDIR>
````

### troubleshoot with auborobot

i see the following error when i try to publish to andriod_gui/gcode_cmd
```
[ERROR] [1692989638.343471973]: Client [/rostopic_17739_1692988617268] wants topic /android_gui/gcode_cmd to have datatype/md5sum [aubo_control/gcodeAction/8261e41e53803494ec669905817b139c], but our version has [aubo_control/gcodeAction/a83a0e1a726f23e73947f0f4e478e627]. Dropping connection.
```

i think this is a noetic-kinetic version mismatch but I am not sure. The custom message compiles and published on the local machine fine, neither remote machine can see the msg from the other computer

the published message (noetic side) looks just fine

```
---
header: 
  seq: 304
  stamp: 
    secs: 0
    nsecs:         0
  frame_id: ''
file_name: "scan_target"
job_cycles: 0
start_job: True
loop: False
start_atline: 0
---
```

we might have to make a patch on the kinetic side to fix this.

### new scenes and launch files from Aubo i10, Summer 2023 at RTT

launch files: 
    - `aubo_i10_lidar.launch`             (used for collecting lidar scans from aubo, spring 2023)
    - `aubo_i10_target.launch`            (used for collecting target scans from aubo lidar for registration)
    - `aubo_i10_source.launch`            (used for collecting source scans from aubo lidar for registration)
    - `aubo_i10_target_source.launch`     (mostly not used, can be removed?)


The process from the summer 2023 season needs documentation!


### filter_cloud 

This routine is designed to automate the selection/identification of the source cloud to be used in registration 












### new test scenes from ds435i depth camera
pcd files in `pcd_images/ds435i_table_parts/`

```
roslaunch seam_detection filter_cloud.launch config:="filter_cloud_ds435i"
```





### Changelog
#### Tagged Versions
- v1.0 (stable - tagged 12/07/2020)
- v1.1 (stable - tagged 12/26/2020)
  - added `round_tube` or `square_tube` segmentation option for part1
  - added `part1_type` to `seam_detection.launch` args
  - removed `thresh` from `seam_detection.launch` args
- v1.2 (stable - tagged 01/15/2021)
  - added yaml files as config files for parameters, see `seam_detection/config/`
  - added `cad_cloud_bulk.cpp`  to process multiple .ply files at once
- v1.3 (stable - tagged 02/05/2021)
  - added part1 types `rect_block` and `round_tube`
  - added calibration sets c0-c8 for `round_tube`, `square_tube`, and 'rect_block' (partial)
  - added `icp_params` to config files to adjust search without re-compile
  - added `src/archive/` for old source code
- v1.4 (stable - tagged ~03/05/2021)
  - changed: reduce to finding a single part, `part1` 
  - exposed `part2` (table) cloud and `filtered` outliers to rviz 
  - added scenes `table_tee` and `table_offset_tee` and `table_offset_tee_clamps`
  - reorganized main node `seam_detection` and source code directory
  - successfully tested part localization approach with artificial data with clamps
    (target,source) -> [Bounding Box -> Voxel -> RANSAC -> ICP] -> transformation 
- v1.5 (development - master/devel)
  - exposed `part2` (table) cloud and `filtered` outliers to rviz 
  - added `ransac_params`  and `filter_params` to config files to adjust search without re-compile
  - added scenes `8in10in_tee` and `8in10in_tee_longclamps` 
- v1.6 (stable - main/devel - tagged 12/14/2022)
  - changed branch name from `master` to `main`
  - added files to run in docker container  
  - successfully tested in `ros:noetic-robot-focal` container, first time successfully testing in 20.04
  - updated example launch commands in this README
  - testing TEASER++ registration, very sensitive to size of input data 
- v1.7 (development - main/devel - tagged 06/22/2023)
  - added new experimental test scans from RPLiDAR A2 + Aubo i10 - `shape1_shape2`
  - added separate source code for `filter_cloud()` and `registration_examples()` to simpify testing 
  - added functions: `register_cloud_icp()`, `register_cloud_teaser()`, `register_cloud_teaser_fpfh()` to simplify testing
  - successfully tested in `registration_examples.cpp` with ideal data, results look good
  - added configuration files for filtering - all configs need documentation
  - testing registration of clouds without pre-segmentation  
  - testing new registration mode: LiDAR scan to LiDAR scan - see `shape1_shape2` examples
  - added bounding box to config files to allow for hand cropping input data
  - create ideal data sets `rect_block_02_blndr` and `rect_block_02`  for testing different registration algorithms- needs better docs
  

#### Things To Do (priority top to bottom):

- [x] design and test process with multi computer cross version setup on robot with LiDAR and RGBD camera
    -> pointcloud and tf published by arm computer, vision computer subscribes
    -> registration result tf published by vision computer, arm computer subscribes   
    [ ] document the process of interfacing with Aubo/RTT robot

- [ ] stream line filtering->clustering->registration for testing alongsisde robot

- [x] use PCL Euclidean cluster extraction to replace segmentation or manual bounding box preparation of lidar scans
    -> proof of concept shown in `filter_cloud.cpp`, robot body is successfully separated from workpeice, clusters shown in rviz
    -> this needs testing for robustness and documentation, 
    -> volume + aspect ratio objective function used, improve this with weighted factors instead of summative

- [x] test clustering with workpeice plus separated objects on table and consider a selection algorithm to determine which cluster is the correct workpeice, cluster size is not a robust metric for selection if the workpeice size varies. The workpeice size will vary.

- [ ] test clustering with workpeice and clamps
    -> older scans 
    -> synthetic data with clamps 
    -> to be collected new scans

- [ ] re-test registration with workpeice compared to workpeice with clamps
    -> older scans 
    -> synthetic data with clamps 
    -> to be collected new scans

- [x] collect new scan data from auboi10 and rplidar and/or lightware 
    -> with extra objects and without
    -> [ ] with clamps and without
    -> consider collection 3D camera data general purpose research
    -> carefully consider design of experiments  

- [x] test icp registration from several tracked starting locations to avoid getting stuck in local minimum, compare scores of each result and lowest should be correct
    -> first pass at this seems to work, four positions were tested and the correct position can be identified with the fitness score 
    -> code only works for ICP, not TEASER or TEASER_FPFH yet

- [ ] test working data sets from recent scans with Overlap Predator, this is being tested in a different repo

    -> Overlap Predator demo was working fine at some point June 29, then the `sh scripts/download_data_weight.sh ` started failing to connect. I do not think this was on my end, but I an not sure. I can test that from the office if it still will not connect tomorrow. Hopefully, they did not kick me out. WOW! Just as I wrote this it finally started working so I guess it was magic or they are spying on me.

    -> MinkowskiEngine Demos seem to be working with visualizations, run `cd MinkowskiEngine` and `python -m examples.indoor` in the docker container to see the segmentation demo

    -> FCGF demo seems to be working, but the visualizations are not. I have tried to use the same environment that is successfully working with MinkowskiEngine, but that has not solved the visualization code yet. I was debugging this with print statements. Wow, writing notes must really be magic. As I was about to walk away, the example finally worked in docker with the visualizations. Good, if nothing else you can run other peoples examples. Keep up the amazing work lol.

    -> these tests are not documented well, and dockerfiles are not in repos so they will be hard to transfer across machines and there is no cloud backup. I would just put them in the forked repos, but .... wait why dont we do that??? why are they in the parent directory anyway? hmmm....


- [x] test new LiDAR sensor from lightware, capture new 3D scans for comparison
    -> test in progress

- [x] update and document config file system to allow for iterative (cascaded) registration, this should be doable without major modifications to the source code
    -> iteratiion is typically built into registration methods, also it does not seem to produce improved results when interation is performed manually

- [x] test iterative registration on current and previous experimental data sets
    -> no significant results shown, you have learned this once again so maybe this time you will remember it

- [ ] clean up README and test archived and older examples  - update old config files with new parameters lists, maybe we should wait until we finish changing lol

- [ ] implement `overlap_predator` registration on experimental data for performance comparision, this might solve orientation issues

- [ ] add separate code for cloud_segmentation to complete separation of steps 

- [ ] use a .cpp class to improve the implementation of seam_detection.cpp, clean up the code in general, it is overbloated!

- [ ] re-visit data preparation with correspodence in mind, aim for equally dense target and source clouds  

- [ ] continue investigating the affects of cloud density on the performance of ICP. It is apparant that this effects the proper convergence of ICP. 

- [ ] investigate and demonstrate the affect of the voxel filter

- [ ] investigate different segmentation models - progress made with multiple planes and `SAC_PERPENDICULAR_PLANE` 

- [ ] develope processing multiple parts, two parts only for now

- [ ] add description of the seam to the model - i have begun by creating seam *.pcd* files - lists for seam points are setup in the config file

- [ ] calculate a *measure of accuracy* - i started this in `analyse_results` then moved this to `register_cloud_icp` in a hurry, needs to go back now

- [ ] The tf migration is incomplete. Parts of both libraries are currently used. For example `tf::transform` is used for  `pcl_ros::transformPointCloud`. There is probably another way, but I have not figured it out yet.
- [ ] Improve conversion from `ICP::` to `TF::` in REGISTER_CLOUD_ICP function in `seam_detection.cpp`. Currently it is clunky and overbloated, but it works.

- [ ] improve efficiecy of `seam_detection.cpp` by redcucing the number of extra copies of cloud objects used in the main workflow. Many of these were only used for debugging purposes. 

- [ ] document data collection and calibration process using 3D LiDAR system - update `scan2cloud` package and docs - TAKE NOTE NEXT TIME !

- [?] prepare a manuscript for ASME IDETC2024 or alternate venue

- [ ] prune this list
