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
- Ubuntu 18.04 - tested and working best
- Ubuntu 20.04 - testing now, working but not converging
##### ROS Version (linked to OS):
- ROS Melodic
- ROS Noetic
##### Hardware
- no hardware requirements identified currently
##### Graphics
- `nvidia-460`
- `nvidia-455` working currently
- `nvidia-340` was not supported but older card worked with `Nouveau`
- intel embededded graphics `Kaby Lake` and others


#### Step 1 - Setup ROS workspace
If you want to use a catkin workspace that is already setup, then you can skip **Step 1** (you workspace must compile). If you do not, then create and build a catkin workspace before proceeding. Choose a location and insert a name for the workpace. Typically this is somewhere in `~/`.

```
mkdir -p ~/<workspace-name>/src
cd ~/<workspace-name>
catkin_make
```

This will build your workspace for the first time. There should now be a `CMakeLists.txt` file and a few new directories in the workspace.

Also, you need to add the workspace path to `/.bashrc` so that ros can find your packages

#### Step 2 - Download seam_detection Package
Change to the source directory of the workapce and pull the package using git.

```
cd ~/<workspace-name>/src
git clone https://github.com/thillRobot/seam_detection.git
```
#### Step 3 - Compile seam_detection in catkin workspace
Change to top of workspace and compile.

```
cd ..
catkin_make
```

The workspace and package should compile without errors.

### Using seam_detection

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


#### RANSAC + ICP SEAM DETECTION - In Development
##### Use RANSAC + ICP for weld seam detection. First segmenmmt with RANSAC(or other) then use ICP to locate the origin of the part.

These examples have the `round_tube` or a `square_tube` and the `plate`. There can be variations in part1 one but you must choose `round_tube` or a `square_tube`for the segmentation to work properly. These work well, but there is some discrepancy along the length of the cylinder. All other dimensions match very well. This seems to be related to the amount of data that is avaialable about this dimension.


Now you define all the file names and other parameters in a <scene>.yaml file. The .yaml files are saved in `config/`

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

Pass the name of scene when using seam_detection as shown below. This is much more convenient and allows for all nodes in the system access to parameters.  


##### Simulated Application - source and target clouds from CAD (simulated LiDAR)

**plate_rect_block**
```
roslaunch seam_detection seam_detection.launch scene:="plate_rect_block_c0_blndr"
```
```
roslaunch seam_detection seam_detection.launch scene:="plate_rect_block_c1_30_blndr"
```
```
roslaunch seam_detection seam_detection.launch scene:="plate_rect_block_c1_blndr"
```
```
roslaunch seam_detection seam_detection.launch scene:="plate_rect_block_c2_blndr"
```

**table_tee**

add these here, this scene was used in the 2022 pub




##### Experimental Application A - LiDAR scans from RPLiDAR A2 on Aubo i5

**table_8in10in_tee**

```
roslaunch seam_detection seam_detection.launch scene:="table_8in10in_tee_x4y24_45"
```


```
roslaunch seam_detection seam_detection.launch scene:="table_8in10in_tee_x0y24"

```


**table_8in10in_tee_longclamps**


```
roslaunch seam_detection seam_detection.launch scene:="table_8in10in_tee_longclamps_x4y24_45"

```



##### archived examples 
These examples will not run because the config files need to be updated to the new format

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

###### current test scenes

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





### running in Docker

Use docker and docker compose to stand up the entire application in a single line. 


Create a source directory and set environment variable $CATKIN_WS_PATH 

```
mkdir -p ~/catkin_ws/src
export CATKIN_WS_PATH=~/catkin_ws
```

Clone this repository into $CATKIN_WS_PATH/src

```
cd $CATKIN_WS_PATH/src
git clone git@github.com:thillrobot/seam_detection
```


Disable xauth access control 
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

Note: if you want generate files in the container, for example when using `cad_cloud` to make PCDs, you must set the permissions of the workspace directory or sub directory on the host to give the docker container write access which would be an 'other' in this case.

```
chmod o+w seam_detection/<SUBDIR>
````




### Changelog
#### Versions
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
  - testing TEASER++ registration, rotation estimation not working yet 


#### Things To Do: 

- [ ] continue investigating the affects of cloud density on the performance of ICP. It is apparant that this effects the proper convergence of ICP. 

- [ ] consider improving the CAD->Cloud process to address the pixel density issue

- [ ] investigate and demonstrate the affect of the voxel filter

- [ ] investigate different segmentation models - progress made with multiple planes and `SAC_PERPENDICULAR_PLANE` 

- [ ] develope processing multiple parts, two parts work now

- [ ] continiue to develop description of the weld seam - lists for seam points are setup in the config file

- [ ] add description of the seam to the model - i have begun by creating seam *.pcd* files

- [ ] calculate a *measure of accuracy* - i started this in `analyse_results` then moved this to `register_cloud_icp` in a hurry

- [ ] update all old config files in archive with new parameters lists, maybe we should wait until we finish changing 

- [x] solve large file repo mess issue! -git-lfs? solution: dont commit large files, use .gitignore

- [ ] The tf migration is incomplete. Parts of both libraries are currently used. For example `tf::transform` is used for  `pcl_ros::transformPointCloud`. There is probably another way, but I have not figured it out yet.
- [ ] Improve conversion from `ICP::` to `TF::` in REGISTER_CLOUD_ICP function in `seam_detection.cpp`. Currently it is clunky and overbloated, but it works.

- [ ] use a .cpp class to improve the implementation of seam_detection.cpp - DO THIS!

- [ ] improve efficiecy of `seam_detection.cpp` by redcucing the number of extra copies of cloud objects used in the main workflow. Many of these were only used for debugging purposes. 

- [ ] re-do Experimental Application A and B with calibrated scans from Aubo i5

- [ ] document calibration process of 3D LiDAR system - update `scan2cloud` package

- [x] complete workpeice localization with TEASER as substitute for ICP and/or RANSAC - testing `seam_detection_teaser.cpp` currently

- [x] troubleshoot teaser rotation estimation, experiment with input cloud density and bounds 
  - Test TEASER with two point clouds from same source to verify idea about required correspondence   
  - Hand select/crop better realistic data
  - re-visit data preparation with correspodence in mind, aim for equally dense target and source clouds  
  - TEASER (no features) works with relatively dense _overlapping_ data. I think the later part is the ket.

- [ ] implement and test additional C++ example from TEASER: `teaser_cpp_fpfh.cpp`

- [ ] implement and test Python examples from TEASER: `teaser_python_fpfh_icp.py`, `teaser_python_3d_smooth.py`, `teaser_python_ply.py`

- [x] temporarily separate the segmentation and registration code to ease testing of new algorithms

- [x] create and save best case ideal data set for testing different registration algorithms

  -  scenes `rect_block_02_blndr` and `rect_block_02` should provide a good base line data set. They have been tested with ICP and TEASER in `registration_examples.cpp`

- [!] prepare a manuscript for ASME IDETC2023, submissions due: 2023-03-13