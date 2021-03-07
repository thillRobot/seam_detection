# seam_detection
This is a ROS package for weld seam detection using pointcloud data based on the point cloud library [PCL](https://pointclouds.org/).

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
<<<<<<< HEAD
=======

>>>>>>> master

Now you define all the file names and other parameters in a <scene>.yaml file. The .yaml files are saved in `config/`
```
---
scene_name: "plate_square_tube_c2"
scene_file: "pcd_images/plate_square_tube/plate_square_tube_c2.pcd"
part1_name: "square_tube_01"
part1_file: "pcd_images/plate_square_tube/square_tube_01.pcd"
part1_type: "square_tube"
part2_name: "plate"
part2_type: "plate with notch"
seam1_length: 1.0

seam1_xs: [10, 11, 12, 13]
seam1_ys: [20, 21, 22, 23]
seam1_zs: [30, 31, 32, 33]

icp_params: [1.0, 100000, 0.000001, 0.000001]

```

Pass the name of scene when using seam_detection as shown below. This is much more convenient and allows for all nodes in the system access to parameters.  

**plate_round_tube_01**
```
roslaunch seam_detection seam_detection.launch scene:="plate_round_tube_01"
```

**plate_square_tube_01**
```
roslaunch seam_detection seam_detection.launch scene:="plate_square_tube_01"
```

**plate_square_tube_02**
```
roslaunch seam_detection seam_detection.launch scene:="table_tee_c2_30_blndr"
```

```
roslaunch seam_detection seam_detection.launch scene:="table_offset_tee_clamps_c1_blndr"

```
```
roslaunch seam_detection seam_detection.launch scene:="table_offset_tee_clamps_c2_30_blndr"

```

##### add second plane that represents the table that the parts are sitting on.
This is not working. RANSAC fails.
```
roslaunch seam_detection seam_detection.launch lidar_file:="table_plate_cylinder.pcd" cad_file:="cylinder.pcd" thresh:=0.0001
```

<<<<<<< HEAD
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
- v1.4 (development - master/devel)
  - goal: reduce to finding a single part, `part1` - retain naming if possible


#### To Prepare for IDETC2021

- [ ] design and test new scenes with v1.2 - Choose scenes for paper - choose units

- [?] develop segmentation of the table and plate. Decide to include table or not include table. It will be in the scan so the code should be able to handle table

- [x] dust off and test workflow for designing new scene and converting to the proper filetypes, this has not been tested recently

- [?] document scene creation and conversion process - the steps are shown above

- [x] figure out square tube RANSAC - currently not used - is it needed?

- [ ] investigate segmentation models - can we set the width of the `SACMODEL_PLANE` ?

- [ ] develop description of the weld seam - draw weld seam in CAD and export as PCD file.

- [ ] add description of the seam to the model - i have begun by creating seam *.pcd* files

- [ ] determine or register control points on key parts from description of the seam

- [ ] calculate a *measure of accuracy*

- [ ] finish the manuscript !


   ##### current test scenes
   - [x] fillet weld: lidarfile= `plate_round_tube_01.ply(.pcd)`, cadfile=`round_tube_01.ply(.pcd)` tested and working
   - [x] fillet weld: lidarfile= `plate_round_tube_02.ply(.pcd)`, cadfile=`round_tube_02.ply(.pcd)` tested and working

   - [x] fillet weld: lidarfile= `plate_square_tube_01.ply(.pcd)`, cadfile=`square_tube_01.ply(.pcd)` tested and working
   - [x] fillet weld: lidarfile= `plate_square_tube_02.ply(.pcd)`, cadfile=`square_tube_02.ply(.pcd)` tested and working
   - [x] fillet weld: lidarfile= `plate_square_tube_03.ply(.pcd)`, cadfile=`square_tube_03.ply(.pcd)` tested and working

   - [ ] fillet weld: `table_plate_cylinder`- does not work - RANSAC segmentation fails

   - [ ] fillet weld: `square tube to plate` - designed by RS - initial tests now
   - [ ] fillet weld: `round tube to plate` - designed by RS - initial tests


- [ ] include **.pcd** and/or **.ply** in this **.gitignore** ?

***The plan is to wait until after IDETC 2021 submission to work on the developement issues below.***
#### Development of `seam detection.cpp` which is implemenation of RANSAC + ICP SEAM DETECTION
   - [x] migration from 'TF'  to 'TF2'. It has already fixed the 'static publisher issue'
   - [ ] The migration is incomplete. Parts of both libraries are currently used. For example `tf::transform` is used for  `pcl_ros::transformPointCloud`. There is probably another way, but I have not figured it out yet.
   - [ ] In REGISTER_CLOUD function in `seam_detection.cpp` I think I could go straight from `ICP` to `TF` and not pull out the values like it is done currently, but it works like that for now.
   - [ ] develope processing multiple parts, two parts work now
   - [ ] add 'markers' to show the results from RANSAC, the markers are there, but they need the pose info now. (03/06/2020)
   - [ ] migration to Fossa/Noetic - everything compiles  and returns
   - [ ] the ICP stage does not work. It does converges but the score is too high, and the results do not make any sense. It works in Melodic but not in Noetic. **This is an issue**.
