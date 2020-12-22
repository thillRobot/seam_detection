# seam_detection
## This is a ROS package for weld seam detection using pointcloud data.

### Installation Instructions for seam_detection 

#### Requirements:
##### Operating System: 
- Ubuntu 18.04 - tested and working best
- Ubuntu 20.04 - testing now, working but not converging
##### ROS Version (linked to OS):
- ROS Melodic
- ROS Noetic
##### Hardware 
- not identified

#### Step 1 - Setup ROS workspace
If you want to use a catkin workspace that is already setup, then you can skip **Step 1** (you workspace must compile). If you do not, then create and build a catkin workspace before proceeding. Choose a location and insert a name for the workpace. Typically this is somewhere in `~/`.

```
mkdir -p ~/<workspace-name>/src
cd ~/<workspace-name>
catkin_make
```

This will build your workspace for the first time. There should now be a `CMakeLists.txt` file and a few new directories in the workspace.

#### Step 2 - Download SEAM_DETECTION Package
Change to the source directory of the workapce and pull the package using git.

```
cd ~/<workspace-name>/src
git clone https://github.com/thillRobot/seam_detection.git
```


### Using SEAM_DETECTION v1.0


These examples require a pre-recorded pointcloud from a 3D Lidar and/or CAD. There are example scans here.
This code is based on **PCL - RANSAC and SEGMENTATION***


<!-- ##### try this one pcd_to_pointcloud - THIS WORKS (02/04/2020)-->
<!--`roslaunch seam_detection segment_plane_line.launch in_file:="test_cloud11.pcd" thresh:=0.01`-->
#### Random Sample Consensus - RANSAC
##### Use RANSAC to fit a models to a pointcloud. 
The library supports planes, cylinders, spheres and more.
```
roslaunch seam_detection ransac_plane.launch in_file:="test_cloud8.pcd" thresh:=0.01
```

##### Use RANSAC to segment, or separate, pointclouds.
```
roslaunch seam_detection segment_plane.launch in_file:="test_cloud8.pcd" thresh:=0.01
```

##### Use RANSAC for weld seam detection. For now it is just locating the origin of the intersection of three planes.
```
roslaunch seam_detection seam_detection_RANSAC.launch in_file:="lidar_scene1.pcd" out_file:="scene1.txt" thresh1:=0.01 thresh2:=0.001
```

#### Iterative Closest Point - ICP
These demos require two points clouds to be saved as `.pcd` files. Use the default images or make you own with the following procedure.
##### Import a cloud from a CAD model: Solidworks(.stl)-->meshlab(.ply)-->cad2cloud(.pcd)-->ROS(pointcloud)

###### Step 1) Make a part in your CAD program of choice. This part will become the 'reference cloud'. Save the file as a '.stl' file. If your CAD program can create a '.ply' or '.pcd' file you can skip steps 2 or 3 respectively.

###### Step 2) Convert the '.stl' to a '.ply' file using meshlab. Open the '.stl' using 'import mesh' and save it as a '.ply' file. Step2 can be done in solidworks. This combines step and step 2. Nice

###### Step 3) Convert the '.ply' to a '.pcd' file using cad_cloud (this is a package I wrote that uses PCL). run the following commands.

```
cd seam_detection

rosrun seam_detection cad_cloud ply_images/input.ply pcd_images/output.pcd -n_samples 100000 -leaf_size 0.001 -write_normals 1 -no_vis_results 0

pcl_viewer -multiview 1 output.pcd
```

###### Step 4) Use ICP to compare the CAD/reference image to the LIDAR/source image. 
The LIDAR '.pcd' file must also be in the image directory. There are four numbered scenes choose from.

```
roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene1.pcd" cad_file:="cad_scene1.pcd"  thresh:=0.003
```


##### use ICP for weld seam detection. For now it is just locating the origin of the part.

```
roslaunch seam_detection seam_detection_ICP.launch lidar_file:="plate_cylinder.pcd" cad_file:="cylinder.pcd"  thresh:=0.0001
```

#### RANSAC + ICP SEAM DETECTION - In Development
##### use RANSAC + ICP for weld seam detection. For now it is just locating the origin of the part.

These two examples have the cylinder and the plate only. These work well, but there is a discrepancy along the length of the cylinder. All other dimensions match very well. 
```
roslaunch seam_detection seam_detection.launch lidar_file:="plate_cylinder.pcd" cad_file:="cylinder.pcd" thresh:=0.0001
```
```
roslaunch seam_detection seam_detection.launch lidar_file:="plate_cylinder_rotated.pcd" cad_file:="cylinder.pcd" thresh:=0.0001
```

This example has a second plane that represents the table that the parts are sitting on. This is not working. RANSAC fails.
```
roslaunch seam_detection seam_detection.launch lidar_file:="table_plate_cylinder.pcd" cad_file:="cylinder.pcd" thresh:=0.0001
```

```
BEGINNING RANSAC SEGMENTATION
Plane coefficients: header: 
seq: 0 stamp: 0 frame_id: 
values[]
  values[0]:   9.70263e-06
  values[1]:   9.7027e-06
  values[2]:   1
  values[3]:   -0.0250056

PointCloud representing the planar component: 2993 data points.
[pcl::SampleConsensusModel::getSamples] Can not select 0 unique points out of 0!
[pcl::RandomSampleConsensus::computeModel] No samples could be selected!
[pcl::SACSegmentationFromNormals::segment] Error segmenting the model! No solution found.
```

### THINGS TO DO

#### Prepare for IDETC2021
- [x] create branch called 'stable' to store the working code 
- [ ] create tag called v1.0 and document how to use and pull with tag - makes a snapshot of code
- [ ] design and test new scenes with v1.0
- [ ] fix RANSAC segmentation of the table and plate
- [ ] decide to include table or not inlcude table. It will be in the scan so I think the code needs to be able to handle table
- [ ] dust off and test workflow for designing new scene and converting to the proper filetypes, this has not been tested recently 

   ##### current test scenes
   - [x] fillet weld: cylinder to plate -  (cylinder has angled top feature) - tested and works 
   - [ ] fillet weld: square tube to plate - RS is designing - test soon
   - [ ] fillet weld: cylinder to plate sitting on table - does not work - RANSAC segmentation fails

- [ ] calculate a *measure of accuracy* 
- [ ] determine or register key points on key parts (?)

#### Continue Development of `seam detection.cpp` which is implemenation of RANSAC + ICP SEAM DETECTION mentioned above.
   - [x] migration from 'TF'  to 'TF2'. It has already fixed the 'static publisher issue'
   - [ ] The migration is incomplete. Parts of both libraries are currently used. For example `tf::transform` is used for  `pcl_ros::transformPointCloud`. There is probably another way, but I have not figured it out yet.
   - [ ] In REGISTER_CLOUD function in `seam_detection.cpp` I think I could go straight from `ICP` to `TF` and not pull out the values like it is done currently, but it works like that for now.
   - [ ] Multiple parts needs to be developed , cylinder and plate seem to work now
   - [ ] add 'markers' to show the results from RANSAC, the markers are there, but they need the pose info now. (03/06/2020)
   - [ ] begin migration to Fossa/Noetic - everything compiles  and returns
   - [ ] there was a driver issue running the T1600 in Ubuntu 20.04 and Noetic, the driver for the old video card called 'nvidia-340' was not supported for the graphics libraries so I switched to the 'Nouveau' driver and this fixed the 'libgl error no matching fbconfigs or visuals found' issueafter that everything runs.
   - [ ] we are running the modern drivers (nvidia455) on the  T1600+GTX1650 GPU, what are we running now on the NUC+ Kaby Lake embedded graphics
   - [ ] the ICP stage does not work. It does converges but the score is too high, and the results do not make any sense. It works in Melodic but not in Noetic. **This is a issue**.



