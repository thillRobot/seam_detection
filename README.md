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
#### PCL - RANSAC and SEGMENTATION


<!-- ##### try this one pcd_to_pointcloud - THIS WORKS (02/04/2020)-->
<!--`roslaunch seam_detection segment_plane_line.launch in_file:="test_cloud11.pcd" thresh:=0.01`-->

##### Use the RANSAC algorithm to fit a models to a pointcloud. The library supports planes, cylinders, spheres and more.
`roslaunch seam_detection ransac_plane.launch in_file:="test_cloud8.pcd" thresh:=0.01`

##### Use RANSAC models to segment, or separate, pointclouds.
`roslaunch seam_detection segment_plane.launch in_file:="test_cloud8.pcd" thresh:=0.01`

##### use RANSAC for weld seam detection. For now it is just locating the origin of the part.
`roslaunch seam_detection seam_detection_RANSAC.launch in_file:="lidar_scene1.pcd" out_file:="scene1.txt" thresh1:=0.01 thresh2:=0.001`


#### PCL - Iterative Closest Point (ICP)

##### Import a cloud from a CAD model. Currently it works like this - from Solidworks(.stl)-->meshlab(.ply)-->cad2cloud(.pcd)-->ROS(pointcloud!)

##### Step 1) Make a part in your CAD program of choice. This part will become the 'reference cloud'. Save the file as a '.stl' file. If your CAD program can create a '.ply' or '.pcd' file you can skip steps 2 or 3 respectively.

##### Step 2) Convert the '.stl' to a '.ply' file using meshlab. Open the '.stl' using 'import mesh' and save it as a '.ply' file. Step2 can be done in solidworks. This combines step and step 2. Nice

##### Step 3) Convert the '.ply' to a '.pcd' file using cad_cloud (this is a package I wrote that uses PCL). run the following commands.

`cd seam_detection`

`rosrun seam_detection cad_cloud ply_images/input.ply pcd_images/output.pcd -n_samples 100000 -leaf_size 0.001 -write_normals 1 -no_vis_results 0`

`pcl_viewer -multiview 1 output.pcd`

##### Step 4) Use ICP to compare the CAD/reference image to the LIDAR/source image. The LIDAR '.pcd' file must also be in the image directory.

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene1.pcd" cad_file:="cad_scene1.pcd"  thresh:=0.003`

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene2.pcd" cad_file:="cad_scene2.pcd"  thresh:=0.003`

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene3.pcd" cad_file:="cad_scene3.pcd"  thresh:=0.003`

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene3.pcd" cad_file:="cad_scene4.pcd"  thresh:=0.003`


##### use ICP for weld seam detection. For now it is just locating the origin of the part.
`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="plate_cylinder.pcd" cad_file:="cylinder.pcd"  thresh:=0.0001`

#### NEW! PCL - RANSAC + ICP SEAM DETECTION ! - NEEDS CLEANUP
##### use RANSAC + ICP for weld seam detection. For now it is just locating the origin of the part.
`roslaunch seam_detection seam_detection.launch lidar_file:="plate_cylinder.pcd" cad_file:="cylinder.pcd" thresh:=0.0001`

`roslaunch seam_detection seam_detection.launch lidar_file:="plate_cylinder_rotated.pcd" cad_file:="cylinder.pcd" thresh:=0.0001`

`roslaunch seam_detection seam_detection.launch lidar_file:="table_plate_cylinder.pcd" cad_file:="cylinder.pcd" thresh:=0.0001`




### THINGS TO DO

- I am migrating the code to 'TF2'. I hope this is a good idea. It has already fixed the 'static publisher issue'

- The migration has been stopped short. Apparently I still need tf::transform , seems wrong though, in the end I am using both ways

- It seems that I still need 'TF' for some things. tf::transform is still used for pcl_ros::transformPointCloud
   there is probably another way but I have not figured it out yet
- Also, in my REGISTER_CLOUD function I think I could go straight from ICP to TF and not pull out the value like it is done, but I guess it works like ###### that now

- Multiple parts needs to be developed , cylinder and plate seem to work now

I am currently adding 'markers' to show the results from RANSAC, the markers are there, but they need the pose info now. (03/06/2020)

- NEW THINGS TO DO
- First - all of the things not done in the list above. :)
- Next (or maybe first) - get demo ready for Friday robotics meeting - semi done! demo with NUC
- Also Next (or first) - create branch called 'stable' to store the working code
- to do this we need to check that it is stable, it seems like it is, fresh clone builds! woop!
- Then - begin migration to Fossa/Noetic - everything compiles  and returns
- I did run into a driver issue running the T1600 in Ubuntu 20.04 and Noetic, the driver for
- the old video card called 'nvidia-340' was not supported for the graphics libraries so I switched
- to the 'Nouveau' driver and this fixed the 'libgl error no matching fbconfigs or visuals found' issue
- after that everything runs but the ICP does not work. Actually it converges but the score is too high
- and the results do not make any sense. It works in Melodic but not in Noetic. This is a issue.
