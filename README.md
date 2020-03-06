# seam_detection

##### This is a ROS package for weld seam detection using pointcloud data.

##### Here are some example uses of the package.

##### These examples require a pre-recorded pointcloud from a 3D Lidar and/or CAD. There are example scans here.

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

##### THINGS TO DO
#####
###### I am migrating the code to 'TF2'. I hope this is a good idea. It has already fixed the 'static publisher issue'

###### The migration has been stopped short. Apparently I still need tf::transform , seems wrong though

###### It seems that I still need 'TF' for some things. tf::transform is still used for pcl_ros::transformPointCloud
###### there is proably another way but I havent figured it out yet
######
###### Also, in my REGISTER_CLOUD function I think I could go straight from ICP to TF and not pul out the value like it is done, but I guess it works like ###### that now
######
###### Multiple parts needs to be developed
######
###### I am currently adding 'markers' to show the results from RANSAC, the markers are there, but they need the pose info now. (03/06/2020)
