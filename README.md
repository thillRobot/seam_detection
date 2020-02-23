# seam_detection

##### This is a ROS package for weld seam detection using pointcloud data.

##### Here are some example uses of the package.

##### These examples require a pre-recorded pointcloud from a 3D Lidar and/or CAD. There are example scans here.

##### Process the pointcloud using PCL-SEGMENTATION

`$ roslaunch seam_detection segment_plane.launch in_file:="test_cloud8.pcd" thresh:=0.01`

##### try this one pcd_to_pointcloud - THIS WORKS (02/04/2020)

`$ roslaunch seam_detection segment_plane_line.launch in_file:="test_cloud11.pcd" thresh:=0.01`

##### Iterative Closest Point (ICP)

##### Import a cloud from a CAD model. Currently it works like this - from Solidworks(.stl)-->meshlab(.ply)-->cad2cloud(.pcd)-->ROS(pointcloud!)

##### Step 1) Make a part in your CAD program of choice. This part will become the 'reference cloud'. Save the file as a '.stl' file. If your CAD program can create a '.ply' or '.pcd' file you can skip steps 2 or 3 respectively.

##### Step 2) Convert the '.stl' to a '.ply' file using meshlab. Open the '.stl' using 'import mesh' and save it as a '.ply' file. Step2 can be done in solidworks. This combines step and step 2. Nice

##### Step 3) Convert the '.ply' to a '.pcd' file using cad_cloud (this is a package I wrote that uses PCL). run the following commands.

`this should be a code block right?`

`rosrun seam_detection cad_cloud ply_images/input.ply output.pcd -n_samples 100000 -leaf_size 0.001 -write_normals 1 -no_vis_results 0`

`pcl_viewer -multiview 1 output.pcd`

##### Step 4) Use ICP to compare the CAD/reference image to the LIDAR/source image. The LIDAR '.pcd' file must also be in the image directory.

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene1.pcd" cad_file:="cad_scene1.pcd"  thresh:=0.003`

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene2.pcd" cad_file:="cad_scene2.pcd"  thresh:=0.003`

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene3.pcd" cad_file:="cad_scene3.pcd"  thresh:=0.003`

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="lidar_scene3.pcd" cad_file:="cad_scene4.pcd"  thresh:=0.003`

##### Ok, well if that works I dont see why we cant do 2 parts...


`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="cylinder.pcd" cad_file:="plate_cylinder.pcd"  thresh:=0.003`
##### The process converges, we get a good score (if low is good, .00032)

##### Try this!

`roslaunch seam_detection seam_detection_ICP.launch lidar_file:="plate_cylinder.pcd" cad_file:="cylinder.pcd"  thresh:=0.0001`
