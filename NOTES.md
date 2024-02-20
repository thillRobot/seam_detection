#### seam_detection notes
 
 - Being too 'far away' can cause ICP to fail. This may seem obvious, but I have not thought about the scale of initial translation until now. This is at least one thing I     have  learned from the new shape1_shape2 dataset.
 
 - 8 new scans have been recorded and saved with the project. These have all been tested with ICP registration and TEASER_FPFH registration.
 
 - registration using a CAD model based target cloud (fixed) and a LiDAR based source cloud (transformed) not always successful. Shape 1 is not successful in this mode, bu    t shape 2 is successful with ICP in several examples.  Some examples have minor alignment error. TEASER and TEASER_FPFH are not successful in this mode.
 
 - shape 1 is not successful in the examples in which alignment requires more than 180 of z rotation. This may be because the alignment must pass through a local minimum w    here both planes are parallel but offset which occurs at a 180 offset because the object is rectangular. This is somewhat confirmed by the fact that registration is succe    ssful if the source orientation is within X degs of target - X needs to be determined
 
 - Dr. Canfield suggested comparing LiDAR scan clouds to different LiDAR scan clouds. I do not know why we have not tried this yet. This mode seems to be more successful.     Registration is successful using ICP and TEASER_FPFH (needs more testing) in several shape1 and shape2 examples.
 
 - re-registration with ICP does not produce improved results - this is not suprising because iteration is built into the routine, max iterations is a parameter
 
 
 
 #### TEASER Notes
 
 Teaser is running on the the current data sets, however the results are not correct or usable.
 
 The translation component of the solution is mostly correct. It appears that the algorithm is successfully locating the centroid of the workpiece based on the ROS  visual    ization.
 
 The rotation portion of the solution is not correct. It is off by 30+ deg.
 
 ##### BIG IDEA
 It appears that registration requires correspondence and/or overlapping point clouds. However, most sample code and algorithm testing is done on standard data sets, and t    he clouds in these sets have correspondance by nature of the test. For example, it is common to test registration on a cloud and a modified version of the same cloud. Thi    s test represents an ideal situation and best case inputs to the registration problem in which correspondence within a tolerance is expected.
 
 The applied registration problem for workpiece localization provides no guarantee that of correpondence between cloud points can be found. Check for this idea in the lite    rature.
 
 
 #### More TEASER Notes
 
 If the number of source points (Ns) is less that the number of target points (Nt), seam_detection_teaser.cpp crashes during TEASER registration. This has been double chec    ked, Ns > Nt or TEASER will crash.
 
 If the number of source points is close to the number of target points, the algorithm converges quickly. Currently testing will N = ~1000
 
 Some of the data sets we are testing are not good because they have low overlap. This is partially due to the segmentation process and partly due to the lidar shadow conc    ept. Next, setup/find a ideal case for algorithm testing. I have a hunch that TEASER is working, but the data we are giving it is not great. CHECKTHISSOON!
 
 It seems that it is all about the input data, duh!
 
 it appears the python example may just be a wrapper on what we already have access to
 
 on the other hand it seems like the python_3dsmooth.py example is different, uses KNN 3DSmoothNet
 
 3DsmoothNet seemed interesting and the demo results are compeling, the downside is that the example is outdated in python 3.5,3.6, this is not what we want but we could u    se docker to make an old environment just for the sake of testing, it would be worth it I think
 
 New Stuff! - While invesigating the 3DsmoothNet author Zan Gojcic(zgojcic@github) I found something very interesting! Guess what it also has a catchy name: OverlapPredato    r [prs-eth/OverlapPredator](https://github.com/prs-eth/OverlapPredator). It seems like the main idea is "Registration of 3D Point Clouds with Low Overlap". This may be wh    at we need. Also, the tested example is for Python 3.8.5 which is much more acceptable than 3.5/3.6 as in previous example.
 
 Predator comes from the Photogrammetry and Remote Sensing Lab: https://github.com/prs-eth
 
 
 ### troubleshoot with auborobot
 
 To use this package with the aubo robot, uncomment and enable compiling of the aubo robot system node in `CMakeLists.txt`. These nodes will not compile by default to allo    w for use in docker and on other platforms.
 
 
 #### issues
 i see the following error when i try to publish to andriod_gui/gcode_cmd
 ```
 [ERROR] [1692989638.343471973]: Client [/rostopic_17739_1692988617268] wants topic /android_gui/gcode_cmd to have datatype/md5sum [aubo_control/gcodeAction/8261e41e538034    94ec669905817b139c], but our version has [aubo_control/gcodeAction/a83a0e1a726f23e73947f0f4e478e627]. Dropping connection.
 ```
#### issues
 i see the following error when i try to publish to andriod_gui/gcode_cmd
 ```
 [ERROR] [1692989638.343471973]: Client [/rostopic_17739_1692988617268] wants topic /android_gui/gcode_cmd to have datatype/md5sum [aubo_control/gcodeAction/8261e41e538034    94ec669905817b139c], but our version has [aubo_control/gcodeAction/a83a0e1a726f23e73947f0f4e478e627]. Dropping connection.
 ```
 
 i think this is a noetic-kinetic version mismatch but I am not sure. The custom message compiles and published on the local machine fine, neither remote machine can see t    he msg from the other computer
 
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

