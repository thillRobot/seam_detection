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


#### IDETC 2024 Notes

##### Datasets

  - `part1_x4_y9_theta0`
  - `part1_x4_y9_theta0_part2_clamp_glove`

  - `part1_x5_y10_theta45` - 
  - `part1_x5_y10_theta45_part2_clamp_glove`
  
  - `part1_x9_y7_theta90`
  - `part1_x9_y7_theta90_part2_clamp_glove`
   
  - `part1_x9_y7_theta90`
  - `part1_x9_y7_theta90_part2_clamp_glove`

  - `part1_x9_y7_theta90`
  - `part1_x9_y7_theta90_part2_clamp_glove`

  - `part1_x3_y9_theta0`
  - `part1_x3_y9_theta0_part2_clamp_glove`
  
  - `part1_x7_y5_theta45_part2_clamp_glove`
  
  - `part1_x3_y11_theta135_part2_clamp_glove`

  - `part1_x4_y5_theta45_part2_clamp_glove`

  - `part1_x9_y2_theta90`

  - `part1_x8_y6_theta30_part2_clamp_glove`


##### Datasets to use in IDETC2024
###### set1
training1:
target: (idx=0)
 `bags/reconstruction/part1_x3_y9_theta0_5_output.pcd`

test1:
source:
 `bags/reconstruction/part1_x7_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.713715,-0.70011,-0.0213854,
0.70042,0.713162,0.0284788,
-0.00468702,-0.0353045,0.999366]
Measured Translation: [-0.222094,-0.159648,-0.0205973]
P_target: [0.0127,-0.4953,0.0508]
P_source: [0.132648,-0.502535,0.0475973]


test2:
source:
 `bags/reconstruction/part1_x4_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[-0.713664,0.697676,0.062713,
-0.700364,-0.712361,-0.0450758,
0.013226,-0.0760909,0.997013]
Measured Translation: [0.148246,-1.13778,-0.0369559]
P_target: [0.0127,-0.4953,0.0508]
P_source: [-0.203191,-0.796131,0.0515481]

test3:
source
 `bags/reconstruction/part1_x3_y11_theta135_5_output.pcd`

Measured Rotation Matrix:
[0.701546,0.712319,-0.0208524,
-0.712488,0.700541,-0.0400354,
-0.01391,0.0429438,0.998981]
Measured Translation: [0.347233,-0.349363,0.0182425]
P_target: [0.0127,-0.4953,0.0508]
P_source: [0.00227234,-0.707423,0.047544]

test4:
source
 `bags/reconstruction/part1_x4_y9_theta0_5_output.pcd`

Measured Rotation Matrix:
[0.999837,0.0123178,-0.0131965,
-0.0117661,0.999086,0.0410973,
0.0136907,-0.0409354,0.999068]
Measured Translation: [0.103524,-0.00317029,-0.0163166]
P_target: [0.0127,-0.4953,0.0508]
P_source: [0.10945,-0.496079,0.0548852]

test5:
source
 `bags/reconstruction/part1_x8_y6_theta30_5_output.pcd`

Measured Rotation Matrix:
[-0.904444,-0.425039,-0.0363863,
0.426576,-0.901875,-0.0682162,
-0.00382135,-0.0772193,0.997007]
Measured Translation: [-0.26533,-1.10214,-0.0429448]
P_target: [0.0127,-0.4953,0.0508]
P_source: [-0.0681429,-0.653487,0.0459013]

test6:
source
 `bags/reconstruction/part1_x5_y10_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.706805,0.706776,0.0298957,
-0.707401,0.705974,0.0344207,
0.00322214,-0.045477,0.99896]
Measured Translation: [0.422514,-0.386578,-0.0202862]
P_target: [0.0127,-0.4953,0.0508]
P_source: [0.0829428,-0.743482,0.0530266]

###### set2
training2:
target: (idx=4)
 `bags/reconstruction/part1_x9_y2_theta90_5_output.pcd`

test1:
source:
 `bags/reconstruction/part1_x7_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[-0.698321,-0.714412,-0.0443162,
0.713371,-0.699711,0.0388073,
-0.0587329,-0.00451399,0.998264]
Measured Translation: [-0.500163,-1.14514,-0.0053346]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [0.172914,-0.534973,0.0517138]

test2:
source:
 `bags/reconstruction/part1_x4_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.703148,0.704752,0.0943776,
-0.701551,0.709239,-0.0693323,
-0.115798,-0.0174598,0.993119]
Measured Translation: [0.418683,-0.145506,-0.0135196]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [-0.243262,-0.766323,0.0571971]

test3:
source
 `bags/reconstruction/part1_x3_y11_theta135_5_output.pcd`

Measured Rotation Matrix:
[0.701449,-0.711309,-0.0448233,
0.708,0.70265,-0.0708472,
0.0818893,0.0179608,0.99648]
Measured Translation: [-0.644716,-0.0811862,0.0126153]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [-0.0278142,-0.749801,0.0438072]


test4: (this test fails by 180 deg)
source
 `bags/reconstruction/part1_x4_y9_theta0_5_output.pcd`
Measured Rotation Matrix:
[-0.162278,0.985452,-0.0504945,
0.00772338,0.0524397,0.998594,
0.986715,0.161659,-0.0161208]
Measured Translation: [0.651191,-0.518365,0.176533]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [-0.240031,-0.515549,-0.00867485]

test5: (this test fails by 180 deg)
source
 `bags/reconstruction/part1_x8_y6_theta30_5_output.pcd`

Measured Rotation Matrix:
[0.417718,-0.907752,-0.0387098,
0.90822,0.418369,-0.0102143,
0.025467,-0.0308903,0.999198]
Measured Translation: [-0.556542,-0.382605,-0.0240121]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [0.24986,-0.797627,0.0538268]


test6: 
source
 `bags/reconstruction/part1_x5_y10_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.708394,-0.705815,-0.00186529,
0.705773,0.708376,-0.00931961,
0.00789925,0.00528548,0.999955]
Measured Translation: [-0.563405,-0.115711,0.00713087]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [0.0504262,-0.786315,0.0528281]





