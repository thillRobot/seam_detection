
#### IDETC 2024 Notes

##### Datasets

  - [x,x,] `part1_x4_y9_theta0` 
  - [x,x,] `part1_x4_y9_theta0_part2_clamp_glove` # problem with segment_clouds

  - [x,x,] `part1_x5_y10_theta45`  
  - [x,x,] `part1_x5_y10_theta45_part2_clamp_glove`

  - [x,x,] `part1_x9_y7_theta90`
  - [x,x,] `part1_x9_y7_theta90_part2_clamp_glove` (10,11,12,14,15)

  - [x,x,] `part1_x5_y6_theta45`  
  - [x,x,] `part1_x5_y6_theta45_part2_clamp_glove`

  - [x,x,] `part1_x3_y9_theta0`
  - [x,x,] `part1_x3_y9_theta0_part2_clamp_glove`
 
  - [x,x,] `part1_x4_y8_theta0`
  - [x,x,] `part1_x4_y8_theta0_part2_clamp_glove`

  - [x,x,] `part1_x2_y4_theta0`

  - [x,x,] `part1_x9_y2_theta90`

  - [x,x,] `part1_x7_y5_theta45_part2_clamp_glove`
  
  - [x,x,] `part1_x3_y11_theta135_part2_clamp_glove` (13,14,15)

  - [x,x,] `part1_x4_y5_theta45_part2_clamp_glove`

  - [x,x,] `part1_x8_y6_theta30_part2_clamp_glove` (10, 11, 12)

  - [] `part1_x5_y5_theta0`
  - [] `part1_x10_y2_theta45_part2_clamp_glove`



##### garage scans, do not use

  - [?] `part1_x4_y5_theta45_part2...` 


##### Datasets to use in IDETC2024
###### set1
training1:
target: (idx=0)
 `bags/reconstruction/part1_x3_y9_theta0_5_output.pcd`

(no registration data for single cloud)


test1:
source:
 `bags/reconstruction/part1_x7_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.713715,-0.70011,-0.0213854,
0.70042,0.713162,0.0284788,
-0.00468702,-0.0353045,0.999366]
Expected Rotation Matrix:
[0.707107,-0.707107,0,
0.707107,0.707107,0,
0,0,1]
Measured Axis Rotations: [ -0.0353122, 0.00468703, 0.775997 ]
Expected Axis Rotations: [ 0, -0, 0.785398 ]
Difference Axis Rotations: [ -0.0353122, -0.00940101, 0.00468703 ]
P_target: [0.0127,-0.5461,0.0508]
P_source: [0.168214,-0.538763,0.0493907]
P_target_source: [-0.155514,-0.00733667,0.00140926]
P_expected_source: [0.1651,-0.5334,0.0508]
P_target_expected_source: [-0.155514,-0.00733667,0.00140926]
P_diff: [-0.00311395,0.00536336,0.00140926]


test2:
source
 `bags/reconstruction/part1_x3_y11_theta135_5_output.pcd`

Measured Rotation Matrix:
[0.701546,0.712319,-0.0208524,
-0.712488,0.700541,-0.0400354,
-0.01391,0.0429438,0.998981]
Expected Rotation Matrix:
[0.707107,0.707107,0,
-0.707107,0.707107,-0,
-0,0,1]
Measured Axis Rotations: [ 0.0429611, 0.0139104, -0.793136 ]
Expected Axis Rotations: [ 0, 0, -0.785398 ]
Difference Axis Rotations: [ 0.0429611, -0.00773768, 0.0139104 ]
P_target: [0.0127,-0.5461,0.0508]
P_source: [-0.0339135,-0.743011,0.0453624]
P_target_source: [0.0466135,0.196911,0.00543755]
P_expected_source: [-0.02,-0.740001,0.0508]
P_target_expected_source: [0.0466135,0.196911,0.00543755]
P_diff: [0.0139135,0.00300968,0.00543755]


test3:
source:
 `bags/reconstruction/part1_x4_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[-0.713664,0.697676,0.062713,
-0.700364,-0.712361,-0.0450758,
0.013226,-0.0760909,0.997013]
Expected Rotation Matrix:
[-0.707107,0.707107,0,
-0.707107,-0.707107,-0,
-0,0,1]
Measured Axis Rotations: [ -0.0761712, -0.0132264, -2.3656 ]
Expected Axis Rotations: [ 0, -0, -2.35619 ]
Difference Axis Rotations: [ -0.0761712, -0.00940539, -0.0132264 ]
P_target: [0.0127,-0.5461,0.0508]
P_source: [-0.238633,-0.759943,0.0554135]
P_target_source: [0.251333,0.213843,-0.00461353]
P_expected_source: [-0.254,-0.762,0.0508]
P_target_expected_source: [0.251333,0.213843,-0.00461353]
P_diff: [-0.0153674,-0.00205662,-0.00461353]


test4:
source
 `bags/reconstruction/part1_x4_y9_theta0_5_output.pcd`

Measured Rotation Matrix:
[0.999837,0.0123178,-0.0131965,
-0.0117661,0.999086,0.0410973,
0.0136907,-0.0409354,0.999068]
Expected Rotation Matrix:
[1,0,0,
0,1,0,
0,0,1]
Measured Axis Rotations: [ -0.0409507, -0.0136911, -0.0117675 ]
Expected Axis Rotations: [ 0, -0, 0 ]
Difference Axis Rotations: [ -0.0409507, -0.0117675, -0.0136911 ]
P_target: [0.0127,-0.5461,0.0508]
P_source: [0.108824,-0.546833,0.0569648]
P_target_source: [-0.0961243,0.00073277,-0.00616476]
P_expected_source: [0.127,-0.5461,0.0508]
P_target_expected_source: [-0.0961243,0.00073277,-0.00616476]
P_diff: [0.0181757,0.00073277,-0.00616476]

test5:
source
 `bags/reconstruction/part1_x8_y6_theta30_5_output.pcd`

Measured Rotation Matrix:
[-0.904444,-0.425039,-0.0363863,
0.426576,-0.901875,-0.0682162,
-0.00382135,-0.0772193,0.997007]
Expected Rotation Matrix:
[-0.866025,-0.5,0,
0.5,-0.866025,0,
0,0,1]
Measured Axis Rotations: [ -0.0772968, 0.00382136, 2.70089 ]
Expected Axis Rotations: [ 0, -0, 2.61799 ]
Difference Axis Rotations: [ -0.0772968, 0.0828916, 0.00382136 ]
P_target: [0.0127,-0.5461,0.0508]
P_source: [-0.046551,-0.607672,0.0498241]
P_target_source: [0.059251,0.0615722,0.000975916]
P_expected_source: [-0.0762,-0.6096,0.0508]
P_target_expected_source: [0.059251,0.0615722,0.000975916]
P_diff: [-0.029649,-0.00192783,0.000975916]


test6:
source
 `bags/reconstruction/part1_x5_y10_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.706805,0.706776,0.0298957,
-0.707401,0.705974,0.0344207,
0.00322214,-0.045477,0.99896]
Expected Rotation Matrix:
[0.707107,0.707107,0,
-0.707107,0.707107,-0,
-0,0,1]
Measured Axis Rotations: [ -0.0454929, -0.00322214, -0.785819 ]
Expected Axis Rotations: [ 0, 0, -0.785398 ]
Difference Axis Rotations: [ -0.0454929, -0.000421207, -0.00322214 ]
P_target: [0.0127,-0.5461,0.0508]
P_source: [0.0470386,-0.779346,0.0553369]
P_target_source: [-0.0343386,0.233246,-0.00453686]
P_expected_source: [0.0508,-0.775,0.0508]
P_target_expected_source: [-0.0343386,0.233246,-0.00453686]
P_diff: [0.00376143,0.00434617,-0.00453686]


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
Expected Rotation Matrix:
[-0.707107,-0.707107,0,
0.707107,-0.707107,0,
0,0,1]
Measured Axis Rotations: [ -0.00452181, 0.0587667, 2.34553 ]
Expected Axis Rotations: [ 0, -0, 2.35619 ]
Difference Axis Rotations: [ -0.00452181, -0.0106612, 0.0587667 ]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [0.172914,-0.534973,0.0517138]
P_target_source: [-0.211014,-0.373077,-0.000913844]
P_expected_source: [0.1651,-0.5334,0.0508]
P_target_expected_source: [-0.211014,-0.373077,-0.000913844]
P_diff: [-0.00781368,0.0015732,-0.000913844]

test2:
source
 `bags/reconstruction/part1_x3_y11_theta135_5_output.pcd`


Measured Rotation Matrix:
[0.701449,-0.711309,-0.0448233,
0.708,0.70265,-0.0708472,
0.0818893,0.0179608,0.99648]
Expected Rotation Matrix:
[0.707107,-0.707107,0,
0.707107,0.707107,0,
0,0,1]
Measured Axis Rotations: [ 0.0180223, -0.0819811, 0.790046 ]
Expected Axis Rotations: [ 0, -0, 0.785398 ]
Difference Axis Rotations: [ 0.0180223, 0.00464753, -0.0819811 ]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [-0.0278142,-0.749801,0.0438072]
P_target_source: [-0.0102858,-0.158249,0.00699284]
P_expected_source: [-0.02,-0.740001,0.0508]
P_target_expected_source: [-0.0102858,-0.158249,0.00699284]
P_diff: [0.00781423,0.00980047,0.00699284]

test3:
source:
 `bags/reconstruction/part1_x4_y5_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.703148,0.704752,0.0943776,
-0.701551,0.709239,-0.0693323,
-0.115798,-0.0174598,0.993119]
Expected Rotation Matrix:
[0.707107,0.707107,0,
-0.707107,0.707107,-0,
-0,0,1]
Measured Axis Rotations: [ -0.0175789, 0.116059, -0.784261 ]
Expected Axis Rotations: [ 0, 0, -0.785398 ]
Difference Axis Rotations: [ -0.0175789, 0.00113754, 0.116059 ]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [-0.243262,-0.766323,0.0571971]
P_target_source: [0.205162,-0.141727,-0.00639709]
P_expected_source: [-0.254,-0.762,0.0508]
P_target_expected_source: [0.205162,-0.141727,-0.00639709]
P_diff: [-0.0107377,0.00432344,-0.00639709]

test4: (this test fails by 180 deg)
source
 `bags/reconstruction/part1_x4_y9_theta0_5_output.pcd`


test5: (this test fails by 180 deg)
source
 `bags/reconstruction/part1_x8_y6_theta30_5_output.pcd`


test6: 
source
 `bags/reconstruction/part1_x5_y10_theta45_5_output.pcd`

Measured Rotation Matrix:
[0.708394,-0.705815,-0.00186529,
0.705773,0.708376,-0.00931961,
0.00789925,0.00528548,0.999955]
Expected Rotation Matrix:
[0.707107,-0.707107,0,
0.707107,0.707107,0,
0,0,1]
Measured Axis Rotations: [ 0.00528567, -0.00789933, 0.783545 ]
Expected Axis Rotations: [ 0, -0, 0.785398 ]
Difference Axis Rotations: [ 0.00528567, -0.00185272, -0.00789933 ]
P_target: [-0.0381,-0.90805,0.0508]
P_source: [0.0504262,-0.786315,0.0528281]
P_target_source: [-0.0885262,-0.121735,-0.00202814]
P_expected_source: [0.0508,-0.775,0.0508]
P_target_expected_source: [-0.0885262,-0.121735,-0.00202814]
P_diff: [0.000373826,0.0113155,-0.00202814]




|---------- SeamDetection::scoreCloudsMulti() ----------|
scores[0]:1.88497
scores[1]:5.84155
scores[2]:8.7151
scores[3]:17.0616
scores[4]:10.1461
scores_min: 1.88497 found at 0
|---------- SeamDetection::scoreCloudsMulti() ----------|
scores[0]:4.55081
scores[1]:7.57289
scores[2]:15.2325
scores[3]:4.67655






scores[0]:2.18797
scores[1]:7.677
scores[2]:6.37832
scores[3]:17.3737
scores[4]:5.22705
scores_min: 2.18797 found at 0
|---------- SeamDetection::scoreCloudsMulti() ----------|
scores[0]:0.511444
scores[1]:6.70079
scores[2]:36.6166
scores[3]:7.31992
scores_min: 0.511444 found at 0
|---------- SeamDetection::scoreCloudsMulti() ----------|
scores[0]:4.61589
scores[1]:7.01587
scores[2]:30.2058
scores_min: 4.61589 found at 0
|---------- SeamDetection::scoreCloudsMulti() ----------|
scores[0]:7.52265
scores[1]:6.47735
scores_min: 6.47735 found at 1

