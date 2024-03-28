

debugging aubo robot model from URDF not displaying correctly


example output from aubo (printed on ubuntu20-nuc8i7inh)
`rostopic echo /joint_states'

```
header: 
  seq: 37898
  stamp: 
    secs: 1690843888
    nsecs: 108381937
  frame_id: "joint_space"
name: 
  - shoulder_joint
  - upperArm_joint
  - foreArm_joint
  - wrist1_joint
  - wrist2_joint
  - wrist3_joint
position: [-1.8086564253668616, 0.04883234589825318, 2.5320549997526482, 1.076083261030979, 1.6694624494895896, 1.6329254772511719]
velocity: []
effort: []
```




testing with `aubo_test.launch`


