#### seam_detection notes

##### BIG IDEA
 It appears that registration requires correspondence and/or overlapping point clouds. However, most sample code and algorithm testing is done on standard data sets, and the clouds in these sets have correspondance by nature of the test. For example, it is common to test registration on a cloud and a modified version of the same cloud. This test represents an ideal situation and best case inputs to the registration problem in which correspondence within a tolerance is expected.
 
 The applied registration problem for workpiece localization provides no guarantee that of correpondence between cloud points can be found. Check for this idea in the literature.

 
 - Being too 'far away' can cause ICP to fail. This may seem obvious, but I have not thought about the scale of initial translation until now. This is at least one thing I     have  learned from the new shape1_shape2 dataset.
 
 - 8 new scans have been recorded and saved with the project. These have all been tested with ICP registration and TEASER_FPFH registration.
 
 - registration using a CAD model based target cloud (fixed) and a LiDAR based source cloud (transformed) not always successful. Shape 1 is not successful in this mode, bu    t shape 2 is successful with ICP in several examples.  Some examples have minor alignment error. TEASER and TEASER_FPFH are not successful in this mode.
 
 - shape 1 is not successful in the examples in which alignment requires more than 180 of z rotation. This may be because the alignment must pass through a local minimum w    here both planes are parallel but offset which occurs at a 180 offset because the object is rectangular. This is somewhat confirmed by the fact that registration is succe    ssful if the source orientation is within X degs of target - X needs to be determined
 
 - SC suggested comparing LiDAR scan clouds to different LiDAR scan clouds. I do not know why we have not tried this yet. This mode seems to be more successful.     Registration is successful using ICP and TEASER_FPFH (needs more testing) in several shape1 and shape2 examples.
 
 - re-registration with ICP does not produce improved results - this is not suprising because iteration is built into the routine, max iterations is a parameter
 

 #### TEASER Notes
 
 Teaser is running on the the current data sets, however the results are not correct or usable.
 
 The translation component of the solution is mostly correct. It appears that the algorithm is successfully locating the centroid of the workpiece based on the ROS  visual    ization.
 
 The rotation portion of the solution is not correct. It is off by 30+ deg.
 
  
 #### More TEASER Notes
 
 If the number of source points (Ns) is less that the number of target points (Nt), seam_detection_teaser.cpp crashes during TEASER registration. This has been double chec    ked, Ns > Nt or TEASER will crash.
 
 If the number of source points is close to the number of target points, the algorithm converges quickly. Currently testing will N = ~1000
 
 Some of the data sets we are testing are not good because they have low overlap. This is partially due to the segmentation process and partly due to the lidar shadow conc    ept. Next, setup/find a ideal case for algorithm testing. I have a hunch that TEASER is working, but the data we are giving it is not great. CHECKTHISSOON!
 
 It seems that it is all about the input data, duh!
 
 it appears the python example may just be a wrapper on what we already have access to
 
 on the other hand it seems like the python_3dsmooth.py example is different, uses KNN 3DSmoothNet
 
 3DsmoothNet seemed interesting and the demo results are compeling, the downside is that the example is outdated in python 3.5,3.6, this is not what we want but we could u    se docker to make an old environment just for the sake of testing, it would be worth it I think
 
 New Stuff! - While invesigating the 3DsmoothNet author Zan Gojcic(zgojcic@github) I found something very interesting! Guess what it also has a catchy name: OverlapPredator [prs-eth/OverlapPredator](https://github.com/prs-eth/OverlapPredator). It seems like the main idea is "Registration of 3D Point Clouds with Low Overlap". This may be wh    at we need. Also, the tested example is for Python 3.8.5 which is much more acceptable than 3.5/3.6 as in previous example.
 
 Predator comes from the Photogrammetry and Remote Sensing Lab: https://github.com/prs-eth
 

## Neural Network Approach

Many algorithms and examples exist for 3D image analysis that use neural networks.
