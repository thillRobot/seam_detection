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
 
 New Stuff! - While invesigating the 3DsmoothNet author Zan Gojcic(zgojcic@github) I found something very interesting! Guess what it also has a catchy name: OverlapPredator [prs-eth/OverlapPredator](https://github.com/prs-eth/OverlapPredator). It seems like the main idea is "Registration of 3D Point Clouds with Low Overlap". This may be wh    at we need. Also, the tested example is for Python 3.8.5 which is much more acceptable than 3.5/3.6 as in previous example.
 
 Predator comes from the Photogrammetry and Remote Sensing Lab: https://github.com/prs-eth
 


## Neural Network Approach

Many algorithms and examples exist for 3D image analysis that use neural networks.



### Datasets

- ModelNet40 - Princeton - (https://modelnet.cs.princeton.edu/)
 - ModelNet10 - aligned subset 1
 - not clear if available for commericial use

- ScanNet - Stanford, Princeton, Tech. Univ. Munich - (http://www.scan-net.org/)
  - 20 classes from hotel room
  - non-commercial use only

- ShapeNet - Princeton, Stanford, TTIC - (https://shapenet.org/)
 - ShapeNetCore - Single Clean 3D models, category and alignment annotations, 55 categories
 - ShapeNetSem - category and alignment labels, dimensions, material, volume, weight, 270 categories
 - PartNet - Hierarchical part annotations from ShapeNet, instance segmenation
 - Use with PyTorch3d (https://github.com/facebookresearch/pytorch3d)
 - non-commercial use only

- Pix3D - data set for generating 3D images from 2D images
  - used in mesh-rcnn demo 

- Pascal 3D+, 12 categories, pose annotations, images in the wild

- ObjectNet3D - Stanford, Chris Choy

- 3DMatch - Princeton
  - Keypoint Matching, 3D Reconstruction, Geometric Registration
  
- KITTI - Karlsruhe Institute of Technology, Toyota Institute at Chicago (https://www.cvlibs.net/datasets/kitti/)
 - vision for autonomous vehicles
 - stereo vision, optical flow, visual odometry, 3D object detection, 3D tracking

- COCO - needs invesigating 


### Libraries/Frameworks

- PyTorch3D (https://github.com/facebookresearch/pytorch3d, updated 2024) 
  - tested by TH, required for mesh-rcnn

- Torch-Points3D (https://github.com/torch-points3d/torch-points3d, updated 2021)

- OpenPCDet (https://github.com/open-mmlab/OpenPCDet, updated 2023)
  - currently testing, possibly useful, designed for autonomous vehicles, consider implementing on Rover demo
  - possibly just a time sink and bad fit for this project
  - documentation is limited

- Minkowski Engine (https://github.com/NVIDIA/MinkowskiEngine/tree/master, updated 2021)
  - auto-differentiation library for sparse tensors. It supports all standard neural network layers, NVIDIA  
  - tested by TH
  - TH wrote basic 3D pixel classifier based on Chris Choys examples and scannet
  - consider re-visiting thisa, at least dusting off example from Dec 2023

- Detectron2 - library for detection and segmentation algorithms (https://github.com/facebookresearch/detectron2?tab=readme-ov-file), facebook 
  - contains implementations of Faster and Mesh R-CNN
  - tested by TH



### Algorithms/Tasks/Methods

- Classification
  "supervised learning task where the goal is to categorize input data into one of several predefined classes or categories. It involves training a model to learn the relationship between input features and their corresponding class labels" - chatgpt3.5

- Regression
  "supervised learning task where the goal is to predict continuous numerical values rather than discrete class labels. In other words, regression models learn to map input data to a continuous output space." - chatgpt3.5

- Segmentation 

  - Semantic Segmentation
    "the goal is to categorize each pixel in an image into a class or object. The goal is to produce a dense pixel-wise segmentation map of an image, where each pixel is assigned to a specific class or object. " (https://paperswithcode.com/task/semantic-segmentation)

    "classify each pixel in an image into a specific class or category, without distinguishing between different instances of the same class."

    - semantic segmentation process 

    1) Input Image: The input to a semantic segmentation model is typically a single image.

    2) Pixel-Level Classification: Semantic segmentation algorithms analyze the entire image and classify each pixel into predefined classes or categories. These classes could include objects such as people, cars, buildings, roads, trees, etc.

    3) Output Map: The output of semantic segmentation is a labeled image where each pixel is assigned a color or a numerical value corresponding to the class it belongs to. This output is often referred to as a segmentation mask or semantic map.

    - chatgpt3.5

  - Instance Segmentation - identifying and separating individual objects within an image (https://paperswithcode.com/task/instance-segmentation) 
  - "identifying and delineating individual objects within an image while also distinguishing between different object instances of the same class"

    - instance segmentation process:

    1) Object Detection: Instance segmentation often begins with object detection, where the goal is to identify and localize objects within an image. This step typically involves using techniques such as region proposal networks or anchor-based methods to generate candidate object bounding boxes.

    2) Segmentation: After detecting objects, the next step is to segment each object instance from the background and from other objects in the image. Unlike semantic segmentation, which groups pixels into classes without distinguishing between instances, instance segmentation aims to assign a unique label to each individual object instance.

    3) Mask Generation: In instance segmentation, each detected object is associated with a binary mask that precisely outlines its boundaries. These masks indicate which pixels in the image belong to the object and which belong to the background.

    4) Object Classification: In addition to segmenting objects, instance segmentation may also involve classifying each object instance into specific categories or classes. This step helps in understanding not just where objects are in the image but also what they are.

    - chatgpt3.5

  - Panoptic Segmentation - combination of semantic and instance segmentation


- Detection 
  - 2D - lots of examples
  - 3D - very few examples 

- Reconstruction 

- Completion

- Model Alignment

- Surface Correspondence

- Shape Retrieval

Concept: Transfer Learning 
Transfer learning with respect to Convolutional Neural Networks (CNNs) is a technique where a pre-trained model, which has been trained on a large dataset for a particular task, is adapted for a related task or dataset.

In CNNs, the lower layers of the network typically learn to extract low-level features such as edges and textures, while the higher layers learn to extract more abstract features relevant to the specific task the network was trained on. Transfer learning leverages these learned features by reusing a pre-trained model's architecture and weights and fine-tuning them on a new dataset or task.

There are typically two main approaches to transfer learning in CNNs:

   1) Feature Extraction: In this approach, you take a pre-trained CNN model and remove the fully connected layers at the top of the network. Then, you replace these layers with new ones that are appropriate for your new task. The weights of the pre-trained layers are frozen, meaning they are not updated during training, and only the weights of the new layers are learned from scratch using the new dataset.

   2) Fine-tuning: In this approach, you not only replace the fully connected layers at the top of the network but also fine-tune some of the pre-trained layers by unfreezing them and allowing their weights to be updated during training on the new dataset. This is particularly useful when the new dataset is significantly different from the original dataset used to train the pre-trained model.

Transfer learning is widely used in practice, especially in scenarios where there is limited availability of labeled data for training deep learning models from scratch. It allows leveraging the knowledge gained from large datasets and applying it to new, related tasks, often leading to improved performance and faster convergence.


### Object Detection Algorithms/Models

- Region-based Convolutional Neural Network 
  - R-CNN 
    1) Region Proposal: The first step of the R-CNN pipeline is to generate region proposals, which are candidate bounding boxes that might contain objects. Initially, a region proposal method such as Selective Search is used to generate a set of potential object bounding boxes in the image. These candidate regions are then passed through the subsequent stages for further processing.

    2) Feature Extraction: Each proposed region is warped to a fixed size and fed into a pre-trained convolutional neural network (CNN), typically models like AlexNet, VGGNet, or ResNet. The CNN extracts a fixed-length feature vector from each region, encoding information about the region's content.

    3) Classification and Localization: The feature vectors extracted from the proposed regions are then used as input to two separate fully connected layers: one for object classification and one for bounding box regression. The classification layer predicts the probability of each region containing an object of interest, while the regression layer refines the coordinates of the bounding box enclosing the object if one is present.

    4) Non-Maximum Suppression (NMS): After classification and regression, a post-processing step called non-maximum suppression is applied to remove duplicate detections and refine the final bounding boxes. NMS ensures that only the most confident and non-overlapping bounding boxes are retained as the output.

  - Note: R-CNN uses combintation of classification and regression to identify and locate the object  

  - Faster R-CNN
  - Fast R-CNN

  - PointRCNN - first two-stage 3D object detector for 3D object detection by using only the raw point cloud as input. 
  
  - Mesh R-CNN -  objects in real-world images (2D) and produces a triangle mesh giving the full 3D shape of each detected object - tested, uses detectron2 and pytorch3d
  
  - wikipedia (https://en.wikipedia.org/wiki/Region_Based_Convolutional_Neural_Networks) 
    " take an input image and produce a set of bounding boxes as output, where each bounding box contains an object and also the category (e.g. car or pedestrian) of the object "
  - benchmark leaders in object detection 

- combined 3D and 2D (YOLOv3 + Frustrum PointNet)

- 3DETR (https://github.com/facebookresearch/3detr)
  - 3D DEtection TRansformer (3D detection with transformers)

  - " a simpler alternative to complex hand-crafted 3D detection pipelines "

- VoteNet (https://github.com/facebookresearch/votenet, https://github.com/qq456cvb/VoteNet)
  - end to end NN based on hough voting

- PointNet - Stanford - (https://github.com/charlesq34/pointnet)
  - Neural network architechture designed for direct use with pointclouds
  - 2016-2017

- Frustrum PointNet (uses TensorFLow)

- PIXOR 
  - oriented 3D object estimates, autonomous driving context



### Image Reconstruction

- 3D-R2R2: 3D Recurrent Reconstruction Neural Network (http://3d-r2n2.stanford.edu/, https://github.com/chrischoy/3D-R2N2/)
  - given one or multiple views of an object, the network generates voxlized reconstruction of the object in 3D
  - uses ShapeNet dataset
  - from Chris Choy, Stanford, Nvidia  




