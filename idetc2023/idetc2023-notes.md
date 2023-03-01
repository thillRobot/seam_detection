# IDETC 2023 - Notes

## concepts and brainstorming

point cloud registration / 3D image registration 


three main steps (as decsribed by Zan Zodjcic)

first interest points are decribed by sampling the local neighborhoods with high dimensional feature descriptors

next feature descriptors are matched to determine correspondance

finally registration is performed through robust minimization of the distance between corresponding points



registration recall

In the context of image registration, registration recall is a performance metric used to evaluate the accuracy of an image registration algorithm. Image registration is the process of aligning two or more images of the same scene, taken from different viewpoints or at different times, to a common coordinate system.

Registration recall measures the percentage of the reference image (or target image) that is correctly captured by the registered image (or source image) after registration. It is calculated by dividing the number of correctly registered pixels in the registered image by the total number of pixels in the reference image.

In other words, registration recall indicates how much of the reference image is correctly registered by the image registration algorithm. A high registration recall indicates a more accurate registration, while a low registration recall indicates a less accurate registration.

Registration recall is often used in conjunction with other performance metrics, such as registration precision, which measures the alignment error between the registered and reference images. Together, these metrics provide a comprehensive evaluation of the accuracy of an image registration algorithm. (chatgpt- this seems a bit forced, I had to provide the context)


weld seam detection 

robot arms and cobots

workspace sensing and workpeice detection

optimization based algorithms

neural network approaches

overlapping point clouds

overlap ratio 

Zan agrees with me that the traditional benchmarks only include high overlapping pairs of clouds
(one or two examples would be nice for the paper)




correspondances-based vs correspondances-free registration

deep neural network - multi layered neural network 

auto-encoders
	an artificial neural network used to learn efficient codings of unlabeled data (unsupervised learning) - wikipedia

region proposal network 

end to end learning algorithms
	End-to-end learning is a type of machine learning where a model is trained to perform a task from end-to-end, i.e., the model is given raw data as input and is expected to produce the desired output without the need for any additional preprocessing or postprocessing. The idea is to allow the model to learn the most efficient way to map the input data to the desired output, without relying on human-engineered features or preprocessing steps. This can make the training process more efficient and can also lead to better performance on the task, since the model is able to learn features directly from the raw data that are specifically relevant to the task at hand. (chatGPT, OpenAI)

SOTA - State of The Art efficiency
in computer science, "state of the art" refers to the current best practices, techniques, and technologies that are available for solving a particular problem. When it comes to efficiency, this typically refers to the fastest or most efficient way of solving a problem using the currently available tools and technologies.

The exact definition of "state of the art" can vary depending on the specific context and the problem that is being solved. For example, in some cases, it may refer to the fastest or most efficient algorithm for solving a problem, while in other cases it may refer to the most advanced hardware or software tools that are available for solving a problem.

Overall, the goal of state of the art efficiency is to find the best way of solving a problem using the current state of the art technologies and techniques. This may involve using advanced algorithms, specialized hardware, or other tools and technologies in order to achieve the best possible performance.

point cloud benchmark data sets

certifiable algorithm
	an algorithm that attempts to solve an intractable problem and provides checkable conditions on whether it succeeded (Yang, Shi, Carlone)

generative learning algorithm
	A generative learning algorithm is a type of machine learning algorithm that is trained to generate new data that is similar to the data it was trained on. These algorithms are typically used for tasks such as image generation, where the goal is to produce new, realistic-looking images based on a set of training examples. Generative learning algorithms typically use unsupervised learning techniques, which means that they learn to generate new data based on the patterns and structures they discover in the training data, without the need for explicit labels or targets. Some examples of generative learning algorithms include generative adversarial networks (GANs), variational autoencoders (VAEs), and autoregressive models such as the PixelRNN. (chatGPT, OpenAI)

Machine Learning Concepts

convolution integral 

A convolution integral is a mathematical operation that combines two functions to produce a third function that represents the way in which the first two functions interact. This operation is used in many areas of mathematics and science, including signal processing, image processing, and probability theory.
To understand convolution integrals, it's helpful to first understand the concept of convolution. Convolution is an operation that takes two functions and "blends" them together in a specific way. Intuitively, convolution is like mixing two substances together: the resulting mixture is a combination of the original substances, but it is also something new that has its own unique properties.
In mathematical terms, convolution is defined as the integral of the product of two functions, with one of the functions reversed and shifted. This may sound complicated, but it can be easier to understand with an example.
Suppose we have two functions, f(x) and g(x), and we want to compute their convolution. The convolution of these two functions is given by the integral:
(f * g)(x) = ∫f(t)g(x-t) dt
This integral may look daunting at first, but it's actually not so bad once we break it down. The variable t is called the "dummy variable" and it is used to "shift" the function g(x-t) as we move through the interval of integration.
For each value of t, we multiply f(t) and g(x-t), and then we add up all of these products to get the convolution of the two functions. This process can be thought of as sliding the function g(x) along the x-axis and computing the overlap with f(x) at each position.
Convolution is a powerful operation that has many applications in mathematics and science. It is used to model physical phenomena such as the flow of heat or electricity, and it is also used in signal processing to filter out noise or enhance signals. Convolution integrals are a key tool for understanding and working with these phenomena. (chatGPT)

convolutional neural network

A convolutional neural network (CNN) is a type of artificial neural network that is commonly used in image and video recognition. It is made up of multiple layers of interconnected neurons, each of which performs a specific function in processing the input data.
The key difference between a CNN and a traditional neural network is the way in which the layers are connected. In a traditional neural network, each neuron in one layer is connected to every neuron in the next layer. This makes it difficult to process inputs that have a grid-like structure, such as an image.
In a CNN, on the other hand, the neurons in each layer are only connected to a small local region of the previous layer. This allows the network to learn local spatial patterns in the input data, which is essential for image and video recognition.
Another key feature of CNNs is the use of pooling layers, which downsample the input data and reduce its dimensionality. This makes the network more efficient and allows it to learn more abstract features of the input data.
Overall, CNNs are a powerful tool for image and video recognition, and they have been used to achieve state-of-the-art performance on many tasks. They are widely used in a variety of applications, including object detection, facial recognition, and medical image analysis. (chatGPT)


Fully Convolutional Neural Network
A Fully Convolutional Neural Network (FCN) is a type of neural network architecture that is designed to process and classify images by using only convolutional layers. In contrast to traditional convolutional neural networks (CNNs), which use a combination of convolutional layers and fully connected layers, FCNs use only convolutional layers.
The architecture of an FCN typically consists of an encoder network followed by a decoder network. The encoder network is a series of convolutional layers that downsample the input image by successively reducing its spatial resolution and increasing its number of channels. This process is typically performed using pooling layers or strided convolutions.
The decoder network is a series of convolutional layers that upsample the feature maps produced by the encoder network to produce a segmentation map. This is typically done using transposed convolutional layers, which increase the spatial resolution of the feature maps while reducing their number of channels.
FCNs are particularly well-suited for image segmentation tasks, where the goal is to classify each pixel in an image into one of several classes. By using only convolutional layers, FCNs can process images of arbitrary size and produce segmentation maps that are the same size as the input image. This makes them a popular choice for tasks such as object detection, semantic segmentation, and image restoration. (chatGPT)

In a neural network, a fully connected layer and a convolutional layer are two different types of layers that serve different purposes.

A fully connected layer, also known as a dense layer, is a type of layer in which each neuron is connected to every neuron in the previous layer. This means that each neuron in a fully connected layer receives input from every neuron in the previous layer, and it produces an output that is connected to every neuron in the next layer. Fully connected layers are typically used in the final layers of a neural network to perform classification or regression tasks, where the input data has already been processed and reduced in dimensionality.

In contrast, a convolutional layer is a type of layer that performs convolution operations on the input data. In a convolutional layer, a set of filters is applied to the input data, producing a set of feature maps. Each filter is a small matrix of weights that is slid over the input data, producing a dot product at each location, which is then aggregated to produce a single output value. Convolutional layers are typically used to extract spatial features from the input data, and they are commonly used in tasks such as image classification, object detection, and segmentation.

The main difference between a fully connected layer and a convolutional layer is in the way they process data. Fully connected layers treat each input feature independently, whereas convolutional layers exploit the spatial relationships between neighboring features. This makes convolutional layers particularly effective at capturing spatial patterns in data, such as the edges and textures in an image. In addition, convolutional layers typically have far fewer parameters than fully connected layers, which makes them more computationally efficient and less prone to overfitting. (chatGPT)

We need to use a convolution kernals to process a point cloud with the NN

Convolutions kernels

In a convolutional neural network (CNN), a convolution kernel, also known as a filter or a weight matrix, is a small matrix of numbers that is used to perform a convolution operation on the input data. The convolution kernel is the key component of a convolutional layer in a CNN, and it is responsible for learning and extracting the features from the input data.

A convolution kernel is typically a square matrix of odd dimensions, such as 3x3, 5x5, or 7x7. During the convolution operation, the kernel is slid over the input data, performing a dot product at each location. The resulting output value is placed into the corresponding position of the output feature map. The values of the convolution kernel are learned during the training process of the CNN, through a process called backpropagation, in which the network adjusts the kernel values to minimize the loss function.

The choice of the size and number of convolution kernels is an important factor in the design of a CNN. A larger kernel size can capture more complex patterns in the input data, but it also requires more computation and can lead to overfitting. A smaller kernel size can capture simpler patterns, but it is faster and less prone to overfitting. The number of convolution kernels used in a layer determines the number of features that are extracted from the input data. Typically, the number of kernels increases as the spatial resolution of the feature maps decreases, resulting in a network that learns increasingly abstract features.

Convolution kernels have become a fundamental building block of modern computer vision systems, and they are used in a wide range of applications, such as image classification, object detection, and segmentation. (chatgpt)

Most point-based convolution networks borrow the common encoder/decoder idea (or encoder only). An encoder operates on a dense point cloud, which is iteratively decimated after each layer or group of layers as we go deeper. The points themselves support feature vectors, and going from one layer to the next usually entails two steps:
downsampling the point cloud;
    for each point in the downsampled point cloud, computing a feature vector based on the features of its neighbours in the previous point cloud.
In short, the deeper in the network, the fewer the points — but the richer their associated features.
(chatgpt)

Sparse Convolution

The convolution problem is similar to what is used in 2D image processing, but the third dimension Z causes the data to be sparse in general -> most voxels are empty
https://towardsdatascience.com/how-does-sparse-convolution-work-3257a0a8fd1


Idea - Use CNNs to replace segmentation in workpiece localization !?!?
 		
Sematantic Segmentation - determine what is in a image and where in the image it is located, label each pixel into a corresponding class
this is typically a dense prediction


Quantization - restrict the number of possible values of (a quantity) or states of (a system) so that certain variables can assume only certain discrete magnitudes - oxford languages

Inference -  a conclusion reached on the basis of evidence and reasoning

Using the model!

Machine learning inference is the process of running data points into a machine learning model to calculate an output such as a single numerical score. This process is also referred to as “operationalizing a machine learning model” or “putting a machine learning model into production.” - https://cloud.google.com/bigquery-ml/docs/reference/standard-sql/bigqueryml-syntax-inference-overview


Machine learning (ML) inference is the process of running live data points into a machine learning algorithm (or “ML model”) to calculate an output such as a single numerical score. This process is also referred to as “operationalizing an ML model” or “putting an ML model into production.” When an ML model is running in production, it is often then described as artificial intelligence (AI) since it is performing functions similar to human thinking and analysis. Machine learning inference basically entails deploying a software application into a production environment, as the ML model is typically just software code that implements a mathematical algorithm. That algorithm makes calculations based on the characteristics of the data, known as “features” in the ML vernacular. - https://hazelcast.com/glossary/machine-learning-inference/




## algorithms/networks



- teaser/teaser++
	- open source implementation available (MIT license)
	- 1.1k stars on Github
	- updated this year

- SmoothNet3D - used by Teaser ?

 - Predator (OverlapPredator) : https://github.com/prs-eth/OverlapPredator


- IMFNet: Interpretable Multimodal Fusion for Point Cloud Registration

- LMVD: End-to-End Learning Local Multi-view Descriptors for 3D Point Clouds

- 3DMatch + RANSAC: 3DMatch: Learning Local Geometric Descriptors from RGB-D Reconstructions

- FPFH + RANSAC: Fast Point Feature Histograms (FPFH) for 3D Registration

- DeepICP

- DeepPRO: Deep Partial Point Cloud Registration of Objects

- DeepMatch

- GenReg: Deep Generative Method for Fast Point Cloud Registration

- VoxelNet


- Torch Points3D - https://github.com/nicolas-chaulet/torch-points3d

	- this looks promising


- Minkowski Engine - Sparse Convolutional Networks for 3D data - examples and python library for networks

## Standard Datasets

for 2D images

- MNIST - handwritten letters

- FashionMNIST - picture of fashion and clothing items, 10 classes

- VGG-Nets - 1000 class of general and specific items 

for 3D images

- Princeton ModelNet
  - ModelNet40 standard 3D images for generalized image recognition and other tasks
  - ModelNet10 subset of 40 with labeled orientation for registration hopefully - !!! This seems promising.

- 3DMatch used by overlap predator
- KITTI used by overlap predator






Herrman Minkowski - generalized rotations




### lists of algoritms 

papers with code
https://paperswithcode.com/sota/point-cloud-registration-on-3dmatch-benchmark




## libraries:

- vinits5/learning3d - https://github.com/vinits5/learning3d

- chrischoy/FCGF

- Python torch + Nvidia CUDA 
 - used by OverlapPredator

- ROS pointcloud_registration - https://github.com/carlosmccosta/pointcloud_registration


## Docker base images

Ubuntu 

Python

CUDA

Pytorch - If we are using python torch this makes sense lol








## recent publications on "point cloud registration"

### review papers

Deep learning based point cloud registration: an overview
Zhiyuan Zhang, Yuchao Dai, Jiadai Sun
Science Direct - Virtual Reality and Intelligent Hardware, 2022

Review: Deep Learning on 3D Point Clouds
Saifullahi Aminu Bello, Shangshu Yu, Cheng Wang, Jibril Muhmmad Adam, Jonathan Li
Various Univ.
Remote Sensing, 2020

### registration algorithm papers


PREDATOR: Registration of 3D Point Clouds with Low Overlap (CVPR 2021, Oral)


DeepMatch: Toward Lightweight in Point Cloud Registration
Lizhe Qi, Fuwang Wu, Zuhao Ge, Yuquan Sun
Frontiers - Frontiers in Robotics, 2022

TEASER: Fast and Certifiable Point Cloud Registration
Heng Yang, Jingnan Shi, Luca Carlone
IEEE Transactions on Robotics, 2021

GenReg: Deep Generative Method for Fast Point Cloud Registration
Xiaoshui Huang, Zongyi Xu, Guofeng Mei, Sheng Li, Jian Zhang, Yifan Zuo, Yucheng Wang
Various Univ.
Cornell Univ - Arxiv, 2021 

ReAgent: Point Cloud Registration Using Imitation and Reinforcement Learning 
Dominik Bauer, Timothy Patten, Markus Vincze
Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), 2021

CorsNet: 3D Point Cloud Registration by Deep Neural Network
Akiyoshi Kurobe; Yusuke Sekikawa; Kohta Ishikawa; Hideo Saito
IEEE Robotics and Automation Letters ( Volume: 5, Issue: 3, July 2020) 

PCRNet: Point Cloud Registration Network using PointNet Encoding
Vinit Sarode, Xueqian Li, Hunter Goforth, Yasuhiro Aoki, Rangaprasad Arun Srivatsan, Simon Lucey, Howie Choset
2019

Deep Closest Point: Learning Representations for Point Cloud Registration
Yue Wang, Justin M. Solomon, MIT
Proceedings of the IEEE/CVF International Conference on Computer Vision (ICCV), 2019

DeepVCP: An End-to-End Deep Neural Network for Point Cloud Registration
Weixin Lu, Guowei Wan, Yao Zhou, Xiangyu Fu, Pengfei Yuan, Shiyu Song
Baidu Autonomous Driving Technology Department
Computer Vision Foundation - IEEE Xplore, 2019

PointNetLK: Robust & Efficient Point Cloud Registration Using PointNet
Yasuhiro Aoki, Hunter Goforth, Rangaprasad Arun Srivatsan, Simon Lucey
Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR), 2019

3D Point Cloud Registration for Localization Using a Deep Neural Network Auto-Encoder
Gil Elbaz, Tamar Avraham, Anath Fischer
Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2017

NCHW stands for:
batch N, channels C, depth D, height H, width W
It is a way to store multidimensional arrays / data frames / matrix into memory - https://stackoverflow.com/questions/67087131/what-is-nchw-format



### related papers

#### point cloud image detection 

VoxelNet: End-to-End Learning for Point Cloud Based 3D Object Detection
Yin Zhou, Oncel Tuzel
Apple Inc.
Arxiv, Computer Science, Computer Vision and Pattern Recognition, 2018


### application papers







### things to do:

- [x] test additional C++ example from TEASER: `teaser_cpp_fpfh.cpp`

  - [ ] further investigate FPFH, visualize features?   https://github.com/chrischoy/FCGF


- [ ] test Python examples from TEASER:

- [ ] test 3DSmoothNet for preparing point cloud data

  - [ ] setup environment and install ... based on ubuntu16... lets look elsewhere....

- [ ] test Minkowski Engine - this seems like a good re-starting point, and it appears to be current
  - [x] use dockerfile available in reposistory, example version check worked fine -> 0.5.4 
  - [ ] test install in thillrobot/machine_learning/(pytorch)



- [x] test Predator example codeL: https://github.com/prs-eth/OverlapPredator
  - see forked code at github.com/thillrobot/OverlapPredator
  - [x] example code is working in docker :)
  - [ ] load data set from seamdetection project, demo code uses .pth (python torch asset) and I do not know how to convert/save point cloud to .pth
  - [ ] generate custom dataset to use with Predator, this should handle the convolution encoding (see _convolution kernel_)

- [ ] investigate Predator fork from Zan Gojcic: https://github.com/zgojcic/OverlapPredator

- [ ] learn about sparse convolution with NVIDIA MinkowskiEngine: https://github.com/NVIDIA/MinkowskiEngine
 - [x] build image from Dockerfile from NVIDIA
 - [x] build image from custom Dockerfile

- [ ] test torch points3d examples
 - [ ] setup environment in docker
 - [ ] prepare data test data set with sparse convolution
 - [ ] test custom point clouds with regstration from torch-points3d

- [ ] read/learn/review convolution (dense) for 2d images
- [ ] read/learn about sparse convolution for 2D images
- [ ] read/learn about sparse convolution for 3D images
https://towardsdatascience.com/how-does-sparse-convolution-work-3257a0a8fd1

- [ ] test/read PyTorch (Python torch) basic tutorials: https://pytorch.org/tutorials/beginner/basics/intro.html
 - [x] tensors
 - [x] datasets
 - [x] transforms
 - [x] build model - need to read this one again
 - [ ] auto differentiation
 - [ ] optimization loop
 - [ ] save, load and use model

- [ ] create a 'test' dataset from seam_detection idetc 2022 lidar images for NN algorithm XYZ

- [ ] clean up this notes docs, organize short term goals 

- [ ] work on literature review