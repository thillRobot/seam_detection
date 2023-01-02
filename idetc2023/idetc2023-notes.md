# IDETC 2023 - Notes

## concepts and brainstorming

point cloud registration 


image registration - seam detection 


robot arms and cobots


workspace sensing and workpeice detection


optimization based algorithms


overlapping point clouds

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

We need to use a convolution kernals to process a point cloud with the NN

Convolutions kernels

Most point-based convolution networks borrow the common encoder/decoder idea (or encoder only). An encoder operates on a dense point cloud, which is iteratively decimated after each layer or group of layers as we go deeper. The points themselves support feature vectors, and going from one layer to the next usually entails two steps:

    downsampling the point cloud;
    for each point in the downsampled point cloud, computing a feature vector based on the features of its neighbours in the previous point cloud.

In short, the deeper in the network, the fewer the points — but the richer their associated features.


Sparse Convolution

The convolution problem is similar to what is used in 2D image processing, but the third dimension Z causes the data to be sparse in general -> most voxels are empty
https://towardsdatascience.com/how-does-sparse-convolution-work-3257a0a8fd1







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



### lists of algoritms 

papers with code
https://paperswithcode.com/sota/point-cloud-registration-on-3dmatch-benchmark




## libraries:

- vinits5/learning3d - https://github.com/vinits5/learning3d

- chrischoy/FCGF

- Python torch + Nvidia CUDA 
 - used by OverlapPredator

- ROS pointcloud_registration - https://github.com/carlosmccosta/pointcloud_registration


## Docker Images

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

- [ ] test Python examples from TEASER:

- [ ] read publication - PREDATOR: Registration of 3D Point Clouds with Low Overlap, https://arxiv.org/pdf/2011.13005.pdf

- [x] test Predator example codeL: https://github.com/prs-eth/OverlapPredator
  - see forked code at github.com/thillrobot/OverlapPredator
  - [x] example code is working in docker :)
  - [ ] load data set from seamdetection project, demo code uses .pth (python torch asset) and I do not know how to convert/save point cloud to .pth
  - [ ] generate custom dataset to use with Predator, this should handle the convolution encoding (see _convolution kernel_)

- [ ] investigate Predator fork from Zan Gojcic: https://github.com/zgojcic/OverlapPredator

- [ ] learn about sparse convolution with NVIDIA MinkowskiEngine: https://github.com/NVIDIA/MinkowskiEngine
 - [ ] build image from Dockerfile from NVIDIA
 - [x] build image from custom Dockerfile

- [ ] test torch points3d examples
 - [ ] setup environment in docker
 - [ ] test custom point clouds with regstration from torch-points3d

- [ ] read/learn/review convolution (dense) for 2d images
- [ ] read/learn about sparse convolution for 2D images
- [ ] read/learn about sparse convolution for 3D images
https://towardsdatascience.com/how-does-sparse-convolution-work-3257a0a8fd1



