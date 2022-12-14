# IDETC 2023 - Notes

## concepts and brainstorming

point cloud registration 


image registration - seam detection 


robot arms and cobots


workspace sensing and workpeice detection


optimization based algorithms


correspondances-based vs correspondances-free registration

deep neural network - multi layered neural network 

auto-encoders - an artificial neural network used to learn efficient codings of unlabeled data (unsupervised learning) - wikipedia

region proposal network 

end to end learning algorithms

SOTA - State of The Art efficiency

point cloud benchmark data sets

certifiable algorithm - an algorithm that attempts to solve an intractable problem and provides checkable conditions on whether it succeeded (Yang, Shi, Carlone)





## algorithms/networks



- teaser/teaser++
	- open source implementation available (MIT license)
	- 1.1k stars on Github
	- updated this year

- SmoothNet3D - used by Teaser ?



- IMFNet: Interpretable Multimodal Fusion for Point Cloud Registration

- LMVD: End-to-End Learning Local Multi-view Descriptors for 3D Point Clouds

- 3DMatch + RANSAC: 3DMatch: Learning Local Geometric Descriptors from RGB-D Reconstructions

- FPFH + RANSAC: Fast Point Feature Histograms (FPFH) for 3D Registration

- DeepICP

- DeepPRO: Deep Partial Point Cloud Registration of Objects

- DeepMatch

- GenReg: Deep Generative Method for Fast Point Cloud Registration



- VoxelNet





### lists of algoritms 

papers with code
https://paperswithcode.com/sota/point-cloud-registration-on-3dmatch-benchmark




## libraries:

- vinits5/learning3d - https://github.com/vinits5/learning3d

- chrischoy/FCGF







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



### related papers

#### point cloud image detection 

VoxelNet: End-to-End Learning for Point Cloud Based 3D Object Detection
Yin Zhou, Oncel Tuzel
Apple Inc.
Arxiv, Computer Science, Computer Vision and Pattern Recognition, 2018




### application papers