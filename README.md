#  SO-SLAM


<p align="center">
  <img src="https://github.com/MRHan-426/SOSLAM/blob/master/.assets/3%2000_00_00-00_00_30.gif" alt="gif">
</p>
     
![License](https://img.shields.io/badge/license-MIT-green)
![Primary language](https://img.shields.io/github/languages/top/MRHan-426/Racing_on_a_Pre-Defined_Map_with_Unknown_Obstacles )
![ROB535](https://img.shields.io/badge/ROB535-Team2-orange)

This is Team 6's final project git repository for ROB530: Mobile Robotics. 

The title of our project is **Implementation and Evaluation of Semantic-Object SLAM Algorithm**

The team members include: Ziqi Han, Zhewei Ye, Tien-Li Lin, Yi-Cheng Liu, Shubh Agrawal.

**Related Paper:**  [RA-L&ICRA 2022]

+ Liao Z, Hu Y, Zhang J, et al. So-slam: Semantic object slam with scale proportional and symmetrical texture constraints[J]. IEEE Robotics and Automation Letters, 2022, 7(2): 4008-4015. [**[PDF]**](https://arxiv.org/abs/2109.04884)

---

## 📚 1. Prerequisites


```shell
sudo apt-get install libglew-dev
sudo apt-get install libeigen3-dev
sudo apt-get install libtbb-dev
sudo apt-get install libmetis-dev
sudo apt-get install libpugixml-dev
sudo apt-get install libpcl-dev
```


```shell
cmake 3.26.0
libboost 1.71.0  # make sure to compile C++ version from source code.
Pangolin 0.8.0
OpenCV 4.7.0
```



## ⚙️ 2. Compile GTSAM

**Note that higher version may bring unexpected errors, we do not test other version so far.**

```shell
git clone --branch 4.1.1 https://github.com/borglab/gtsam.git
```

Modify Eigen cmake config file: cmake/HandleEigen.cmake

```shell
set(GTSAM_USE_SYSTEM_EIGEN ON)
```

Then:

```shell
mkdir build
cd build
cmake ..
make check
sudo make install
```




## 🛠️ 3. Compile our repo

Branch Master contains point cloud visualization, so you have some more prerequisites.

```shell
git clone --branch master https://github.com/MRHan-426/SOSLAM.git
```

Branch 0.0.1 doesnot contain point cloud visualization, so you don't have to compile PCL, VTK.

```shell
git clone --branch 0.0.1 https://github.com/MRHan-426/SOSLAM.git
```

Then:

```shell
mkdir build
cd build
cmake ..
make
```




## 🌟 4. Examples

First, prepare dataset and rename as **input** directory. We provide three hand-labeled dataset below.

It cost us a lot of time to label and associate these datasets, so please star if we do help you.

There are detailed configurations in **config.yaml**, please change if you need.


+ **Dummy Example**: We provide a demo to visualize for debugging. It will shows two hand designed ellipsoid.

```shell
./soslam_exe --dummy --3d
```




+ **Fr2 Desk:** We provide a demo running dataset TUM RGBD. [**Download hand labeled dataset**](https://drive.google.com/file/d/1V3Ow0UOeU0FA58rtzbE325CzNy2-PVOL/view?usp=sharing)

```shell
./soslam_exe --Fr2_Desk --3d
```
<p align="center">
  <img src="https://github.com/MRHan-426/SOSLAM/blob/master/.assets/3.png" alt="image" width="66%" height="auto" />
</p>




+ **Fr1 Desk2:** We provide a demo running dataset TUM RGBD. [**Download hand labeled dataset**](https://drive.google.com/file/d/19szLlFB4Yxnx0SC5PewdLyk07I1OXINP/view?usp=sharing)

```shell
./soslam_exe --Fr1_Desk2 --3d
```

<p align="center">
  <img src="https://github.com/MRHan-426/SOSLAM/blob/master/.assets/4.png" alt="image" width="66%" height="auto" />
</p>




+ **Fr2 Dishes:** We provide a demo running dataset TUM RGBD. [**Download hand labeled dataset**](https://drive.google.com/file/d/1ASIADGoLiYin7QoxdcVXOR1_HpVC9U2t/view?usp=sharing)

```shell
./soslam_exe --Fr2_Dishes --3d
```


<p align="center">
  <img src="https://github.com/MRHan-426/SOSLAM/blob/master/.assets/2.png" alt="image" width="66%" height="auto" />
</p>




## 🎬 5. Videos and Documentation

+ Our project presentation video is on [**[YouTube]**](https://youtu.be/_yUy5nOtfMM).




+ Project Document: [**[PDF]**](https://github.com/MRHan-426/SOSLAM/blob/master/.assets/rob530_group6_final_report.pdf)




## 📝 6. Note

+ If you want to use it in your work or with other datasets, you should prepare the dataset containing:

  - RGB image
  - Label xml (contain "objectKey" key to store the data association information)
  - Odom txt
  - Depth image (if you do not need point cloud visualization, just ignore)
  - Camera intrinsic txt

  Be aware that you should rename your images and xmls as number 1,2,3,...

  Be aware that RGB, Depth, Label, Odom must match.

+ This is an incomplete version of our project. 
    - We have a lot of experiments to be done.
    - We have not achieved real-time.




## 🏅 7. Acknowledgement

Thanks for the great work: 

+ [**SO-SLAM**](https://github.com/XunshanMan/SoSLAM)
+ [**GTSAM**](https://github.com/borglab/gtsam)
+ [**Quadric-SLAM**](https://github.com/qcr/quadricslam)
+ [**EAO-SLAM**](https://github.com/yanmin-wu/EAO-SLAM) 
+ [**ORB-SLAM2**](https://github.com/raulmur/ORB_SLAM2)
+ [**YOLO-v8**](https://github.com/ultralytics/ultralytics)



## 📫 8. Contact

+ Ziqi Han, Email: ziqihan@umich.edu
+ Zhewei Ye, Email: yezhewei@umich.edu
+ Tien-Li Lin, Email: tienli@umich.edu
+ Yi-Cheng Liu, Email: liuyiche@umich.edu
+ Shubh Agrawal, Email: shbhgrwl@umich.edu 




**Please cite the author's paper if you use the code in your work.**

```
@article{liao2022so,
  title={So-slam: Semantic object slam with scale proportional and symmetrical texture constraints},
  author={Liao, Ziwei and Hu, Yutong and Zhang, Jiadong and Qi, Xianyu and Zhang, Xiaoyu and Wang, Wei},
  journal={IEEE Robotics and Automation Letters},
  volume={7},
  number={2},
  pages={4008--4015},
  year={2022},
  publisher={IEEE}
}
```




## 4. Project Log & Todo List
    
- [x] 1. Finish writing PID control for part1.

- [x] 2. Use PID control for part2.

    - [x] 2.1 Debug.
    - [x] 2.2 Test for success rate and time => This method is abandoned.

- [x] 3. Use fmincon for part2.

    - [x] 3.1 Finish writing constraints.
    - [x] 3.2 Debug => This method is abandoned.

- [x] 4. Use MPC for part2.

    - [x] 4.1 Generate reference trajectory for MPC.
    - [x] 4.2 Debug.
    - [x] 4.3 solve QP no solution problem.
    - [x] 4.4 Test for success rate and time.

- [x] 5. Upload and Report.

    - [x] 5.1 Draw trajectory.
    - [x] 5.2 Write Report and Upload code.

- [ ] 6. Use MPPI for part2.

    - [ ] 6.1 Write Cost function.
    - [ ] 6.2 Write MPPI loop.
    - [ ] 6.3 Test for success rate and time.

