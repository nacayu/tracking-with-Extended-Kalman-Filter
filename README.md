# **Object Tracking with Extended Kalman Filter**

### 功能
基于UKF完成毫米波雷达和激光雷达**后融合**的**单目标**跟踪实战

### 假设
- 目标：CV模型（Constant Velocity）恒定速度模型，不考虑加速度
- 激光雷达测量值：x, y
- 毫米波雷达测量值：角度，径向距离，径向速度
- 已经得到object，不需要在进行聚类(DBSCAN, KMeans)
- 单目标跟踪：不需要匹配算法(匈牙利算法)

### **Demo: 使用激光雷达和雷达测量的物体跟踪**

[![gif_demo1][both_gif]](https://www.youtube.com/watch?v=XswKMtQBTCo)

在这个演示中，蓝色汽车是被跟踪的对象，但被跟踪的对象可以是任何类型，例如行人、车辆或其他移动物体。我们连续得到了激光雷达（**红圈**）和雷达（**蓝圈**）的测量结果。测量汽车在定义的坐标中的位置，但数据中可能存在噪声和误差。我们需要找到一种方法来融合这两种类型的传感器测量数据，以估计被跟踪物体的正确位置。

在实际场景，自动驾驶汽车同时接受激光雷达和雷达传感器对物体的测量，然后将这些测量结果应用于扩展卡尔曼滤波，根据这两种类型的传感器数据来跟踪物体。
 

---


## Code & Files
### 1. Dependencies & environment

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [Eigen library](src/Eigen)


### 2. My project files

* [CMakeLists.txt](CMakeLists.txt) is the cmake file.

* [data](data) folder contains test lidar and radar measurements.

* [Docs](Docs) folder contains docments which describe the data.

* [src](src) folder contains the source code.

* [include](include) folder contains head file

* [output](output) folder contains your results

* [visualization script](output/visualization.ipynb) visualize our project


### 3. Code Style

* [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).


### 4. How to run the code

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 
   * On windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it by either of the following commands: 
   * `cd /path/to/build`
   * `./ExtendedKF  ../data/obj_pose-laser-radar-synthetic-input.txt ../output/output.txt`
   * `./ExtendedKF  ../data/sample-laser-radar-measurement-data-1.txt ../output/output.txt`
5. Visualize your output results
   * see in [visualization](output/visualization.ipynb) and run the jupyter kernal to visualize your results (figures mainly show the relationship of radar, lidar, all sensors measurement and corresponding ground truth)
## System details

### 1. How does LIDAR measurement look like

![][image4]

激光雷达将产生三维测量值px,py,pz。但对于在道路上行驶的情况，我们可以将跟踪对象的姿势简化为追踪对象的姿势简化为：px,py,和一个角度。换句话说，我们可以只用px和px来表示物体的位置，以及角度来表示物体的方向。



### 2. How does RADAR measurement look like

![][image5]




### 3. How does the Extended Kalman Filter Work


![][image2]


### 4. Extended Kalman Filter V.S. Kalman Filter


![][image3]


* _x_ is the mean state vector.
* _F_ is the state transition function.
* _P_ is the state covariance matrix, indicating the uncertainty of the object's state.
* _u_ is the process noise, which is a Gaussian with zero mean and covariance as Q.
* _Q_ is the covariance matrix of the process noise.

**For EKF**
* To calculate predicted state vector _x′_, the prediction function _f(x)_, is used instead of the _F_ matrix.
* The _F_ matrix will be replaced by _Fj_ (jocobian matrix of _f_) when calculating _P′_.

---------------------------------------------------------

* _y_ is the innovation term, i.e. the difference between the measurement and the prediction. In order to compute the innovation term, we transform the state to measurement space by measurement function, so that we can compare the measurement and prediction directly.
* _S_ is the predicted measurement covariance matrix, or named innovation covariance matrix.
* _H_ is the measurement function.
* _z_ is the measurement.
* _R_ is the covariance matrix of the measurement noise.
* _I_ is the identity matrix.
* _K_ is the Kalman filter gain.
* _Hj_ and _Fj_ are the jacobian matrix.

**For EKF**
* To calculate innovation _y_, the measurement function _h(x')_ is used instead of the _H_ matrix.
* The _H_ matrix in the Kalman filter will be replaced by the _Hj_(Jacobian matrix of _h(x')_)when calculating _S_, _K_, and _P_.


**All Kalman filters have the same three steps:**

1. Initialization
2. Prediction
3. Update

一个**标准的卡尔曼滤波器**只能处理线性方程。**扩展卡尔曼滤波器**（EKF）和**无损卡尔曼滤波器**都允许你使用非线性方程；EKF和UKF的区别在于它们如何处理非线性方程： 扩展卡尔曼滤波器使用雅可比矩阵来线性化非线性函数；而无损卡尔曼滤波器则不需要线性化非线性函数，相反，无损卡尔曼滤波器从高斯分布中获取代表点。




[//]: # (Image References)
[image1]: ./data/ekf_combine.png
[image2]: ./data/ekf_flow.jpg
[image3]: ./data/ekf_vs_kf.jpg
[image4]: ./data/lidar.jpg
[image5]: ./data/radar.jpg
[image6]: ./data/camera-vs-radar-vs-lidar_1.png
[radar_gif]: ./data/radar.gif
[lidar_gif]: ./data/lidar.gif
[both_gif]: ./data/both_lidar_radar.gif
