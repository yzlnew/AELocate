# Geiger算法C++实现

## 总体概况

* `C++`实现了`Geiger`算法
* `Visual Studio 2010` 编译通过
* 使用[Eigen](http://google.com)开源矩阵库来进行矩阵计算

## 文件说明

* `Geiger.h`包含`geigerSolver`类的定义
* `Geiger.cpp`实现了`geigerSolver`
* `main.cpp`测试了一组来自实验的数据，输出为定位点坐标值

## 类定义

```cpp
    class geigerSolver{
    private:
        Matrix<double, Dynamic, 3> sensorLoc;    //传感器坐标
        VectorXd arrivalTime;                    //到达时间
        // Vector3d spaceLimit;                  //试件的空间尺寸
        int sensorNumber;
        float sonicSpeed;

    public:
        geigerSolver(double *LocOfSensor,double *TimeOfArrival,float *LimitOfSpace,int NumOfSensors,float SpeedOfSound);  //构造函数
        double* doSolve();                                   //求解函数
        double pdist(RowVector3d, RowVector3d);              //求向量距离
        double LocRes[3] = { 0,0,0 };                        //定位结果
        bool isAccurate = true;               //是否准确的标志位，默认为真
        void resRevised();                    //当结果有偏离时，修正函数
        bool isInBox();                       //判断是否在体内
        double targetFunc(RowVector3d);       //评价函数
    };
```

## 构造函数参数说明

* `double *LocOfSensor`：按到达时间为序的传感器的坐标值构成的一维数组指针
* `double *TimeOfArrival`：到达时间构成的一维数组指针
* `float *LimitOfSpace`：试件的空间尺寸构成的一维数组指针
* `int NumOfSensors`：本次计算的使用的传感器数量
* `float SpeedOfSound`：预设声速值

## CMake

设置环境变量`EIGEN3_INCLUDE_DIR`为`Eigen`的`include`文件夹。
