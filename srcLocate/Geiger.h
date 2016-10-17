#pragma once


#include <Eigen>
#include <vector>

using namespace Eigen;
using namespace std;

class geigerSolver{
private:
	Matrix<double, Dynamic, 3> sensorLoc;	//传感器坐标
	VectorXd arrivalTime;					//到达时间
	Vector3d spaceLimit;					//试件的空间尺寸
	int sensorNumber;
	float sonicSpeed;

public:
	geigerSolver(double *LocOfSensor,double *TimeOfArrival,float *LimitOfSpace,int NumOfSensors,float SpeedOfSound);	//构造函数
	double* doSolve();						//求解函数
	double pdist(RowVector3d, RowVector3d);	//求向量距离
	double LocRes[3] = { 0,0,0 };			//定位结果
	bool isAccurate = true;					//是否准确的标志位，默认为真
	void resRevised();						//当结果有偏离时，修正函数
	bool isInBox();							//判断是否在体内
	double targetFunc(RowVector3d);			//评价函数
};