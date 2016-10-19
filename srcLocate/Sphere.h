#pragma once

#include <Eigen>
#include <vector>

using namespace Eigen;
using namespace std;

class sphereSolver{
private:
	MatrixX2d sensorLoc;					//传感器的球面坐标
	MatrixX3d sensorLocXYZ;					//传感器的三维直角坐标 
	VectorXd arrivalTime;					//到达时间
	float radius;							//球面的半径
	int sensorNumber;
	float sonicSpeed;

public:
	sphereSolver(double *LocOfSensor,double *TimeOfArrival,
						float Radius,int NumOfSensors,float SpeedOfSound);	//构造函数
	double* doSolve();						//求解函数
	double dist(RowVector2d, RowVector2d);	//求球面距离
	double LocRes[3] = { 0,0,0 };			//定位结果
	bool isAccurate = true;					//是否准确的标志位，默认为真
	void resRevised();						//当结果有偏离时，修正函数
	bool isInBox();							//判断是否在体内
};