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
	double LocRes[3] = { 0,0,0 };			//定位结果
};