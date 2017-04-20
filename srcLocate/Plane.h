#pragma once

#include <Eigen>
#include <vector>

using namespace Eigen;
using namespace std;

class planeSolver {
private:
	MatrixX2d sensorLoc;					//平面传感器坐标
	VectorXd arrivalTime;					//到达时间
	Vector2d spaceLimit;					//平面尺寸
	int sensorNumber;
	float sonicSpeed;

public:
	planeSolver(double *LocOfSensor, double *TimeOfArrival, float *LimitOfSpace, int NumOfSensors, 
					float SpeedOfSound, double Height);	//构造函数
	double* doSolve();						//求解函数
	double pdist(const RowVector2d& r1, const RowVector2d& r2);	//求向量距离
	double LocRes[2] = { 0,0 };			    //定位结果
	bool isAccurate = true;					//是否准确的标志位，默认为真
	bool isInBox();							//判断是否在平面内
	void resRevised();						//当结果有偏离时，修正函数
};