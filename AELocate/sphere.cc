#include "sphere.h"
#include <math.h>

/**
 * 构造函数
 * @param  LocOfSensor   传感器球面坐标一维数组,以天顶角、方位角的顺序排列
 * @param  TimeOfArrival 到达时间数组
 * @param  Radius        球面半径
 * @param  NumOfSensors  传感器数量
 * @param  SpeedOfSound  声速
 */

sphereSolver::sphereSolver(double *LocOfSensor, double *TimeOfArrival, float Radius, int NumOfSensors, float SpeedOfSound) {
	sensorNumber = NumOfSensors;
	sonicSpeed = SpeedOfSound;
	radius = Radius;
	// arrivalTime = VectorXd::Zero(sensorNumber);
	// sensorLoc = MatrixX2d::Zero(sensorNumber, 2);
	arrivalTime = Map<VectorXd>(TimeOfArrival, sensorNumber);
	//arrivalTime = TimeOfArrival;
	sensorLoc = Map<Matrix2Xd>(LocOfSensor, 2, sensorNumber).transpose();
//	for (int i = 0; i < NumOfSensors; i++) {
//		sensorLoc(i, 0) = *(LocOfSensor + 2 * i + 0);
//		sensorLoc(i, 1) = *(LocOfSensor + 2 * i + 1);
//	}
}


/**
 * 球面定位求解函数
 * @return 定位结果的三维直角坐标
 */

double* sphereSolver::doSolve(){

	MatrixX3d A = MatrixX3d::Zero(sensorNumber, 3);
	/**
	* 将球面坐标转化为三维直角坐标
	*/
	for (int i = 0; i<sensorNumber; i++) {
		A(i, 0) = radius * sin(sensorLoc(i, 0)) * cos(sensorLoc(i, 1));
		A(i, 1) = radius * sin(sensorLoc(i, 0)) * sin(sensorLoc(i, 1));
		A(i, 2) = radius * cos(sensorLoc(i, 0));
	}

	VectorXd b;
	b = VectorXd::Ones(sensorNumber);
	for(int i = 0;i<sensorNumber;i++){
		b(i) = pow(radius,2) * ( 1 - 2 * pow(sin( arrivalTime(i) *sonicSpeed/(2*radius)),2) );
	}

	Vector3d resVec =  A.colPivHouseholderQr().solve(b);
	resVec = radius * resVec /(sqrt(resVec.array().pow(2).sum()));

	LocRes[0] = resVec(0);
	LocRes[1] = resVec(1);
	LocRes[2] = resVec(2);

	return LocRes;
}
