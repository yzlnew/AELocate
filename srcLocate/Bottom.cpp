#include "Bottom.h"
#include <math.h>

bottomSolver::bottomSolver(double *LocOfSensor, double *TimeOfArrival, 
								int NumOfSensors, float SpeedOfSound, double Height, double Radius) {
	sensorNumber = NumOfSensors;
	sonicSpeed = SpeedOfSound;
	//arrivalTime = VectorXd::Zero(sensorNumber);
	//sensorLoc = MatrixX3d::Zero(sensorNumber, 3);
	arrivalTime = Map<VectorXd>(TimeOfArrival, sensorNumber);
	sensorLoc = Map<Matrix2Xd>(LocOfSensor, 2, sensorNumber).transpose();
	height = Height;
	radius = Radius;
	sensorLoc = (height + radius)*sensorLoc / radius;							//把传感器坐标转换为平面
}

double* bottomSolver::doSolve() {
	RowVector2d iterationPoint = sensorLoc.colwise().sum() / sensorNumber;			//传感器的中心点设为初始迭代点
	RowVector2d temp;
	int iterationStep = 3;														//设置迭代步数，推荐2以上
	VectorXd allDist;															//迭代点到各个传感器的距离向量
	allDist.resize(this->sensorNumber);
	allDist.fill(0);
	Matrix<double, Dynamic, 3> A;
	A = Matrix<double, Dynamic, 3>::Ones(this->sensorNumber, 3);
	VectorXd B;
	B = VectorXd::Ones(this->sensorNumber);
	VectorXd delta;
	delta = VectorXd::Ones(this->sensorNumber);

	for (int i = 0; i < sensorNumber; i++) {
		allDist(i) = pdist(iterationPoint, sensorLoc.row(i));
	}
	while (iterationStep > 0) {
		for (int i = 0; i < sensorNumber; i++) {
			RowVector2d temp_i = (iterationPoint - sensorLoc.row(i)) / (sonicSpeed*allDist(i));
			A(i, 0) = temp_i(0);
			A(i, 1) = temp_i(1);
			A(i, 2) = 1;
		}
		B = (arrivalTime - allDist / sonicSpeed).adjoint();
		//delta = (A.transpose() *A).ldlt().solve(A.transpose() *B);			//常规计算最小二乘
		delta = A.colPivHouseholderQr().solve(B);								//QR分解计算最小二乘
																				//delta = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);			//暂时不适用
		temp = delta.head(2);
		iterationPoint = iterationPoint + temp;
		for (int i = 0; i < sensorNumber; i++) {
			allDist(i) = pdist(iterationPoint, sensorLoc.row(i));
		}
		iterationStep = iterationStep - 1;
	}


	LocRes[0] = iterationPoint(0);
	LocRes[1] = iterationPoint(1);

	if (!this->isInBox()) {									//不在体内需要修正
		this->resRevised();
	}

	return LocRes;
}

double bottomSolver::pdist(const RowVector2d& r1, const RowVector2d& r2) {
	return sqrt(((r1 - r2).cwiseProduct(r1 - r2)).sum());
}

bool bottomSolver::isInBox()
{
	double x = LocRes[0];
	double y = LocRes[1];
	if ( x*x+y*y<radius*radius )
	{
		return true;
	}
	return false;
}

void bottomSolver::resRevised()
{
	for (int i = 0; i < 2; i++) {
		if (LocRes[i] < 0) LocRes[i] = 0;
		if (LocRes[i] > this->spaceLimit(i)) LocRes[i] = spaceLimit(i);
	}
}
