#include "Geiger.h"
#include <math.h>

geigerSolver::geigerSolver(double *LocOfSensor,double *TimeOfArrival,float *LimitOfSpace,int NumOfSensors,float SpeedOfSound){
	sensorNumber=NumOfSensors;
	sonicSpeed = SpeedOfSound;
	spaceLimit << *LimitOfSpace, *(LimitOfSpace + 1), *(LimitOfSpace + 2);
	//arrivalTime = VectorXd::Zero(sensorNumber);
	//sensorLoc = MatrixX3d::Zero(sensorNumber, 3);
	arrivalTime=Map<VectorXd>(TimeOfArrival, sensorNumber);
	sensorLoc = Map<Matrix3Xd>(LocOfSensor, 3,sensorNumber).transpose();
	//for(int i=0;i<NumOfSensors;i++){
		//arrivalTime(i) = *(TimeOfArrival + i);
		//sensorLoc(i,0) = *(LocOfSensor + 3 * i + 0);
		//sensorLoc(i,1) = *(LocOfSensor + 3 * i + 1); 
		//sensorLoc(i,2) = *(LocOfSensor + 3 * i + 2);
	//}
}

double* geigerSolver::doSolve() {
	RowVector3d iterationPoint=sensorLoc.colwise().sum()/sensorNumber;			//传感器的中心点设为初始迭代点
	RowVector3d temp;
	int iterationStep = 3;														//设置迭代步数，推荐2以上
	VectorXd allDist;															//迭代点到各个传感器的距离向量
	allDist.resize(this->sensorNumber);
	allDist.fill(0);
	Matrix<double, Dynamic, 4> A;
	A = Matrix<double, Dynamic, 4>::Ones(this->sensorNumber, 4);
	VectorXd B;
	B = VectorXd::Ones(this->sensorNumber);
	VectorXd delta;
	delta = VectorXd::Ones(this->sensorNumber);

	for (int i = 0; i < sensorNumber; i++) {
		allDist(i) = pdist(iterationPoint, sensorLoc.row(i));
	}
	while (iterationStep > 0) {
		for (int i = 0; i < sensorNumber; i++) {
			RowVector3d temp_i = (iterationPoint - sensorLoc.row(i)) / (sonicSpeed*allDist(i));
			A(i, 0) = temp_i(0);
			A(i, 1) = temp_i(1);
			A(i, 2) = temp_i(2);
			A(i, 3) = 1;
		}
		B = (arrivalTime - allDist / sonicSpeed).adjoint();
		//delta = (A.transpose() *A).ldlt().solve(A.transpose() *B);			//常规计算最小二乘
		delta = A.colPivHouseholderQr().solve(B);								//QR分解计算最小二乘
		//delta = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(B);			//暂时不适用
		temp = delta.head(3);
		iterationPoint = iterationPoint + temp;
		for (int i = 0; i < sensorNumber; i++) {
			allDist(i) = pdist(iterationPoint, sensorLoc.row(i));
		}
		iterationStep = iterationStep - 1;
	}
	double x_Limit = this->spaceLimit(0);
	double y_Limit = this->spaceLimit(1);
	double z_Limit = this->spaceLimit(2);

	LocRes[0] = iterationPoint(0);
	LocRes[1] = iterationPoint(1);
	LocRes[2] = iterationPoint(2);
	
	if ( allDist.maxCoeff() > 1.5*sqrt( pow(x_Limit,2.0) + pow(y_Limit,2.0) + pow(z_Limit,2.0)) )
	{
		this->isAccurate = false;
		return LocRes;
	}
	if (!this->isInBox()) {									//不在体内需要修正
		this->resRevised();
	}

	return LocRes;
}

double* geigerSolver::doSolvePlane(){
	RowVector3d iterationPoint=spaceLimit.transpose()/2; //初始迭代点设置为物件中心
	double a = 0.95;
	double b = 0.95;
	int step_length = 5;
	double scale = iterationPoint.maxCoeff();
	int step = 0;
	double T,TI,TE;
	TI = 10; TE = 0.001;

	T = TI;
	double Pre, Cur,Delta,potential;
	while (T>TE)
	{
		while (step<step_length)
		{
			Pre = targetFunc(iterationPoint);
			RowVector3d newPoint = iterationPoint + scale*RowVector3d::Random();
			Cur = targetFunc(newPoint);
			Delta = Cur - Pre;

			if (Delta<0)
			{
				potential = 1;
			}
			else
			{
				potential = exp(-Delta / T);
			}
			double r = rand() / (float)RAND_MAX;
			if (potential>r)
			{
				iterationPoint = newPoint;
			}
			step++;
		}
		T = a*T;
		scale = b*scale;
	}
	LocRes[0] = iterationPoint(0);
	LocRes[1] = iterationPoint(1);
	LocRes[2] = iterationPoint(2);
	return LocRes;
}

double geigerSolver::pdist(RowVector3d r1, RowVector3d r2) {
	return sqrt(((r1 - r2).cwiseProduct(r1 - r2)).sum());
}

void geigerSolver::resRevised() {
	for (int i = 0; i < 3; i++) {
		if (LocRes[i] < 0) LocRes[i] = 0;
		if (LocRes[i] > this->spaceLimit(i)) LocRes[i] = spaceLimit(i);
	}
	//在修正的基础上随机取点，取评价函数最小点作为结果，默认计算5次
	//可以不使用
	/*
	RowVector3d currentPoint;
	RowVector3d randStep;
	int scale = 3;
	randStep = RowVector3d::Random() * scale;
	currentPoint << LocRes[0], LocRes[1], LocRes[2];
	int reviseStep = 5;
	while (reviseStep > 0) {
		RowVector3d pendingPoint = currentPoint + randStep;
		if (this->targetFunc(pendingPoint) < this->targetFunc(currentPoint)) {
			currentPoint = pendingPoint;
			LocRes[0] = currentPoint(0);
			LocRes[1] = currentPoint(1);
			LocRes[2] = currentPoint(2);
		}

		reviseStep--;
	}*/
}

bool geigerSolver::isInBox() {
	if ((LocRes[0] > 0 && LocRes[1] > 0 && LocRes[2] > 0 && 
		LocRes[0] < this->spaceLimit(0) &&LocRes[1] < this->spaceLimit(1) &&LocRes[2] < this->spaceLimit(2))) {		
		return true;
	}
	return false;
}

double geigerSolver::targetFunc(RowVector3d rv) {
	double Res=0;
	for (int i = 0; i < sensorNumber; i++) {
		double temp = pdist(rv, sensorLoc.row(i)) - this->sonicSpeed*(this->arrivalTime(i));
		Res=Res+temp*temp;
	}
	return Res;
}