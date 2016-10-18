#include "Sphere.h"
#include <math.h>

sphereSolver::sphereSolver(double *LocOfSensor,double *TimeOfArrival,
								float *Radius,int NumOfSensors,float SpeedOfSound){
	sensorNumber=NumOfSensors;
	sonicSpeed = SpeedOfSound;
	radius = Radius;
	arrivalTime = VectorXd::Zero(sensorNumber);
	sensorLoc = Matrix<double, Dynamic, 2>::Zero(sensorNumber, 2);

	for(int i=0;i<NumOfSensors;i++){
		arrivalTime(i) = *(TimeOfArrival + i);
		sensorLoc(i,0) = *(LocOfSensor + 2 * i + 0);
		sensorLoc(i,1) = *(LocOfSensor + 2 * i + 1); 
	}
}

