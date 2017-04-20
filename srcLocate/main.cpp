#include <iostream>
#include "Geiger.h"
#include "Sphere.h"
#include "Plane.h"
#include "Bottom.h"
using namespace Eigen;
using namespace std;

#define PI 3.1415926
int main()
{
	double LocOfSensors[12] = { 0,50,100,0,50,0,50,0,0,50,0,100 };							//按探头顺序存储的当前计算所用的传感器位置
    int NumOfSensors = 4;																			//使用的传感器数量，当前为5
	float LimitOfSpace[3] = { 100,100,100 };															//三维体的空间限制
    float SpeedOfSound=3200*1000;																	//声速
    double TimeOfArrival[4]={1242e-7,1290e-7,1298e-7,1385e-7 };								//依次的到达时间，单位为s
    
	geigerSolver Test(LocOfSensors, TimeOfArrival, LimitOfSpace, NumOfSensors, SpeedOfSound);
    double* Res=Test.doSolvePlane();
    cout<<"3D space test"<<endl;
    for(int i=0;i<3;i++){
        cout<<*(Res+i)<<endl;
    }

    NumOfSensors = 3;
	SpeedOfSound = 3000 * 1000;
    float Radius = 1000;
    double LocOfSensors2[9] = {PI/2, 0 , PI/2, PI/2 , 0, 0};
	double TimeOfArrival2[3] = { 0.000349065850398866,0.000349065850398866,0.000261799387799149 };
    cout<<"Sphere test"<<endl;
    sphereSolver Test2(LocOfSensors2,TimeOfArrival2,Radius,NumOfSensors,SpeedOfSound);
    Res = Test2.doSolve();
    for(int i=0;i<3;i++){
        cout<<*(Res+i)<<endl;
    }

	NumOfSensors = 3;
	SpeedOfSound = 3000 * 1000;
	double LocOfSensors3[6] = { 0,0,0,100,100,0 };
	double TimeOfArrival3[3] = { 1242e-7,1290e-7,1298e-7 };
	float LimitOfSpace3[2] = { 100,100 };
	planeSolver Test3(LocOfSensors3, TimeOfArrival3, LimitOfSpace3, NumOfSensors, SpeedOfSound);
	cout << "Plane test" << endl;
	Res = Test3.doSolve();
	for (int i = 0; i<2; i++) {
		cout << *(Res + i) << endl;
	}

	NumOfSensors = 3;
	SpeedOfSound = 3000 * 1000;
	double LocOfSensors4[6] = { 0,100,100,0,60,80 };
	double TimeOfArrival4[3] = { 1242e-7,1290e-7,1298e-7 };
	double Height = 100;
	double Radius2 = 100;
	bottomSolver Test4(LocOfSensors4, TimeOfArrival4, NumOfSensors, SpeedOfSound, Height, Radius2);
	cout << "Bottom test" << endl;
	Res = Test4.doSolve();
	for (int i = 0; i<2; i++) {
		cout << *(Res + i) << endl;
	}
}