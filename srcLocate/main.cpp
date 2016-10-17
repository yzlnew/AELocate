#include <iostream>
#include "Geiger.h"
using namespace Eigen;
using namespace std;

int main()
{
	double LocOfSensors[15] = { 0,0,0,0,30,0,0,30,144,64,0,144,64,30,144 };							//按探头顺序存储的当前计算所用的传感器位置
    int NumOfSensors = 5;																			//使用的传感器数量，当前为5
	float LimitOfSpace[3] = { 64,30,144 };															//三维体的空间限制
    float SpeedOfSound=2000*1000;																	//声速
    double TimeOfArrival[5]={6854e-7,6885e-7,7314e-7,7368e-7,7409e-7 };								//依次的到达时间，单位为s
    
	geigerSolver Test(LocOfSensors, TimeOfArrival, LimitOfSpace, NumOfSensors, SpeedOfSound);
    double* Res=Test.doSolve();
    for(int i=0;i<3;i++){
        cout<<*(Res+i)<<endl;
    }

}