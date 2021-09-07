#ifndef H_CONTROL_GD
#define H_CONTROL_GD

#include <unistd.h>
#include "createoi.h"
#include "createoi_patch.h"

#define CYCLE_CONTROL 15000 	//15 ms
#define MIN_VEL -500		//-500mm/s
#define MAX_VEL 500		//500mm/s
#define ONE_UNIT 600		//600mm
#define HALF_UNIT 300		//300mm
#define HALF_UNIT_GRID 350	//350mm
#define HALF_BRICK 130		//130mm
#define ROBOT_RADIUS 170	//170mm
#define SONG_LENGTH 6
#define TARGET_DIS2WALL 1.8
#define NO_WALL -1

int startControl();

int WallFollower(int mode);
int RunDist(int dist);
int TurnRadius(int angle, int radius);

void DodgeLeft();
void StopCreate();

int MineDetect();
int CurrentPosition();

int CorrectTheta();
int CorrectPosition_Front();
int CorrectPosition_Right();
int CorrectPosition_Follow();

int ChangeGrid(int Critical_Value);
int ChangeDirection();

int EncodeDirection();

int Plan_First(int X_Length, int Y_Length);
 
int DriveUntilBump();

int UpdateStep(); 


float Dis2WallSensor();
int VelAngVel(short vel, float angvel);
float PID(float cte, float pre_cte);
short PIDdist(float de, float pre_de, short vel);
short PIDturn(int ae, int pre_ae, short vel,int radius);

void horn();

int stopControl();

#endif


