#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include "createoi.h"
#include <math.h>
#include <string.h>
#include "createoi_patch.h"
#include "control.h"
#include "plan.h"

#define N 50

int theta = 90, l = 0; //Angle and delta length

int MinesDetected[] = {0,0}; //Mark mines detected

int Map[N][N]; //Large Map before start

int Grid_X, Grid_Y;
int Direction_X = 0, Direction_Y = 1;
float Position_X = 0, Position_Y = -470; //Start Position

int Start_X, Start_Y;
int Stop_X, Stop_Y;

int flow_rcd[100][3];
int flow_rcd_outer[100][3];

int Current_Step = 0;
int Total_Step = 0;

int** map_grid;		// Grid Map: -1 meaning unexplored cell
flow_grid** flow_map;	// Flow Map: used for planning

int gStop[3];;  // grid coordination of the cell to stop at
int gTarget[3];   // grid coordination of target cell
int gexploreDirect;    // direction from stop cell to target cell

int Virtual_Wall_ON = 0;

pthread_t control_thread;

void* Controller(void* arg);

int startControl()
{
	pthread_create(&control_thread, NULL, &Controller, NULL);
	
}

void *Controller(void *arg)
{
	
	//Reset Map and flow_rcd
	memset(Map,0,sizeof(Map));
	memset(flow_rcd,0, sizeof(flow_rcd));	


	//Initialize Start Position
	Start_X = 25;
	Start_Y = 0;
	Grid_X = Start_X;
	Grid_Y = Start_Y;
	RunDist(380);
	TurnRadius(-90,-1);
	ChangeDirection();
	Virtual_Wall_ON = 1;
		
	//First time follow the boundary of the maze
	Stop_X = 25, Stop_Y = 0;
	WallFollower(1);
	drive(0,0);

	FILE *file;
	file = fopen("Grid_Map.txt","a+");
	
	//Calculate Maze size
	int i,j;
	int X_Max=0,X_Min=N,Y_Max=0;
	for (i=0; i<N; i++)
	{
		for (j =0; j<N; j++)
		{	
			if(Map[i][j])
			{
				if(i<X_Min)
					X_Min = i;
				if(i>X_Max)
					X_Max = i;
				if(j>Y_Max)
					Y_Max = j;
					
			}				
		}		
	}
	
	int X_Length = X_Max-X_Min+1;
	int Y_Length = Y_Max+1;
		
	printf("X_Length: %d Y_Length: %d\n", X_Length, Y_Length);
	fprintf(file,"X_Length: %d Y_Length: %d\n", X_Length, Y_Length);
	fprintf(file,"-----------\n");
	fclose(file);
		
	//Generate flow map
	for (i=0; i<=Current_Step; i++)
	{
		//FILE *file;
		file = fopen("Grid_Map.txt","a+"); //print flow map into a file
		flow_rcd[i][0] -= X_Min;
		flow_rcd_outer[i][0] = flow_rcd[i][0];
		flow_rcd_outer[i][1] = flow_rcd[i][1];
		flow_rcd_outer[i][2] = flow_rcd[i][2];
		printf("X: %d Y: %d Direction: %d\n", flow_rcd_outer[i][0],flow_rcd_outer[i][1],flow_rcd_outer[i][2]);
		fprintf(file,"X: %d Y: %d Direction: %d\n", flow_rcd_outer[i][0],flow_rcd_outer[i][1],flow_rcd_outer[i][2]);
		fclose(file);
	}
	Total_Step = Current_Step;
	

	// initialize map_grid to -1: unexplored
	map_grid = (int**) malloc(sizeof(int*)*X_Length);
	flow_map = (flow_grid**) malloc(sizeof(flow_grid*)*Y_Length);
	
	for(i = 0; i < Y_Length; i++) {
		map_grid[i] = (int*) malloc(sizeof(int)*X_Length);
		flow_map[i] = (flow_grid*) malloc(sizeof(flow_grid)*Y_Length);
		for(j = 0; j < X_Length; j++){
			
			map_grid[i][j] = -1;
		}
	}


	//Plan Next Stop
	Plan_First(X_Length, Y_Length);
	Start_X -= X_Min;
	Start_Y = 0;	
	
	//Change Grid to the size of the map
	Grid_X -= X_Min; 
	Grid_Y = Start_Y;
	
	while(MinesDetected[0]==0 | MinesDetected[1]==0) //While mines are not found
	{
			
		Current_Step = 0;
		memset(flow_rcd,0, sizeof(flow_rcd));
		WallFollower(2); //Mode 2 Stop at a specified point
		drive(0,0);
		RunDist(100);
		
		file = fopen("Grid_Map.txt","a+");	 //print flow map into a file	
		for (i=0; i<=Current_Step; i++)
		{
			
			fprintf(file,"X: %d Y: %d Direction: %d\n", flow_rcd[i][0],flow_rcd[i][1],flow_rcd[i][2]);
			
		}
		fprintf(file,"-----------\n");
		fclose(file);
			
		if (gexploreDirect == 3) //if stop and turn 90 degrees
		{
			TurnRadius(90,1);
			ChangeDirection();
		}
		DriveUntilBump();
		
		Current_Step = 0;
		memset(flow_rcd,0, sizeof(flow_rcd));	
		Stop_X = Grid_X;
		Stop_Y = Grid_Y;
		WallFollower(1); //Mode 1 Follow until reaching the start point again
		drive(0,0);
		RunDist(100);
		
		file = fopen("Grid_Map.txt","a+");		
		for (i=0; i<=Current_Step; i++) //print flow map into a file	
		{
			
			fprintf(file,"X: %d Y: %d Direction: %d\n", flow_rcd[i][0],flow_rcd[i][1],flow_rcd[i][2]);
			
		}
		fprintf(file,"-----------\n");
		fclose(file);
		printf("finish wall follow_1\n");
		Plan_First(X_Length,Y_Length);	//Plan next stop	
	}
	
	Plan_Return(X_Length, Y_Length); //When all the mines are found, plan the point to stop
	Current_Step = 0;
	memset(flow_rcd,0, sizeof(flow_rcd));	
	WallFollower(2); //Mode 2 Follow the wall to stop point
	drive(0,0);
	RunDist(100);

	if (gexploreDirect == 3)
	{
		TurnRadius(90,1);
		ChangeDirection();
	}
	
	DriveUntilBump();	
	Current_Step = 0;
	memset(flow_rcd,0, sizeof(flow_rcd));
	
	Stop_X = Start_X;
	Stop_Y = Start_Y-1; //Set the stop point

	Virtual_Wall_ON = 0; //Turn off virtual wall
	WallFollower(1); //Follow the wall to exit

	return NULL;
}

//Plan the Stop point
int Plan_First(int X_Length, int Y_Length)
{
		
	InitFlowMap(flow_map,X_Length,Y_Length);
	
	// Step 1: Use flow_rcd to generate flow_map
	Flow2Map(flow_rcd, Current_Step, flow_map,X_Length,Y_Length);
	PrintFlowMap(flow_map,X_Length,Y_Length);
	// Step 2: Use flow_map to mark grid map
	MarkGridMap(flow_rcd, Current_Step, map_grid,X_Length,Y_Length);
	GridMapPrint(map_grid,X_Length,Y_Length);
	// Step 3: Use map_grid and flow_map to generate stop-target pair
	if (Planning(map_grid, flow_map,gStop, gTarget, &gexploreDirect,X_Length,Y_Length)){
		printf("cannot find stop target pair\n");
		return 0;
	} 
	else
	{
		FILE *file;
		file = fopen("Grid_Map.txt","a+"); //print stop point into a file
		printf("Stop = [%d, %d, %d], Target = [%d, %d], exploreDirect = %d\n",
				gStop[0],gStop[1],gStop[2], gTarget[0], gTarget[1], gexploreDirect);
		fprintf(file,"Stop = [%d, %d, %d], Target = [%d, %d], exploreDirect = %d\n",
				gStop[0],gStop[1],gStop[2], gTarget[0], gTarget[1], gexploreDirect);
		fclose(file);
		return 1;
		
	}

}

//Plan return stop point
int Plan_Return(int X_Length, int Y_Length)
{
	InitFlowMap(flow_map,X_Length,Y_Length);

	// Step 1: Use flow_rcd to generate flow_map for the current wall-following circle
	Flow2Map(flow_rcd, Current_Step, flow_map,X_Length,Y_Length);
	PrintFlowMap(flow_map,X_Length,Y_Length);

	// Step 2: Use outer_flow_rcd and flow_map to find stop cell
	if (PlanningReturn(flow_rcd_outer, Total_Step,flow_map,gStop, gTarget, &gexploreDirect)){
		printf("cannot find stop target pair\n");
	} else {
		printf("Stop = [%d, %d, %d], Target = [%d, %d], exploreDirect = %d\n",
				gStop[0],gStop[1],gStop[2], gTarget[0], gTarget[1], gexploreDirect);
	}
}

//Follow the wall
//Input: Mode
//Mode 1: Follow the wall until reaching the start point
//Mode 2: Follow the wall until reaching the specified stop point 
int WallFollower(int mode)
{	
	float dis2wall;  //Distance to the wall
	float pre_cte=0; //Predicted error
	float cte;		// Actual error
	float angVel; //Wall follow angle velocity
	
	int follow_distance = 0; //Memorise the distance of following
		
	getDistance_Patch(); //Clear the distance sensor
	getAngle_Patch(); //Clear the angle sensor
	
	while(1)
	{
				
		dis2wall = Dis2WallSensor();
		
		
		//Virtual wall at start point		
		if (Position_X > -ONE_UNIT && Position_X < ONE_UNIT && Position_Y < 0 && dis2wall == NO_WALL && Virtual_Wall_ON)
		{
			dis2wall = (Position_Y + HALF_UNIT-ROBOT_RADIUS-10)/25.4;
			if (dis2wall < 0)
				dis2wall = 0.1;
		}
		
		//If driving directly on to the virtual wall, treat it like a bump from front
		if (Position_X > - HALF_UNIT && Position_X < HALF_UNIT && Position_Y < -90 && Direction_X == 0 && Direction_Y == -1 && Virtual_Wall_ON)
		{
			CorrectPosition_Front();	//Correct Position when bump front
			drive(0,0);	
			follow_distance = 0;
			pre_cte=0;
			if (TurnRadius(90,1) == -1) //Turn left 90 degrees
				return 0;
			ChangeDirection(); //Change direction
		}
		
		//If bump front, turn left 90 degrees
		else if (getBumpsAndWheelDrops_Patch()==3 | getBumpsAndWheelDrops_Patch()==2)
		{
			
			CorrectPosition_Front();	//Correct Position when bump front
			drive(0,0);	
			follow_distance = 0;
			pre_cte=0;
			if (TurnRadius(90,1) == -1)
				return 0;
			ChangeDirection();
						
		}
		
		//If bump right, turn left 10 degrees
		else if (getBumpsAndWheelDrops_Patch()==1 | dis2wall == 0 )
		{
			CorrectPosition_Right();
			drive(0,0);
			pre_cte=0;
			if (TurnRadius(10,1) == -1)
				return 0;	
			
		}
		
		//If no wall, turn right 90 degrees with radius 210
		else if (dis2wall == NO_WALL)
		{
			
			drive(0,0);
			follow_distance = 0;
			pre_cte=0;
			
			if (TurnRadius(-90, -210) == -1)
				return 0;
			
		}
		
		//Else keep following the wall 
		else 		
		{
			cte =  dis2wall - TARGET_DIS2WALL; //calculate actual error
			angVel = PID(cte, pre_cte);
			pre_cte = cte;
			VelAngVel(150, angVel);
			
			usleep(CYCLE_CONTROL);
			
			l = getDistance_Patch();
			theta += getAngle_Patch();
			theta = (theta+360)%360;
			
			if(ChangeGrid(400))
			{
					follow_distance = 0;
					Current_Step++;				
					UpdateStep();			
			}
			
								
			follow_distance += l; //Calculate the follow distance
					
			//If follow for a while
			if (follow_distance > HALF_BRICK)
			
			{
				MineDetect();	// Detect if there is a mine
				
				CorrectTheta(); //Correct the angle
				CorrectPosition_Follow(); // Correct the position
				follow_distance = 0;

				if (EncodeDirection() != flow_rcd[Current_Step][2])
				{
					if(flow_rcd[Current_Step][2])
						Current_Step++;
										
					UpdateStep();
					if ((Grid_X != flow_rcd[Current_Step -1][0] | Grid_Y != flow_rcd[Current_Step -1][1]) && flow_rcd[Current_Step -1][2] && (EncodeDirection()!= flow_rcd[Current_Step -1][2]))
						flow_rcd[Current_Step][2] = 0;		
					
					else 
						flow_rcd[Current_Step][2] = EncodeDirection();							
				
				}				
												
			}
			if(mode == 1)	//Mode 1: Stop when reaching start point 	
			{
				if (Grid_X == Stop_X && Grid_Y == Stop_Y && Current_Step>0)
					return 0;
			}
			else if (mode == 2) //Mode 2: Stop at a specified point
			{
				if (flow_rcd[Current_Step][0]==gStop[0] && flow_rcd[Current_Step][1]==gStop[1] && flow_rcd[Current_Step][2]==gStop[2])
					return 0;
			}		
			
		}
		
		ChangeDirection();	//Change direction
	
		
	}
	return 0;
	
}

//Update flow map 
int UpdateStep() 
{
	flow_rcd[Current_Step][0] = Grid_X;
	flow_rcd[Current_Step][1] = Grid_Y;
	printf("Current_Step: %d\n", Current_Step);
	printf("X: %d Y: %d Direction: %d\n", flow_rcd[Current_Step][0],flow_rcd[Current_Step][1],flow_rcd[Current_Step][2]);
	return 1;
}


//Drive directly until hit the wall
int DriveUntilBump()
{
	getDistance_Patch();
	getAngle_Patch();
	
	while(!getBumpsAndWheelDrops_Patch())  
	{
		drive(200,0);
		usleep(CYCLE_CONTROL);
		l = getDistance_Patch();
		theta += getAngle_Patch();
		
		theta = (theta+360)%360;
		if(ChangeGrid(400));
			MineDetect();	
	}
	return 1;
}

//Change the grid based on current position
//Input: the critical value to change a grid
int ChangeGrid(int Critical_Value)
{
	FILE* file;
	file = fopen("Position.txt","a+");
	Position_X += l * cos((float)theta/180*3.14);
	fprintf(file, "%f %f;\n",Position_X ,Position_Y); //print all positions into a file
	fclose(file);
	Position_Y += l * sin((float)theta/180*3.14);			
	
	int change = 0; //Mark whether the position changed or not
	if ((Position_X - (Grid_X - Start_X)*ONE_UNIT)>Critical_Value)
	{
		Grid_X ++;
		change = 1;
		Map[Grid_X][Grid_Y]++;
				
	}
	else if ((Position_X - (Grid_X - Start_X)*ONE_UNIT)<-Critical_Value)
	{
		Grid_X --;
		change = 1;
		Map[Grid_X][Grid_Y]++;
		if (Grid_X < 0)
		{	Grid_X = 0;	
			change = 0;
		}
	}
	else if ((Position_Y - (Grid_Y - Start_Y)*ONE_UNIT)>Critical_Value)
	{
		Grid_Y ++;
		change = 1;		
		Map[Grid_X][Grid_Y]++;
		
	}
	else if ((Position_Y - (Grid_Y - Start_Y)*ONE_UNIT)<-Critical_Value)

	{
		Grid_Y --;
		change = 1;		
		Map[Grid_X][Grid_Y]++;
		
	}
	
	return change;
	
}

// Run a certain distance
int RunDist(int dist)
{
	float distRunned=0, pre_de=0;
	float target = dist;	// distance to run
	float de;
	short vel;
	int delta;
	getDistance_Patch(); //set distance sensor to 0
	getAngle_Patch(); //set angle sensor to 0
		
	while (1)
	{
		
		theta += getAngle_Patch();
		theta = (theta+360)%360;
		
		if(getBumpsAndWheelDrops_Patch() && dist >0)
			return 0;
		delta = getDistance_Patch();
		distRunned += delta;
		l = delta;
		
		ChangeGrid(400); 
				
		de = target - distRunned;
		vel = PIDdist(de, pre_de, 150);
		pre_de = de;
		directDrive(vel, vel);
		
		if (abs(de) <= 1) //If error is small enough, return
			break;
		usleep(CYCLE_CONTROL);
		
	}
	return 1;
}

// Turn a certain angle
int TurnRadius(int angle, int radius)
{
	int angleTurned = 0, pre_ae=0;
	int target = angle; //Target Angle
	int ae;
	short vel;
	int dis2wall;
	int delta;
	getAngle_Patch();
	getDistance_Patch();
	
	while(1)
	{
		delta = getAngle_Patch();
		angleTurned += delta;
	
		l = getDistance_Patch();
		theta += delta;
		theta = (theta+360)%360;		
		if(ChangeGrid(330)) //Check if the grid needs to be changed
		{
			Current_Step++;
			UpdateStep();
			MineDetect();	
		}
					
		dis2wall = Dis2WallSensor();
		if (dis2wall < TARGET_DIS2WALL && dis2wall > 0 && getBumpsAndWheelDrops_Patch()==0&& abs(radius) >1)
			return 0;
		
		if((getBumpsAndWheelDrops_Patch()==3 | getBumpsAndWheelDrops_Patch()==2)&& abs(radius) >1)
			return 0;
			
		if (getBumpsAndWheelDrops_Patch()==1 && radius < 0)
			return 0;
		
		ae = target - angleTurned;
		vel = PIDturn(ae,pre_ae, 150,radius)*((radius>0)-(radius<0));
		pre_ae = ae;		
				
		if (abs(ae)<=1) //If error is small enough, return
			break;
		if (radius == 0)
			directDrive (vel,-vel);
		else		
			drive (vel, radius);
		usleep(CYCLE_CONTROL);			
	}
	return 1;

}

//Detect the mines
int MineDetect()
{
	int detect = 0;
	int * Mines = getCliffSignals(); 
	int i,Max=0;
	for (i=0; i<4; i++)
	{
		if (Mines[i] > Max)
			Max = Mines[i];
	}	
	
	//printf("Max = :%d\n", Max);	
	if (Max > 600 && Max < 800 && (MinesDetected[0]==0)) //200-800
	{	
		detect = 1;
		MinesDetected[0]=1;
	}
	else if (Max >2000 && (MinesDetected[1]==0)) //>800
	{
		detect = 2;	
		MinesDetected[1]=1;
	}	
				
	free(Mines);	
			
	if (detect)
	{	
		drive(0,0);
		horn(); //Play a song
		usleep(3000000);
		
	}
	
	return detect;	
}



 // measure the distance to the right side wall, using wall sensor signal
 float Dis2WallSensor()
{
	int dist[4] = {0,   1,    2,  3}; 	// Unit: inch
	int read[4] = {450, 270, 45,  4};   // Sensor signals
	unsigned int wall_signal;
	float dis2wall;
	int i;
	wall_signal = getWallSignal();
	
	if (read[0] < wall_signal) {
		dis2wall = 0;
	}
	else if (read[1] < wall_signal && wall_signal <= read[0]) {
		i = 0;
		dis2wall = (float)((dist[i] - dist[i+1]) * wall_signal
			+ dist[i+1] * read[i] - dist[i] * read[i+1])
			/ (read[i] - read[i+1]);
		
	}
	else if (read[2] < wall_signal && wall_signal <=  read[1]) {
		i = 1;
		dis2wall = (float)((dist[i] - dist[i+1]) * wall_signal
			+ dist[i+1] * read[i] - dist[i] * read[i+1])
			/ (read[i] - read[i+1]);
	}
	else if (read[3] < wall_signal && wall_signal <= read[2]) {
		i = 2;
		dis2wall = (float)((dist[i] - dist[i+1]) * wall_signal
			+ dist[i+1] * read[i] - dist[i] * read[i+1])
			/ (read[i] - read[i+1]);
	}
	else {
		dis2wall = NO_WALL;	// means no wall

	}
	return dis2wall;
}

// set forward and angular velocities of the robot

int VelAngVel(short vel, float angvel)
{
	int b = 258; // unit: mm, axel length
	short vl, vr; // velocities of left and right wheels
	int ret = 0;	
	int overflow = 0;
	
	vr = (short)(vel + (b>>1) * angvel);
	vl = (short)(vel - (b>>1) * angvel);

	if(vr > MAX_VEL) {
		vr = MAX_VEL;
		overflow = 1;
	} else if (vr < MIN_VEL) {
		vr = MIN_VEL;
		overflow = 1;
	}
	if (vl > MAX_VEL) {
		vl = MAX_VEL;
		overflow = 1;
	} else if (vl < MIN_VEL) {
		vl = MIN_VEL;
		overflow = 1;
	}
	if(overflow == 1){
		printf("Warning: wheel velocity overflow!\n");
	}
	ret = directDrive(vl, vr);
	return ret;
} 

// PID controller for wall follower
//input: cte error
// output: du, angular control
float PID(float cte, float pre_cte)
{
	float max_du = 3.14/6.0;
	float para_P = 0.2, para_D = 2; //0.4, 10 ok
	float diff_cte = cte - pre_cte;
	float du;
	du = -para_P * cte - para_D * diff_cte;
	if (du > max_du) {
		du = max_du;
	} else if (du < -max_du) {
	        du = -max_du;
	}
	
    return du;
}

// PID controller for distance
// input: cte error
// output: dv, angular control
short PIDdist(float de, float pre_de, short vel)
{
	float para_P = 4, para_D = 4; //0.4, 10 ok
	float diff_de = de - pre_de;
	short dv;
	
	dv = para_P * de + para_D * diff_de;
	if (dv > vel) {
		dv = vel;
	}
	
    return dv;
}

// PID controller for turn radius
// input: cte error
// output: dv, angular control
short PIDturn(int ae, int pre_ae, short vel,int radius)
{
	float para_P = 3, para_D = 2; //0.4, 10 ok
	int diff_ae = ae - pre_ae;
	float dw;
	int dv;

	dw = (para_P * ae + para_D * diff_ae) * 3.14 / 180;  // angular velocity: radian/sec
	
	if (abs(radius) < 130)
		dv = (int)(dw*130);
	else	
		dv = (int)(dw * abs(radius));
	if (abs(dv) > vel) {
		dv = vel*((dv>0)-(dv<0));
	}
    return (short)dv;
}

//Correct Position when following for a while
int CorrectPosition_Follow()
{
	if(Direction_X == 0 && Direction_Y == 1)
	{
		Position_X = (Grid_X - Start_X)*ONE_UNIT + HALF_UNIT - ROBOT_RADIUS-75;
	}
	else if (Direction_X == 1 && Direction_Y == 0)
	{
		Position_Y = (Grid_Y - Start_Y)*ONE_UNIT - HALF_UNIT + ROBOT_RADIUS+75;
	}
	else if (Direction_X == 0 && Direction_Y == -1)
	{
		Position_X = (Grid_X - Start_X)*ONE_UNIT - HALF_UNIT + ROBOT_RADIUS+75;
	}
	else if (Direction_X == -1 && Direction_Y == 0)
	{
		Position_Y = (Grid_Y - Start_Y)*ONE_UNIT + HALF_UNIT - ROBOT_RADIUS-75;
	}
	return 0;
}

// Correct the position when bump on the right
int CorrectPosition_Right()
{
	printf("CorrectPosition_Right\n");
	if(Direction_X == 0 && Direction_Y == 1)
	{
		Position_X = (Grid_X - Start_X)*ONE_UNIT + HALF_UNIT - ROBOT_RADIUS-25;
	}
	else if (Direction_X == 1 && Direction_Y == 0)
	{
		Position_Y = (Grid_Y - Start_Y)*ONE_UNIT - HALF_UNIT + ROBOT_RADIUS+25;
	}
	else if (Direction_X == 0 && Direction_Y == -1)
	{
		Position_X = (Grid_X - Start_X)*ONE_UNIT - HALF_UNIT + ROBOT_RADIUS+25;
	}
	else if (Direction_X == -1 && Direction_Y == 0)
	{
		Position_Y = (Grid_Y - Start_Y)*ONE_UNIT + HALF_UNIT - ROBOT_RADIUS-25;
	}
	return 0;
}

//Correct the position when bump on the front
int CorrectPosition_Front()
{
	printf("CorrectPosition_Front\n");
	//printf("Direction_X: %d, Direction_Y, %d\n",Direction_X,Direction_Y);

	if(Direction_X == 0 && Direction_Y == 1)
	{
		Position_Y = (Grid_Y - Start_Y)*ONE_UNIT + HALF_UNIT - ROBOT_RADIUS-25;
	}
	else if (Direction_X == 1 && Direction_Y == 0)
	{
		Position_X = (Grid_X - Start_X)*ONE_UNIT + HALF_UNIT - ROBOT_RADIUS-25;
	}
	else if (Direction_X == 0 && Direction_Y == -1)
	{
		Position_Y = (Grid_Y - Start_Y)*ONE_UNIT - HALF_UNIT + ROBOT_RADIUS+25;
	}
	else if (Direction_X == -1 && Direction_Y == 0)
	{
		Position_X = (Grid_X - Start_X)*ONE_UNIT - HALF_UNIT + ROBOT_RADIUS+25;
	}
	return 0;
}

//Change direction according to current angle
int ChangeDirection()
{
	int Change_X = Direction_X, Change_Y = Direction_Y;
	
	if(theta < 40 | (theta >320 && theta <360))
	{
		Change_X = 1;
		Change_Y = 0;		
	}
	else if (theta >50 && theta <130)
	{
		Change_X = 0;
		Change_Y = 1;
		
	}
	else if (theta > 140 && theta < 220)
	{
		Change_X = -1;
		Change_Y = 0;
		
	}
	else if (theta > 230 && theta <310) 
	{
		Change_X = 0;
		Change_Y = -1;		
	}
	if (Change_X == Direction_X && Change_Y == Direction_Y)
		return 0;
	else

	{	
		Direction_X = Change_X;
		Direction_Y = Change_Y;
		printf("theta: %d Direction_X %d Direction_& %d\n", theta, Direction_X, Direction_Y);
		return 1;
	}
}

//Correct the angle when following for a while
int CorrectTheta()
{
	if(Direction_X == 0 && Direction_Y == 1)
	{
		theta = 90;
	}
	else if (Direction_X == 1 && Direction_Y == 0)
	{
		theta = 0;
	}
	else if (Direction_X == 0 && Direction_Y == -1)
	{
		theta = 270;
	}
	else if (Direction_X == -1 && Direction_Y == 0)
	{
		theta = 180;
	}
	return 0;
}

//Encode the direction
//Up 1; Down 2; Left 3; Right 4
int EncodeDirection()
{
	
	if(Direction_X == 0 && Direction_Y == 1)
		return 1;
	else if (Direction_X == 1 && Direction_Y == 0)
		return 4;
	else if (Direction_X == 0 && Direction_Y == -1)
		return 2;
	else if (Direction_X == -1 && Direction_Y == 0)
		return 3;
	else
		return 0;

}

//Play a song
void horn() {
  	char song[SONG_LENGTH * 2];
  	int i;
  	for (i =0; i< SONG_LENGTH; i++) {
    		song[i*2] = 100 ;
    		song[i*2 +1] = 32 ;
  	}
  	writeSong(0, SONG_LENGTH, song);
  	playSong (0);
}

//Stop control
int stopControl()
{
	pthread_join(control_thread, NULL);
}
