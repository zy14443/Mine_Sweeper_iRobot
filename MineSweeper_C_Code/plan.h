#ifndef _PLAN_H
#define _PLAN_H

#define UP    1
#define FORWARD    1
#define DOWN  2
#define LEFT  3
#define RIGHT 4


typedef struct { // entry used in Flow Map
	int Up;		  // field keeps the step number on that direction
	int Down;
	int Left;
	int Right;
}flow_grid;



void InitFlowMap(flow_grid** flow_map,int MapSizeX, int MapSizeY);
void GridMapPrint(int** map_grid,int MapSizeX, int MapSizeY);
void PrintFlowMap(flow_grid** flow_map,int MapSizeX, int MapSizeY);
void Flow2Map(int flow_rcd[][3], int length, flow_grid** flow_map,int MapSizeX, int MapSizeY);
void MarkGridMap(int flow_rcd[][3], int length, int **map_grid,int MapSizeX, int MapSizeY);
int Planning(int **map_grid,flow_grid** flow_map,
	        int* gStop, int *gTarget, int* gexploreDirect,int MapSizeX, int MapSizeY);
int PlanningReturn(int flow_rcd_outer[][3], int length, flow_grid** flow_map,
				   int gStop[], int gTarget[], int* gexploreDirect);
int DirRev(int direct);

#endif
