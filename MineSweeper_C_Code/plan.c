#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "plan.h"

/*-----------------------------------------
 * Print out Grid Map
 *-----------------------------------------*/
void GridMapPrint(int** map_grid,int MapSizeX, int MapSizeY)
{
	FILE *file;
	file = fopen("Grid_Map.txt","a+");
	int i,j;
	printf("----Grid Map:----\n");
	for(i = MapSizeY - 1; i >= 0; i--) {
		for(j = 0; j < MapSizeX; j++){
			printf("%3d ",map_grid[i][j]);
			fprintf(file,"%3d",map_grid[i][j]);
		}
		printf("\n");
		fprintf(file,"\n");
	}
	printf("------------------\n");
	fprintf(file,"------------------\n");
	fclose(file);
}

/*-----------------------------------------
 * Print out Flow Map
 *-----------------------------------------*/
void PrintFlowMap(flow_grid** flow_map,int MapSizeX, int MapSizeY){
	int i,j;
	printf("----Flow/Trajectory Map:----\n");
	for(i = MapSizeY - 1; i >= 0; i--) {
		for(j = 0; j < MapSizeX; j++){
			printf("^%2dv%2d<%2d>%2d|",flow_map[i][j].Up,flow_map[i][j].Down,flow_map[i][j].Left,flow_map[i][j].Right);
		}
		printf("\n\n");
	}
	printf("------------------\n");
}

/*-----------------------------------------
 * Initialize Flow Map, all fields to -1
 *-----------------------------------------*/
void InitFlowMap(flow_grid** flow_map,int MapSizeX, int MapSizeY){
	int i,j;
	for(i = 0; i < MapSizeY; i++) {
		for(j = 0; j < MapSizeX; j++){
			flow_map[i][j].Up = -1;
			flow_map[i][j].Down = -1;
			flow_map[i][j].Left = -1;
			flow_map[i][j].Right = -1;
		}
	}
}

/*-----------------------------------------
 * Mark Flow Map based on flow record
 * !!!NOTE!!!: if the last and the first entries are the same, last one should be deleted
 *-----------------------------------------*/
void Flow2Map(int flow_rcd[][3], int length, flow_grid** flow_map,int MapSizeX, int MapSizeY)
{
	int rcd_num;
	for(rcd_num = 0; rcd_num < length; rcd_num++) {
		switch (flow_rcd[rcd_num][2]) {
		case UP:
			flow_map[flow_rcd[rcd_num][1]][flow_rcd[rcd_num][0]].Up = rcd_num;
			break;
		case DOWN:
			flow_map[flow_rcd[rcd_num][1]][flow_rcd[rcd_num][0]].Down = rcd_num;
			break;
		case LEFT:
			flow_map[flow_rcd[rcd_num][1]][flow_rcd[rcd_num][0]].Left = rcd_num;
			break;
		case RIGHT:
			flow_map[flow_rcd[rcd_num][1]][flow_rcd[rcd_num][0]].Right = rcd_num;
			break;
		}
	}
}

/*-----------------------------------------
 * Mark Grid Map based on flow record
 * 0: path, 1: obstacle, -1: unexplored
 *-----------------------------------------*/
void MarkGridMap(int flow_rcd[][3], int length, int **map_grid,int MapSizeX, int MapSizeY)
{
	int rcd_num, x, y;
	for(rcd_num = 0; rcd_num < length; rcd_num++) {
		x = flow_rcd[rcd_num][0]; // horizontal grid coordinate
		y = flow_rcd[rcd_num][1]; // vertical grid coordinate
		if((map_grid)[y][x] == -1){
			(map_grid)[y][x] = 0;
		}
	}
	//  walk through again to mark the obstacles
	// !!!NOTE: only works when obstacle has neighboring cell(s) passed by the robot
	for(rcd_num = 0; rcd_num < length; rcd_num++) {
		x = flow_rcd[rcd_num][0]; // horizontal grid coordinate
		y = flow_rcd[rcd_num][1]; // vertical grid coordinate
		switch (flow_rcd[rcd_num][2]) {
		case UP: // check right side
			x++;
			if(x < MapSizeX){
				if((map_grid)[y][x] == -1){
					(map_grid)[y][x] = 1; // 1 represents obstacle
				}
			}
			break;
		case DOWN: // check left
			x--;
			if(x >= 0){
				if((map_grid)[y][x] == -1){
					(map_grid)[y][x] = 1; // 1 represents obstacle
				}
			}
			break;
		case LEFT: // check top
			y++;
			if(y < MapSizeY){
				if((map_grid)[y][x] == -1){
					(map_grid)[y][x] = 1; // 1 represents obstacle
				}
			}
			break;
		case RIGHT: // check bottom
			y--;
			if(y >= 0){
				if((map_grid)[y][x] == -1){
					(map_grid)[y][x] = 1; // 1 represents obstacle
				}
			}
			break;
		}
		
	}
}

/*-----------------------------------------
 * decide which grid to stop at, and which is the targeted unkown
 * grid, find the pair with minimum cost
 * use: 1. grid map, 2. flow map
 * modify: grid coordination of two grids, exploreDirect: direction from
 * stop cell to target cell
 * return 0 when found, -1 not found
 *-----------------------------------------*/
int Planning(int** map_grid, flow_grid** flow_map,
	         int gStop[], int gTarget[], int* gexploreDirect,int MapSizeX, int MapSizeY)
{
	int cost = INT_MAX;
	int grid_dist;      // distance within which to find stop and target pair
	int weightDist = 0; // distance weight
	int y, x;
	int ret = -1;			// return value

	for(grid_dist = 1; grid_dist <= 2; grid_dist++) {  // max dist == 2
		for(y = 0; y < MapSizeY; y++){
			for(x = 0; x < MapSizeX; x++){
				if (map_grid[y][x] == -1){ // find a cell unexplored
					// Check top cell
					if(y + grid_dist < MapSizeY){
						// LEFT direction
						if(flow_map[y+grid_dist][x].Left >= 0 && flow_map[y+grid_dist][x].Left + grid_dist*weightDist < cost){
							cost = flow_map[y+grid_dist][x].Left + grid_dist*weightDist;
							gStop[0] = x; gStop[1] = y+grid_dist; gStop[2] = LEFT;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = LEFT;
						}
						// FORWARD direction
						if(flow_map[y+grid_dist][x].Down >= 0 && flow_map[y+grid_dist][x].Down + grid_dist*weightDist < cost){
							cost = flow_map[y+grid_dist][x].Down + grid_dist*weightDist;
							gStop[0] = x; gStop[1] = y+grid_dist; gStop[2] = DOWN;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = FORWARD;
						}
					}
					// Check bottom cell
					if(y - grid_dist >= 0){
						//LEFT 
						if(flow_map[y-grid_dist][x].Right >= 0 && flow_map[y-grid_dist][x].Right + grid_dist*weightDist < cost){
							cost = flow_map[y-grid_dist][x].Right + grid_dist*weightDist;
							gStop[0] = x; gStop[1] = y-grid_dist; gStop[2] = RIGHT;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = LEFT;
						}
						// FORWARD
						if(flow_map[y-grid_dist][x].Up >= 0 && flow_map[y-grid_dist][x].Up + grid_dist*weightDist < cost){
							cost = flow_map[y-grid_dist][x].Up + grid_dist*weightDist;
							gStop[0] = x; gStop[1] = y-grid_dist; gStop[2] = UP;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = FORWARD;
						}
					}
					// Check right cell
					if(x + grid_dist < MapSizeX){
						// LEFT direction
						if(flow_map[y][x+grid_dist].Up >= 0 && flow_map[y][x+grid_dist].Up + grid_dist*weightDist < cost){
							cost = flow_map[y][x+grid_dist].Up + grid_dist*weightDist;
							gStop[0] = x+grid_dist; gStop[1] = y; gStop[2] = UP;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = LEFT;
						}
						// FORWARD direction
						if(flow_map[y][x+grid_dist].Left >= 0 && flow_map[y][x+grid_dist].Left + grid_dist*weightDist < cost){
							cost = flow_map[y][x+grid_dist].Left + grid_dist*weightDist;
							gStop[0] = x+grid_dist; gStop[1] = y; gStop[2] = LEFT;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = FORWARD;
						}
					}
					// Check left cell
					if(x - grid_dist >= 0){
						// LEFT direction
						if(flow_map[y][x-grid_dist].Down >= 0 && flow_map[y][x-grid_dist].Down + grid_dist*weightDist < cost){
							cost = flow_map[y][x-grid_dist].Down + grid_dist*weightDist;
							gStop[0] = x-grid_dist; gStop[1] = y; gStop[2] = DOWN;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = LEFT;
						}
						// FORWARD direction
						if(flow_map[y][x-grid_dist].Right >= 0 && flow_map[y][x-grid_dist].Right + grid_dist*weightDist < cost){
							cost = flow_map[y][x-grid_dist].Right + grid_dist*weightDist;
							gStop[0] = x-grid_dist; gStop[1] = y; gStop[2] = RIGHT;
						    gTarget[0] = x; gTarget[1] = y; 
							*gexploreDirect = FORWARD;
						}
					}
				}
			} //for x
		}// for y
		if (cost < INT_MAX) {
			ret = 0;
			break;
		}
	}

	return ret;
}

int PlanningReturn(int flow_rcd_outer[][3], int length, flow_grid** flow_map, int gStop[], int gTarget[], int* gexploreDirect)
{
	int out_dist, x, y, direct, rev_dir;
		int cost = INT_MAX;
		int ret;
		for(out_dist = 0; out_dist < length; out_dist++){
			x = flow_rcd_outer[length - 1 - out_dist][0]; // reverse order
			y = flow_rcd_outer[length - 1 - out_dist][1];
			direct = flow_rcd_outer[length - 1 - out_dist][2];
			if (direct > 0) {
				rev_dir = DirRev(direct);
				switch (rev_dir){
				case UP:
					if (flow_map[y][x].Up >= 0 && flow_map[y][x].Up + out_dist < cost) {
						cost = flow_map[y][x].Up + out_dist;
						gStop[0] = x; gStop[1] = y; gStop[2] = UP;
						gTarget[0] = x; gTarget[1] = y; 
						*gexploreDirect = LEFT;
					}
					break;
				case DOWN:
					if (flow_map[y][x].Down >= 0 && flow_map[y][x].Down + out_dist < cost) {
						cost = flow_map[y][x].Down + out_dist;
						gStop[0] = x; gStop[1] = y; gStop[2] = DOWN;
						gTarget[0] = x; gTarget[1] = y; 
						*gexploreDirect = LEFT;
					}
					break;
				case LEFT:
					if (flow_map[y][x].Left >= 0 && flow_map[y][x].Left + out_dist < cost) {
						cost = flow_map[y][x].Left + out_dist;
						gStop[0] = x; gStop[1] = y; gStop[2] = LEFT;
						gTarget[0] = x; gTarget[1] = y; 
						*gexploreDirect = LEFT;
					}
					break;
				case RIGHT:
					if (flow_map[y][x].Right >= 0 && flow_map[y][x].Right + out_dist < cost) {
						cost = flow_map[y][x].Right + out_dist;
						gStop[0] = x; gStop[1] = y; gStop[2] = RIGHT;
						gTarget[0] = x; gTarget[1] = y; 
						*gexploreDirect = LEFT;
					}
					break;
				}
			}
		}
		if(cost < INT_MAX){
			ret = 0;
		} else {
			ret = -1;
		}
		return ret;
}

/*-----------------------------------------
 * Find the reverse direction of the input one
 * Used by PlanningReturn()
 *-----------------------------------------*/
int DirRev(int direct)
{
	int rev_dir;
	switch (direct) {
	case UP:
		rev_dir = DOWN;
		break;
	case DOWN:
		rev_dir = UP;
		break;
	case LEFT:
		rev_dir = RIGHT;
		break;
	case RIGHT:
		rev_dir = LEFT;
		break;
	}
	return rev_dir;
}
