#include <stdlib.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sys/time.h>
#include <pthread.h>
#include "createoi.h"
#include "createoi_patch.h"

#define CYCLE_TIME_PATCH 20000  //delay (in mircoseconds) between reads in MT mode.

static int THREAD_MODE_PATCH = 0;            ///multi-thread mode status. 
pthread_mutex_t sensor_cache_patch_mutex;    ///locks sensor cache struct.

pthread_t sensor_thread_patch;

void *sensorThreadFuncPatch( void *ptr );

typedef struct {
	int wall_signal;
	int cliff_left_signal;
	int cliff_front_left_signal;
	int cliff_front_right_signal;
	int cliff_right_signal;
	int bumps_and_wheel_drops;
	int distance;
	int angle;
	int shut_down; 
} sensor_cache_patch_t;

sensor_cache_patch_t * sensor_cache_patch;

int startOI_MT_Patch(const char* serial)
{
	if (startOI(serial) != 0)
		return -1;
	THREAD_MODE_PATCH = 1;

	pthread_mutex_init(&sensor_cache_patch_mutex, NULL); 
	sensor_cache_patch = (sensor_cache_patch_t*) malloc(sizeof(sensor_cache_patch_t));
	sensor_cache_patch->shut_down = 0;
	pthread_create( &sensor_thread_patch, NULL, sensorThreadFuncPatch, NULL);
	usleep(100000);//give sensor thread time to get valid readings.	
}

void *sensorThreadFuncPatch( void *ptr )
{
	int done = 0;
	while (!done) {
		
		pthread_mutex_lock( &sensor_cache_patch_mutex );
		sensor_cache_patch->wall_signal = readSensor(SENSOR_WALL_SIGNAL);		
		sensor_cache_patch->cliff_left_signal = readSensor(SENSOR_CLIFF_LEFT_SIGNAL);
		sensor_cache_patch->cliff_front_left_signal =  readSensor (SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
		sensor_cache_patch->cliff_front_right_signal =  readSensor (SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
		sensor_cache_patch->cliff_right_signal =  readSensor (SENSOR_CLIFF_RIGHT_SIGNAL);
		sensor_cache_patch->bumps_and_wheel_drops = readSensor (SENSOR_BUMPS_AND_WHEEL_DROPS);
		sensor_cache_patch->distance = readSensor (SENSOR_DISTANCE);
		sensor_cache_patch->angle = readSensor (SENSOR_ANGLE);
		done = sensor_cache_patch->shut_down;
		pthread_mutex_unlock( &sensor_cache_patch_mutex );
		
		
		usleep(CYCLE_TIME_PATCH);
	}
	pthread_exit(NULL);		
}


int getDistance_Patch()
{
	//printf("readSensor_Distance\n");
	if (THREAD_MODE_PATCH == 0) {
		
		return readSensor (SENSOR_DISTANCE);
	} 
	else {
		int dist;
		pthread_mutex_lock(&sensor_cache_patch_mutex);
		dist = sensor_cache_patch->distance;
		sensor_cache_patch->distance = 0;
		pthread_mutex_unlock(&sensor_cache_patch_mutex);
		return dist;
	}
}

int getAngle_Patch()
{
	//printf("readSensor_Angle\n");
	if (THREAD_MODE_PATCH == 0) {
		
		return readSensor (SENSOR_ANGLE);
	} 
	else {
		int angle;
		pthread_mutex_lock(&sensor_cache_patch_mutex);
		angle = sensor_cache_patch->angle;
		sensor_cache_patch->angle = 0;
		pthread_mutex_unlock(&sensor_cache_patch_mutex);
		return angle;
	}
}

int getBumpsAndWheelDrops_Patch ()
{
	//printf("readSensor_Bump\n");
	if (THREAD_MODE_PATCH == 0) {
		
		return readSensor (SENSOR_BUMPS_AND_WHEEL_DROPS);
	} 
	else {
		int bumps;
		pthread_mutex_lock(&sensor_cache_patch_mutex);
		bumps = sensor_cache_patch->bumps_and_wheel_drops;
		pthread_mutex_unlock(&sensor_cache_patch_mutex);
		return bumps;
	}
}


int getWallSignal()
{
	//printf("readSensor_Wall_Signal\n");
	if (THREAD_MODE_PATCH == 0) {
		
		return readSensor (SENSOR_WALL_SIGNAL);
	} 
	else {
		int wall_signal;
		pthread_mutex_lock(&sensor_cache_patch_mutex);
		wall_signal = sensor_cache_patch->wall_signal;
		pthread_mutex_unlock(&sensor_cache_patch_mutex);
		return wall_signal;
	}
}

int* getCliffSignals()
{
	int* cliff_signals = (int*)malloc (4*sizeof(int));
	if (NULL == cliff_signals)
	{
		fprintf (stderr, "Could not get cliff signals:  Memory allocation failed\n");
		return NULL;
	}
	memset (cliff_signals, 0, 4*sizeof(int));
	//printf("readSensor_Cliff_Signal\n");

	if (THREAD_MODE_PATCH == 0) {
		cliff_signals[0] = readSensor (SENSOR_CLIFF_LEFT_SIGNAL);
		cliff_signals[1] = readSensor (SENSOR_CLIFF_FRONT_LEFT_SIGNAL);
		cliff_signals[2] = readSensor (SENSOR_CLIFF_FRONT_RIGHT_SIGNAL);
		cliff_signals[3] = readSensor (SENSOR_CLIFF_RIGHT_SIGNAL);
	} 
	else {
		pthread_mutex_lock(&sensor_cache_patch_mutex);
		cliff_signals[0] = sensor_cache_patch->cliff_left_signal;
		cliff_signals[1] = sensor_cache_patch->cliff_front_left_signal;
		cliff_signals[2] = sensor_cache_patch->cliff_front_right_signal;
		cliff_signals[3] = sensor_cache_patch->cliff_right_signal;
		pthread_mutex_unlock(&sensor_cache_patch_mutex);
	}
	return cliff_signals;
}


int stopOI_MT_Patch ()
{
	pthread_mutex_lock( &sensor_cache_patch_mutex );
	sensor_cache_patch->shut_down = 1;
	pthread_mutex_unlock( &sensor_cache_patch_mutex );
	pthread_join(sensor_thread_patch, NULL);

	pthread_mutex_destroy(& sensor_cache_patch_mutex);
	
	if (stopOI()  !=0) 
		return -1;
	
	return 0;
}
