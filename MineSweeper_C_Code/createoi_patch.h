#ifndef H_CREATEOI_PATCH_GD
#define H_CREATEOI_PATCH_GD

#include <unistd.h>
#include "createoi.h"

#ifdef __cplusplus
extern "C" {
#endif

int startOI_MT_Patch(const char* serial);
int getWallSignal();
int getDistance_Patch();
int getAngle_Patch();
int getBumpsAndWheelDrops_Patch ();
int* getCliffSignals();
int stopOI_MT_Patch ();





#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif



#endif //H_CREATEOI_GD
