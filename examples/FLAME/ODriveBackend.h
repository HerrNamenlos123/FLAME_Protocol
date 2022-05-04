
#ifndef ODRIVE_BACKEND_H
#define ODRIVE_BACKEND_H

#include "ODrive.h"
#include "ODriveProperties.h"

extern ODriveProperties odrvData0;
extern ODriveProperties odrvData1;

void setupODrives();
void updateODrives();

#endif // ODRIVE_BACKEND_H
