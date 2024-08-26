#ifndef APPLY_XBEE_PROFILE_H
#define APPLY_XBEE_PROFILE_H

#include "xbee/device.h"
#include "xbee/atcmd.h"
#include "xbee/wpan.h"

size_t chomp(char *line);
int apply_profile(xbee_dev_t *xbee, const char *filename);

#endif