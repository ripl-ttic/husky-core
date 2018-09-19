/*********************************************************
 *
 * This source code is part of the Carnegie Mellon Robot
 * Navigation Toolkit (CARMEN)
 *
 * CARMEN Copyright (c) 2002 Michael Montemerlo, Nicholas
 * Roy, Sebastian Thrun, Dirk Haehnel, Cyrill Stachniss,
 * and Jared Glover
 *
 * CARMEN is free software; you can redistribute it and/or 
 * modify it under the terms of the GNU General Public 
 * License as published by the Free Software Foundation; 
 * either version 2 of the License, or (at your option)
 * any later version.
 *
 * CARMEN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied 
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more 
 * details.
 *
 * You should have received a copy of the GNU General 
 * Public License along with CARMEN; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, 
 * Suite 330, Boston, MA  02111-1307 USA
 *
 ********************************************************/

/* Standard include file */

#ifndef ER_CARMEN_H
#define ER_CARMEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#ifndef va_copy
#define va_copy __va_copy
#endif 

#include <unistd.h>
#include <ctype.h>
#ifdef __USE_BSD
#undef __USE_BSD
#include <string.h>
#define __USE_BSD
#else
#include <string.h>
#endif
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#ifdef CYGWIN
#include <sys/socket.h>
#endif
#include <errno.h>
#include <limits.h>
#include <float.h>
#ifndef MAXDOUBLE
#define MAXDOUBLE DBL_MAX
#endif
#ifndef MAXFLOAT
#define MAXFLOAT FLT_MAX
#endif

#include <carmen_utils/global.h>
    
#include <carmen_utils/laser_messages.h>    
#include <carmen_utils/map.h>
#include <carmen_utils/map_messages.h>
#include <carmen_utils/map_interface.h>

#include <carmen_utils/geometry.h>
#include <carmen_utils/navigator_messages.h>
#include <carmen_utils/robot_messages.h>
#include <carmen_utils/base_messages.h>
#include <carmen_utils/map_io.h>

#ifdef __cplusplus
}
#endif

#endif
