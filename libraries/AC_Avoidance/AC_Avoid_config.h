
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AC_AVOID_ENABLED
#define AC_AVOID_ENABLED 1
#endif

#ifndef AP_OAPATHPLANNER_ENABLED
#define AP_OAPATHPLANNER_ENABLED AC_AVOID_ENABLED
#endif

#ifndef AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED
#define AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED AP_OAPATHPLANNER_ENABLED
#endif

#ifndef AP_OAPATHPLANNER_BENDYRULER_ENABLED
#define AP_OAPATHPLANNER_BENDYRULER_ENABLED  AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED
#endif

#ifndef AP_OAPATHPLANNER_DIJKSTRA_ENABLED
#define AP_OAPATHPLANNER_DIJKSTRA_ENABLED  AP_OAPATHPLANNER_BACKEND_DEFAULT_ENABLED
#endif
