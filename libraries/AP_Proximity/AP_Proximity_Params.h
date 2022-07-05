#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>

#define PROXIMITY_MAX_IGNORE                6   // up to six areas can be ignored

class AP_Proximity_Params {

public:

    static const struct AP_Param::GroupInfo var_info[];

    AP_Proximity_Params(void);

    /* Do not allow copies */
    AP_Proximity_Params(const AP_Proximity_Params &other) = delete;
    AP_Proximity_Params &operator=(const AP_Proximity_Params&) = delete;

    AP_Int8 type;                                       // type of sensor
    AP_Int8 orientation;                                // orientation (e.g. right-side-up or upside-down)
    AP_Int16 yaw_correction;                            // yaw correction in degrees
    AP_Int16 ignore_angle_deg[PROXIMITY_MAX_IGNORE];    // angle (in degrees) of area that should be ignored by sensor (i.e. leg shows up)
    AP_Int8 ignore_width_deg[PROXIMITY_MAX_IGNORE];     // width of beam (in degrees) that should be ignored
    AP_Float max_m;                                     // maximum range in meters
    AP_Float min_m;                                     // minimum range in meters
};
