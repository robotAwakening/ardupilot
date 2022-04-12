/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Proximity_Utils.h"

#if HAL_PROXIMITY_ENABLED

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AC_Avoidance/AP_OADatabase.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;


// check if a reading should be ignored because it falls into an ignore area (check_for_ign_area should be sent as false if this check is not needed)
// pitch is the vertical body-frame angle (in degrees) to the obstacle (0=directly ahead, 90 is above the vehicle)
// yaw is the horizontal body-frame angle (in degrees) to the obstacle (0=directly ahead of the vehicle, 90 is to the right of the vehicle)
// Also checks if obstacle is near land or out of range
// angles should be in degrees and in the range of 0 to 360, distance should be in meteres
bool AP_Proximity_Utils::ignore_reading(float pitch, float yaw, float distance_m, bool check_for_ign_area, float max_range_m, float min_range_m) const
{
    // check if distances are supposed to be in a particular range
    if (!is_zero(max_range_m)) {
        if (distance_m > max_range_m) {
            // too far away
            return true;
        }
    }

    if (!is_zero(min_range_m)) {
        if (distance_m < min_range_m) {
            // too close
            return true;
        }
    }

   // check if obstacle is near land
   return check_obstacle_near_ground(pitch, yaw, distance_m);
}

// store rangefinder values
void AP_Proximity_Utils::set_rangefinder_alt(bool use, bool healthy, float alt_cm)
{
    _last_downward_update_ms = AP_HAL::millis();
    _rangefinder_use = use;
    _rangefinder_healthy = healthy;
    _rangefinder_alt = alt_cm * 0.01f;
}

// get alt from rangefinder in meters
bool AP_Proximity_Utils::get_rangefinder_alt(float &alt_m) const
{
    if (!_rangefinder_use || !_rangefinder_healthy) {
        // range finder is not healthy
        return false;
    }

    const uint32_t dt = AP_HAL::millis() - _last_downward_update_ms;
    if (dt > PROXIMITY_ALT_DETECT_TIMEOUT_MS) {
        return false;
    }

    // readings are healthy
    alt_m = _rangefinder_alt;
    return true;
}

// Check if Obstacle defined by body-frame yaw and pitch is near ground
bool AP_Proximity_Utils::check_obstacle_near_ground(float pitch, float yaw, float distance) const
{
    if (!hal.util->get_soft_armed()) {
        // don't run this feature while vehicle is disarmed, otherwise proximity data will not show up on GCS
        return false;
    }
    if ((pitch > 90.0f) || (pitch < -90.0f)) {
        // sanity check on pitch
        return false;
    }
    // Assume object is yaw and pitch bearing and distance meters away from the vehicle
    Vector3f object_3D;
    object_3D.offset_bearing(wrap_180(yaw), (pitch * -1.0f), distance);
    const Matrix3f body_to_ned = AP::ahrs().get_rotation_body_to_ned();
    const Vector3f rotated_object_3D = body_to_ned * object_3D;

    float alt = FLT_MAX;
    if (!get_rangefinder_alt(alt)) {
        return false;
    }

    if (rotated_object_3D.z > -0.5f) {
        // obstacle is at the most 0.5 meters above vehicle
        if ((alt - PROXIMITY_GND_DETECT_THRESHOLD) < rotated_object_3D.z) {
            // obstacle is near or below ground
            return true;
        }
    }
    return false;
}

// returns true if database is ready to be pushed to and all cached data is ready
bool AP_Proximity_Utils::database_prepare_for_push(Vector3f &current_pos, Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return false;
    }

    if (!AP::ahrs().get_relative_position_NED_origin(current_pos)) {
        return false;
    }

    body_to_ned = AP::ahrs().get_rotation_body_to_ned();

    return true;
}

// update Object Avoidance database with Earth-frame point
void AP_Proximity_Utils::database_push(float angle, float distance)
{
    Vector3f current_pos;
    Matrix3f body_to_ned;

    if (database_prepare_for_push(current_pos, body_to_ned)) {
        database_push(angle, distance, AP_HAL::millis(), current_pos, body_to_ned);
    }
}

// update Object Avoidance database with Earth-frame point
// pitch can be optionally provided if needed
void AP_Proximity_Utils::database_push(float angle, float pitch, float distance, uint32_t timestamp_ms, const Vector3f &current_pos, const Matrix3f &body_to_ned)
{
    AP_OADatabase *oaDb = AP::oadatabase();
    if (oaDb == nullptr || !oaDb->healthy()) {
        return;
    }
    if ((pitch > 90.0f) || (pitch < -90.0f)) {
        // sanity check on pitch
        return;
    }
    //Assume object is angle and pitch bearing and distance meters away from the vehicle 
    Vector3f object_3D;
    object_3D.offset_bearing(wrap_180(angle), (pitch * -1.0f), distance);
    const Vector3f rotated_object_3D = body_to_ned * object_3D;

    //Calculate the position vector from origin
    Vector3f temp_pos = current_pos + rotated_object_3D;
    //Convert the vector to a NEU frame from NED
    temp_pos.z = temp_pos.z * -1.0f;

    oaDb->queue_push(temp_pos, timestamp_ms, distance);
}

#endif // HAL_PROXIMITY_ENABLED

