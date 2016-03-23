#pragma once

// this class is #included into the Copter:: namespace

class FlightMode {
    friend class Copter;
    friend class AP_Arming_Copter;

public:

    FlightMode(Copter &copter) :
        _copter(copter),
        g(copter.g),
        g2(copter.g2),
        wp_nav(_copter.wp_nav),
        pos_control(_copter.pos_control),
        inertial_nav(_copter.inertial_nav),
        ahrs(_copter.ahrs),
        attitude_control(_copter.attitude_control),
        motors(_copter.motors),
        channel_roll(_copter.channel_roll),
        channel_pitch(_copter.channel_pitch),
        channel_throttle(_copter.channel_throttle),
        channel_yaw(_copter.channel_yaw),
        G_Dt(_copter.G_Dt),
        ap(_copter.ap),
        takeoff_state(_copter.takeoff_state),
        ekfGndSpdLimit(_copter.ekfGndSpdLimit),
        ekfNavVelGainScaler(_copter.ekfNavVelGainScaler)
        { };

protected:

    virtual bool init(bool ignore_checks) = 0; // should be called at 100hz or more
    virtual void run() = 0; // should be called at 100hz or more

    virtual bool is_autopilot() const { return false; }
    virtual bool requires_GPS() const = 0;
    virtual bool has_manual_throttle() const = 0;
    virtual bool allows_arming(bool from_gcs) const = 0;
    void print_FlightMode(AP_HAL::BetterStream *port) const {
        port->print(name());
    }
    virtual const char *name() const = 0;

    // returns a string for this flightmode, exactly 4 bytes
    virtual const char *name4() const = 0;

    Copter &_copter;

    // convenience references to avoid code churn in conversion:
    Parameters &g;
    ParametersG2 &g2;
    AC_WPNav *&wp_nav;
    AC_PosControl *&pos_control;
    AP_InertialNav &inertial_nav;
    AP_AHRS &ahrs;
    AC_AttitudeControl_t *&attitude_control;
    MOTOR_CLASS *&motors;
    RC_Channel *&channel_roll;
    RC_Channel *&channel_pitch;
    RC_Channel *&channel_throttle;
    RC_Channel *&channel_yaw;
    float &G_Dt;
    ap_t &ap;
    takeoff_state_t &takeoff_state;

    // gnd speed limit required to observe optical flow sensor limits
    float &ekfGndSpdLimit;

    // scale factor applied to velocity controller gain to prevent optical flow noise causing excessive angle demand noise
    float &ekfNavVelGainScaler;

    // pass-through functions to reduce code churn on conversion;
    // these are candidates for moving into the FlightMode base
    // class.
    virtual void get_pilot_desired_lean_angles(float roll_in, float pitch_in, float &roll_out, float &pitch_out, float angle_max) {
        _copter.get_pilot_desired_lean_angles(roll_in, pitch_in, roll_out, pitch_out, angle_max);
    }
    virtual float get_surface_tracking_climb_rate(int16_t target_rate, float current_alt_target, float dt) {
        return _copter.get_surface_tracking_climb_rate(target_rate, current_alt_target, dt);
    }
    virtual float get_pilot_desired_yaw_rate(int16_t stick_angle) {
        return _copter.get_pilot_desired_yaw_rate(stick_angle);
    }
    virtual float get_pilot_desired_climb_rate(float throttle_control) {
        return _copter.get_pilot_desired_climb_rate(throttle_control);
    }
    virtual float get_pilot_desired_throttle(int16_t throttle_control, float thr_mid = 0.0f) {
        return _copter.get_pilot_desired_throttle(throttle_control, thr_mid);
    }
    virtual float get_non_takeoff_throttle() {
        return _copter.get_non_takeoff_throttle();
    }
    virtual void update_simple_mode(void) {
        _copter.update_simple_mode();
    }
    virtual float get_smoothing_gain() {
        return _copter.get_smoothing_gain();
    }
    virtual bool set_mode(control_mode_t mode, mode_reason_t reason) {
        return _copter.set_mode(mode, reason);
    }
    virtual void set_land_complete(bool b) {
        return _copter.set_land_complete(b);
    }
    virtual void gcs_send_text(MAV_SEVERITY severity, const char *str) {
        return _copter.gcs_send_text(severity, str);
    }
    virtual void Log_Write_Event(uint8_t id) {
        return _copter.Log_Write_Event(id);
    }
    virtual void set_throttle_takeoff() {
        return _copter.set_throttle_takeoff();
    }
    virtual void set_auto_yaw_mode(uint8_t yaw_mode) {
        return _copter.set_auto_yaw_mode(yaw_mode);
    }
    virtual void takeoff_timer_start(float alt_cm) {
        return _copter.takeoff_timer_start(alt_cm);
    }
    virtual void takeoff_stop() {
        return _copter.takeoff_stop();
    }
    virtual void takeoff_get_climb_rates(float& pilot_climb_rate, float& takeoff_climb_rate) {
        return _copter.takeoff_get_climb_rates(pilot_climb_rate, takeoff_climb_rate);
    }
    float get_auto_heading() {
        return _copter.get_auto_heading();
    }
    float get_avoidance_adjusted_climbrate(float target_rate) {
        return _copter.get_avoidance_adjusted_climbrate(target_rate);
    }

    // end pass-through functions
};


class FlightMode_ACRO : public FlightMode {

public:

    FlightMode_ACRO(Copter &copter) :
        Copter::FlightMode(copter)
        { }
    virtual bool init(bool ignore_checks) override;
    virtual void run() override; // should be called at 100hz or more

    virtual bool is_autopilot() const override { return false; }
    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return true; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };

protected:

    const char *name() const override { return "ACRO"; }
    const char *name4() const override { return "ACRO"; }

    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

private:

};

#if FRAME_CONFIG == HELI_FRAME
class FlightMode_ACRO_Heli : public FlightMode_ACRO {

public:

    FlightMode_ACRO_Heli(Copter &copter) :
        Copter::FlightMode_ACRO(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    void get_pilot_desired_yaw_rate(int16_t yaw_in, float &yaw_out);

protected:
private:
};
#endif



class FlightMode_ALTHOLD : public FlightMode {

public:

    FlightMode_ALTHOLD(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "ALT_HOLD"; }
    const char *name4() const override { return "ALTH"; }

private:

};



class FlightMode_STABILIZE : public FlightMode {

public:

    FlightMode_STABILIZE(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override; // should be called at 100hz or more

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return true; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "STABILIZE"; }
    const char *name4() const override { return "STAB"; }

private:

};

#if FRAME_CONFIG == HELI_FRAME
class FlightMode_STABILIZE_Heli : public FlightMode_STABILIZE {

public:

    FlightMode_STABILIZE_Heli(Copter &copter) :
        Copter::FlightMode_STABILIZE(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

protected:

private:

};
#endif



class FlightMode_AUTO : public FlightMode {

public:

    FlightMode_AUTO(Copter &copter, AP_Mission &_mission, AC_Circle *& _circle_nav) :
        Copter::FlightMode(copter),
        mission(_mission),
        circle_nav(_circle_nav)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override; // should be called at 100hz or more

    virtual bool is_autopilot() const override { return true; }
    virtual bool requires_GPS() const override { return true; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return false; };

    // Auto
    AutoMode mode() { return _mode; }

    bool loiter_start();
    void rtl_start();
    void takeoff_start(const Location& dest_loc);
    void wp_start(const Vector3f& destination);
    void wp_start(const Location_Class& dest_loc);
    void land_start();
    void land_start(const Vector3f& destination);
    void circle_movetoedge_start(const Location_Class &circle_center, float radius_m);
    void circle_start();
    void spline_start(const Vector3f& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Vector3f& next_spline_destination);
    void spline_start(const Location_Class& destination, bool stopped_at_start, AC_WPNav::spline_segment_end_type seg_end_type, const Location_Class& next_destination);
    void nav_guided_start();

    bool landing_gear_should_be_deployed();

    void payload_place_start();

protected:

    const char *name() const override { return "AUTO"; }
    const char *name4() const override { return "AUTO"; }

//    void get_pilot_desired_angle_rates(int16_t roll_in, int16_t pitch_in, int16_t yaw_in, float &roll_out, float &pitch_out, float &yaw_out);

private:

    void takeoff_run();
    void wp_run();
    void spline_run();
    void land_run();
    void rtl_run();
    void circle_run();
    void nav_guided_run();
    void loiter_run();

    void payload_place_start(const Vector3f& destination);
    void payload_place_run();
    bool payload_place_run_should_run();
    void payload_place_run_loiter();
    void payload_place_run_descend();
    void payload_place_run_release();

    AutoMode _mode = Auto_TakeOff;   // controls which auto controller is run

    AP_Mission &mission;
    AC_Circle *&circle_nav;
};



class FlightMode_CIRCLE : public FlightMode {

public:

    FlightMode_CIRCLE(Copter &copter, AC_Circle *& _circle_nav) :
        Copter::FlightMode(copter),
        circle_nav(_circle_nav)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

protected:

    const char *name() const override { return "CIRCLE"; }
    const char *name4() const override { return "CIRC"; }

private:

    // Circle
    bool pilot_yaw_override = false; // true if pilot is overriding yaw
    AC_Circle *&circle_nav;

};



class FlightMode_LOITER : public FlightMode {

public:

    FlightMode_LOITER(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

#if PRECISION_LANDING == ENABLED
    void set_precision_loiter_enabled(bool value) { _precision_loiter_enabled = value; }
#endif

protected:

    const char *name() const override { return "LOITER"; }
    const char *name4() const override { return "LOIT"; }

#if PRECISION_LANDING == ENABLED
    bool do_precision_loiter() const;
    void precision_loiter_xy();
#endif

private:

#if PRECISION_LANDING == ENABLED
    bool _precision_loiter_enabled;
#endif

};



class FlightMode_GUIDED : public FlightMode {

public:

    FlightMode_GUIDED(Copter &copter) :
        Copter::FlightMode(copter)        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override {
        if (from_gcs) {
            return true;
        }
        return false;
    };
    bool is_autopilot() const override { return true; }

    void set_angle(const Quaternion &q, float climb_rate_cms, bool use_yaw_rate, float yaw_rate_rads);
    void set_destination_posvel(const Vector3f& destination, const Vector3f& velocity);
    void set_velocity(const Vector3f& velocity);
    bool set_destination(const Vector3f& destination);
    bool set_destination(const Location_Class& dest_loc);

    void limit_clear();
    void limit_init_time_and_pos();
    void limit_set(uint32_t timeout_ms, float alt_min_cm, float alt_max_cm, float horiz_max_cm);
    bool limit_check();

    bool takeoff_start(float final_alt_above_home);

    GuidedMode mode() { return guided_mode; }

    void angle_control_start();
    void angle_control_run();

protected:

    const char *name() const override { return "GUIDED"; }
    const char *name4() const override { return "GUID"; }

private:

    void pos_control_start();
    void vel_control_start();
    void posvel_control_start();
    void takeoff_run();
    void pos_control_run();
    void vel_control_run();
    void posvel_control_run();
    void set_desired_velocity_with_accel_and_fence_limits(const Vector3f& vel_des);

    // controls which controller is run (pos or vel):
    GuidedMode guided_mode = Guided_TakeOff;

};



class FlightMode_LAND : public FlightMode {

public:

    FlightMode_LAND(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return true; }

    float get_land_descent_speed();
    bool landing_with_GPS();
    void do_not_use_GPS();

    int32_t get_alt_above_ground(void);

protected:

    const char *name() const override { return "LAND"; }
    const char *name4() const override { return "LAND"; }

private:

    void gps_run();
    void nogps_run();
};



class FlightMode_RTL : public FlightMode {

public:

    FlightMode_RTL(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return true; }

    RTLState state() { return _state; }

    // this should probably not be exposed
    bool state_complete() { return _state_complete; }

    bool landing_gear_should_be_deployed();

    void restart_without_terrain();

protected:

    const char *name() const override { return "RTL"; }
    const char *name4() const override { return "RTL "; }

private:

    void climb_start();
    void return_start();
    void climb_return_run();
    void loiterathome_start();
    void loiterathome_run();
    void descent_start();
    void descent_run();
    void land_start();
    void land_run();
    void build_path(bool terrain_following_allowed);
    void compute_return_target(bool terrain_following_allowed);

    // RTL
    RTLState _state = RTL_InitialClimb;  // records state of rtl (initial climb, returning home, etc)
    bool _state_complete = false; // set to true if the current state is completed

    struct {
        // NEU w/ Z element alt-above-ekf-origin unless use_terrain is true in which case Z element is alt-above-terrain
        Location_Class origin_point;
        Location_Class climb_target;
        Location_Class return_target;
        Location_Class descent_target;
        bool land;
        bool terrain_used;
    } rtl_path;

    // Loiter timer - Records how long we have been in loiter
    uint32_t _loiter_start_time = 0;
};



class FlightMode_DRIFT : public FlightMode {

public:

    FlightMode_DRIFT(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override; // should be called at 100hz or more

    virtual bool requires_GPS() const override { return true; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "DRIFT"; }
    const char *name4() const override { return "DRIF"; }

private:

    float get_throttle_assist(float velz, float pilot_throttle_scaled);

};



class FlightMode_SPORT : public FlightMode {

public:

    FlightMode_SPORT(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override; // should be called at 100hz or more

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return true; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "SPORT"; }
    const char *name4() const override { return "SPRT"; }

private:

};



class FlightMode_FLIP : public FlightMode {

public:

    FlightMode_FLIP(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    virtual bool init(bool ignore_checks) override;
    virtual void run() override; // should be called at 100hz or more

    virtual bool requires_GPS() const override { return false; }
    virtual bool has_manual_throttle() const override { return false; }
    virtual bool allows_arming(bool from_gcs) const override { return false; };
    virtual bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "FLIP"; }
    const char *name4() const override { return "FLIP"; }

private:

    // Flip
    Vector3f flip_orig_attitude;         // original copter attitude before flip

};



#if AUTOTUNE_ENABLED == ENABLED
class FlightMode_AUTOTUNE : public FlightMode {

public:

    FlightMode_AUTOTUNE(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return false; }

    float get_autotune_descent_speed();
    bool autotuneing_with_GPS();
    void do_not_use_GPS();

    void autotune_stop();
    void autotune_save_tuning_gains();

protected:

    const char *name() const override { return "AUTOTUNE"; }
    const char *name4() const override { return "ATUN"; }

private:

    bool autotune_start(bool ignore_checks);
    void autotune_attitude_control();
    void autotune_backup_gains_and_initialise();
    void autotune_load_orig_gains();
    void autotune_load_tuned_gains();
    void autotune_load_intra_test_gains();
    void autotune_load_twitch_gains();
    void autotune_update_gcs(uint8_t message_id);
    bool autotune_roll_enabled();
    bool autotune_pitch_enabled();
    bool autotune_yaw_enabled();
    void autotune_twitching_test(float measurement, float target, float &measurement_min, float &measurement_max);
    void autotune_updating_d_up(float &tune_d, float tune_d_min, float tune_d_max, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_updating_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_updating_p_down(float &tune_p, float tune_p_min, float tune_p_step_ratio, float target, float measurement_max);
    void autotune_updating_p_up(float &tune_p, float tune_p_max, float tune_p_step_ratio, float target, float measurement_max);
    void autotune_updating_p_up_d_down(float &tune_d, float tune_d_min, float tune_d_step_ratio, float &tune_p, float tune_p_min, float tune_p_max, float tune_p_step_ratio, float target, float measurement_min, float measurement_max);
    void autotune_twitching_measure_acceleration(float &rate_of_change, float rate_measurement, float &rate_measurement_max);
    void autotune_get_poshold_attitude(float &roll_cd, float &pitch_cd, float &yaw_cd);

    void Log_Write_AutoTune(uint8_t axis, uint8_t tune_step, float meas_target, float meas_min, float meas_max, float new_gain_rp, float new_gain_rd, float new_gain_sp, float new_ddt);
    void Log_Write_AutoTuneDetails(float angle_cd, float rate_cds);
};
#endif



#if POSHOLD_ENABLED == ENABLED
class FlightMode_POSHOLD : public FlightMode {

public:

    FlightMode_POSHOLD(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return true; };
    bool is_autopilot() const override { return false; }

protected:

    const char *name() const override { return "POSHOLD"; }
    const char *name4() const override { return "PHLD"; }

private:

    void poshold_update_pilot_lean_angle(float &lean_angle_filtered, float &lean_angle_raw);
    int16_t poshold_mix_controls(float mix_ratio, int16_t first_control, int16_t second_control);
    void poshold_update_brake_angle_from_velocity(int16_t &brake_angle, float velocity);
    void poshold_update_wind_comp_estimate();
    void poshold_get_wind_comp_lean_angles(int16_t &roll_angle, int16_t &pitch_angle);
    void poshold_roll_controller_to_pilot_override();
    void poshold_pitch_controller_to_pilot_override();

};
#endif



class FlightMode_BRAKE : public FlightMode {

public:

    FlightMode_BRAKE(Copter &copter) :
        Copter::FlightMode(copter)
        { }

    bool init(bool ignore_checks) override;
    void run() override; // should be called at 100hz or more

    bool requires_GPS() const override { return true; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(bool from_gcs) const override { return false; };
    bool is_autopilot() const override { return false; }

    void timeout_to_loiter_ms(uint32_t timeout_ms);

protected:

    const char *name() const override { return "BRAKE"; }
    const char *name4() const override { return "BRAK"; }

private:

    uint32_t _timeout_start;
    uint32_t _timeout_ms;

};
