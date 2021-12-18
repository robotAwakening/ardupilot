-- perform simple aerobatic manoeuvres in AUTO mode

local running = false

local roll_stage = 0

local ROLL_TCONST = param:get('RLL2SRV_TCONST') * 0.5
local PITCH_TCONST = param:get('PTCH2SRV_TCONST') * 0.5

DO_JUMP = 177
NAV_WAYPOINT = 16

k_throttle = 70

function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

local SCR_USER1 = bind_param("SCR_USER1") -- height P gain
local SCR_USER2 = bind_param("SCR_USER2") -- height I gain
local SCR_USER3 = bind_param("SCR_USER3") -- throttle FF from pitch
local SCR_USER4 = bind_param("SCR_USER4") -- height knifeedge addition for pitch
local SCR_USER5 = bind_param("SCR_USER5") -- speed P gain
local SCR_USER6 = bind_param("SCR_USER6") -- speed I gain
local TRIM_THROTTLE = bind_param("TRIM_THROTTLE")
local TRIM_ARSPD_CM = bind_param("TRIM_ARSPD_CM")

local last_roll_err = 0.0
local last_id = 0
local initial_yaw_deg = 0
local wp_yaw_deg = 0
local initial_height = 0

-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

-- a controller to target a zero roll angle, coping with inverted flight
-- output is a body frame roll rate, with convergence over time tconst in seconds
function roll_zero_controller(tconst)
   local roll_deg = math.deg(ahrs:get_roll())
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_err = 0.0
   if math.abs(pitch_deg) > 85 then
      -- close to 90 we retain the last roll rate
      roll_err = last_roll_err
   elseif roll_deg > 90 then
      roll_err = 180 - roll_deg
   elseif roll_deg < -90 then
      roll_err = (-180) - roll_deg
   else
      roll_err = -roll_deg
   end
   last_roll_err = roll_err
   return roll_err / tconst
end


function wrap_360(angle)
   local res = math.fmod(angle, 360.0)
    if res < 0 then
        res = res + 360.0
    end
    return res
end

function wrap_180(angle)
    local res = wrap_360(angle)
    if res > 180 then
       res = res - 360
    end
    return res
end

function wrap_pi(angle_rad)
   return math.rad(wrap_180(math.deg(angle_rad)))
end

function convert_ef_rates_to_bf_rates(roll, pitch, yaw, ef_roll_rate, ef_pitch_rate, ef_yaw_rate)
   local cr = math.cos(roll)
   local sr = math.sin(roll)
   local cp = math.cos(pitch)
   local sp = math.sin(pitch) 
   local cy = math.cos(yaw)
   local sy = math.sin(yaw)
   
   local bf_roll_rate  =  ef_roll_rate                   -    sp*ef_yaw_rate
   local bf_pitch_rate =                cr*ef_pitch_rate + sr*cp*ef_yaw_rate
   local bf_yaw_rate   =               -sr*ef_pitch_rate + cr*cp*ef_yaw_rate

   return bf_roll_rate, bf_pitch_rate, bf_yaw_rate
end

-- a PI controller implemented as a Lua object
local function PI_controller(kP,kI,iMax)
   -- the new instance. You can put public variables inside this self
   -- declaration if you want to
   local self = {}

   -- private fields as locals
   local _kP = kP or 0.0
   local _kI = kI or 0.0
   local _kD = kD or 0.0
   local _iMax = iMax
   local _last_t = nil
   local _I = 0
   local _P = 0
   local _total = 0
   local _counter = 0
   local _target = 0
   local _current = 0

   -- update the controller.
   function self.update(target, current)
      local now = millis():tofloat() * 0.001
      if not _last_t then
         _last_t = now
      end
      local dt = now - _last_t
      _last_t = now
      local err = target - current
      _counter = _counter + 1

      local P = _kP * err
      _I = _I + _kI * err * dt
      if _iMax then
         _I = constrain(_I, -_iMax, iMax)
      end
      local I = _I
      local ret = P + I

      _target = target
      _current = current
      _P = P
      _total = ret
      return ret
   end

   -- reset integrator to an initial value
   function self.reset(integrator)
      _I = integrator
   end

   function self.set_I(I)
      _kI = I
   end

   function self.set_P(P)
      _kP = P
   end
   
   function self.set_Imax(Imax)
      _iMax = Imax
   end
   
   -- log the controller internals
   function self.log(name, add_total)
      -- allow for an external addition to total
      logger.write(name,'Targ,Curr,P,I,Total,Add','ffffff',_target,_current,_P,_I,_total,add_total)
   end

   -- return the instance
   return self
end

local function height_controller(kP_param,kI_param,KnifeEdge_param,Imax)
   local self = {}
   local kP = kP_param
   local kI = kI_param
   local KnifeEdge = KnifeEdge_param
   local PI = PI_controller(kP:get(), kI:get(), Imax)

   function self.update(target)
      local target_pitch = PI.update(initial_height, ahrs:get_position():alt()*0.01)
      local roll_rad = ahrs:get_roll()
      local ke_add = math.abs(math.sin(roll_rad)) * KnifeEdge:get()
      target_pitch = target_pitch + ke_add
      PI.log("HPI", ke_add)
      return target_pitch
   end

   function self.reset()
      PI.reset(math.max(math.deg(ahrs:get_pitch()), 3.0))
      PI.set_P(kP:get())
      PI.set_I(kI:get())
   end

   return self
end

local height_PI = height_controller(SCR_USER1, SCR_USER2, SCR_USER4, 20.0)
local speed_PI = PI_controller(SCR_USER5:get(), SCR_USER6:get(), 100.0)

-- a controller to target a zero pitch angle and zero heading change, used in a roll
-- output is a body frame pitch rate, with convergence over time tconst in seconds
function pitch_controller(target_pitch_deg, target_yaw_deg, tconst)
   local roll_deg = math.deg(ahrs:get_roll())
   local pitch_deg = math.deg(ahrs:get_pitch())
   local yaw_deg = math.deg(ahrs:get_yaw())

   -- get earth frame pitch and yaw rates
   local ef_pitch_rate = (target_pitch_deg - pitch_deg) / tconst
   local ef_yaw_rate = wrap_180(target_yaw_deg - yaw_deg) / tconst

   local bf_pitch_rate = math.sin(math.rad(roll_deg)) * ef_yaw_rate + math.cos(math.rad(roll_deg)) * ef_pitch_rate
   local bf_yaw_rate   = math.cos(math.rad(roll_deg)) * ef_yaw_rate - math.sin(math.rad(roll_deg)) * ef_pitch_rate
   return bf_pitch_rate, bf_yaw_rate
end

-- a controller for throttle to account for pitch
function throttle_controller(tconst)
   local pitch_rad = ahrs:get_pitch()
   local thr_ff = SCR_USER3:get()
   local throttle = TRIM_THROTTLE:get() + math.sin(pitch_rad) * thr_ff
   return constrain(throttle, 0.0, 100.0)
end

function do_axial_roll(arg1, arg2)
   -- constant roll rate axial roll
   if not running then
      running = true
      roll_stage = 0
      height_PI.reset()
      gcs:send_text(0, string.format("Starting roll"))
   end
   local roll_rate = arg1
   local throttle = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if roll_stage == 0 then
      if roll_deg > 45 then
         roll_stage = 1
      end
   elseif roll_stage == 1 then
      if roll_deg > -5 and roll_deg < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished roll r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_script_time_done(last_id)
         roll_stage = 2
         return
      end
   end
   if roll_stage < 2 then
      target_pitch = height_PI.update(initial_height)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wp_yaw_deg, PITCH_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, yaw_rate)
   end
end

local loop_stage = 0

function do_loop(arg1, arg2)
   -- do one loop with controllable pitch rate and throttle
   if not running then
      running = true
      loop_stage = 0
      gcs:send_text(0, string.format("Starting loop"))
   end
   local pitch_rate = arg1
   local throttle = throttle_controller()
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   if loop_stage == 0 then
      if pitch_deg > 60 then
         loop_stage = 1
      end
   elseif loop_stage == 1 then
      if math.abs(roll_deg) < 90 and pitch_deg > -5 and pitch_deg < 5 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished loop p=%.1f", pitch_deg))
         vehicle:nav_script_time_done(last_id)
         loop_stage = 2
         return
      end
   end
   if loop_stage < 2 then
      local roll_rate = roll_zero_controller(ROLL_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate, pitch_rate, 0)
   end
end

local rolling_circle_stage = 0
local rolling_circle_yaw = 0
local rolling_circle_last_ms = 0

function do_rolling_circle(arg1, arg2)
   -- constant roll rate circle roll
   if not running then
      running = true
      rolling_circle_stage = 0
      rolling_circle_yaw = 0
      rolling_circle_last_ms = millis()
      height_PI.reset()

      speed_PI.set_P(SCR_USER5:get())
      speed_PI.set_I(SCR_USER6:get())
      speed_PI.reset(math.max(SRV_Channels:get_output_scaled(k_throttle), TRIM_THROTTLE:get()))
      gcs:send_text(0, string.format("Starting rolling circle"))
   end
   local radius = arg1
   local num_rolls = arg2
   local pitch_deg = math.deg(ahrs:get_pitch())
   local roll_deg = math.deg(ahrs:get_roll())
   local yaw_deg = math.deg(ahrs:get_yaw())
   local gspeed = ahrs:groundspeed_vector():length()
   local circumference = math.abs(math.pi * 2.0 * radius)
   local circle_time = circumference / gspeed
   local yaw_rate_dps = 360.0 / circle_time
   local now_ms = millis()
   local dt = (now_ms - rolling_circle_last_ms):tofloat() * 0.001
   rolling_circle_last_ms = now_ms

   if radius < 0.0 then
      yaw_rate_dps = -yaw_rate_dps
   end

   local roll_rate = (360.0 * num_rolls) / circle_time

   rolling_circle_yaw = rolling_circle_yaw + yaw_rate_dps * dt

   if rolling_circle_stage == 0 then
      if math.abs(rolling_circle_yaw) > 10.0 then
         rolling_circle_stage = 1
      end
   elseif rolling_circle_stage == 1 then
      if math.abs(rolling_circle_yaw) >= 360.0 then
         running = false
         -- we're done
         gcs:send_text(0, string.format("Finished rollcircle r=%.1f p=%.1f", roll_deg, pitch_deg))
         vehicle:nav_script_time_done(last_id)
         rolling_circle_stage = 2
         return
      end
   end

   local target_roll = num_rolls * math.abs(rolling_circle_yaw)
   local roll_error = wrap_180(target_roll - roll_deg)
   local roll_error_P = 0.5
   local roll_rate_corrected = roll_rate + roll_error * roll_error_P

   if rolling_circle_stage < 2 then
      target_pitch = height_PI.update(initial_height)
      vel = ahrs:get_velocity_NED()
      throttle = speed_PI.update(TRIM_ARSPD_CM:get()*0.01, vel:length())
      throttle = constrain(throttle, 0, 100.0)
      --saturation_error = constrained_throttle - target_throttle
      --gcs:send_text(0, string.format("saturation_error: e=%.1f", saturation_error))
      speed_PI.log("SPI", 0.0)
      pitch_rate, yaw_rate = pitch_controller(target_pitch, wrap_360(rolling_circle_yaw+initial_yaw_deg), PITCH_TCONST)
      vehicle:set_target_throttle_rate_rpy(throttle, roll_rate_corrected, pitch_rate, yaw_rate)
   end
end

--- get a location object for a given WP number
function get_wp_location(i)
   local m = mission:get_item(i)
   local loc = Location()
   loc:lat(m:x())
   loc:lng(m:y())
   loc:relative_alt(false)
   loc:terrain_alt(false)
   loc:origin_alt(false)
   loc:alt(math.floor(m:z()*100))
   return loc
end

function resolve_jump(i)
   local m = mission:get_item(i)
   while m:command() == DO_JUMP do
      i = math.floor(m:param1())
      m = mission:get_item(i)
   end
   return i
end

function path_circle(t, radius, unused)
   t = t*math.pi*2
   --TODO: include * overload in lua bindings
   local vec = Vector3f()
   vec:x(1.0-math.cos(t))
   vec:y(-math.sin(t))
   return vec:scale(radius)
end

function path_figure_eight(t, radius, unused)
   t = t*math.pi*2
   local vec = Vector3f()
   vec:x(math.sin(t))
   vec:y(math.sin(t)*math.cos(t))
   return vec:scale(radius)
end

function path_length(path_f, arg1, arg2)
   local dt = 0.01
   local total = 0.0
   for i = 0, math.floor(1.0/dt) do
      local t = i*dt
      local t2 = t + dt
      local v1 = path_f(t, arg1, arg2)
      local v2 = path_f(t2, arg1, arg2)

      local dv = v2-v1
      total = total + dv:length()
   end
   return total
end

function rotate_position(position, yaw)
   local rotated_point = Vector3f()
   local cs = math.cos(yaw)
   local sn = math.sin(yaw)

   rotated_point:x(position:x() * cs - position:y() * sn)
   rotated_point:y(position:x() * sn + position:y() * cs)
   return rotated_point
end

function rotate_path(path_f, t, arg1, arg2, yaw)
   return rotate_position(path_f(t, arg1, arg2), yaw)
end

local path_var = {}
path_var.count = 0

function do_path(path, arg1, arg2)

   local now = millis():tofloat() * 0.001

   path_var.count = path_var.count + 1

   if not running then
      running = true
      path_var.length = path_length(path, arg1, arg2)
      local speed = TRIM_ARSPD_CM:get()*0.01

      --assuming constant velocity
      path_var.total_time = path_var.length/speed
      path_var.start_time = now
      path_var.last_time = now
      path_var.last_pos = path(0.0, arg1, arg2)
      local path_delta = path(0.001, arg1, arg2)
      local init_tangent = path_delta - path_var.last_pos
      path_var.init_yaw = wrap_pi(math.atan(init_tangent:y(), init_tangent:x()))
      path_var.init_yaw = -wrap_pi(path_var.init_yaw - math.rad(wp_yaw_deg))
      
      corrected_position_t0 = rotate_path(path, 0.0, arg1, arg2, path_var.init_yaw)
      corrected_position_t1 = rotate_path(path, 0.001, arg1, arg2, path_var.init_yaw)

      path_var.prev_target_pos = corrected_position_t0

      local tangent = corrected_position_t1 - corrected_position_t0
      local corrected_yaw = wrap_pi(math.atan(tangent:y(), tangent:x()))

      path_var.initial_ef_pos = ahrs:get_relative_position_NED_origin()

      height_PI.reset()

      speed_PI.set_P(SCR_USER5:get())
      speed_PI.set_I(SCR_USER6:get())
      speed_PI.reset(math.max(SRV_Channels:get_output_scaled(k_throttle), TRIM_THROTTLE:get()))
   end

   local dt = now - path_var.last_time

   --normalise current time to [0, 1]
   local t = (now - path_var.start_time)/path_var.total_time

   --TODO: add time correction

   --TODO: Fix this exit condition
   if t > 1.0 then --done
      running = false
      vehicle:nav_script_time_done(last_id)
      return
   end

   --where we aim to be on the path at this timestamp
   local current_target_pos = rotate_path(path, t, arg1, arg2, path_var.init_yaw)
   local current_measured_pos = ahrs:get_relative_position_NED_origin() - path_var.initial_ef_pos
   local position_error = current_target_pos - current_measured_pos
   --arbitrary time constant larger than dt over which to correct the position error
   local correction_time = 2.0
   --velocity required to correct current position error
   local position_error_velocity = (position_error):scale(1.0/correction_time)
   --velocity required to travel along trajectory
   local trajectory_velocity = (current_target_pos - path_var.prev_target_pos):scale(1.0/dt)
   local target_velocity = position_error_velocity + trajectory_velocity

   path_var.prev_target_pos = current_target_pos
   path_var.last_measured_pos = current_pos
   path_var.last_time = now

   local target_yaw_rad = wrap_pi(math.atan(target_velocity:y(), target_velocity:x()))   
   local current_yaw_rad = wrap_pi(ahrs:get_yaw())
   local yaw_error = wrap_pi(target_yaw_rad - current_yaw_rad)
   local yaw_tconst = dt
   local ef_yaw_rate = yaw_error/yaw_tconst  

   local target_pitch_rad = wrap_pi(math.atan(-target_velocity:z(), math.sqrt(target_velocity:x()^2 + target_velocity:y()^2)))
   local current_pitch_rad = wrap_pi(ahrs:get_pitch())
   local pitch_error = wrap_pi(target_pitch_rad - current_pitch_rad)
   local pitch_tconst = dt
   local ef_pitch_rate = pitch_error/pitch_tconst

   --assume roll 0 for now, should work for 2D paths like circle and figure eight
   local target_roll_rad = 0.0
   local current_roll_rad = wrap_pi(ahrs:get_roll())
   local roll_error = wrap_pi(target_roll_rad - current_roll_rad)
   local roll_tconst = dt
   local ef_roll_rate = roll_error/roll_tconst

   --convert ef rates to bf rates, I assume there is a function we can bind in Ardupilot to do this already, but couldn't find it.
   bf_roll_rate, bf_pitch_rate, bf_yaw_rate = convert_ef_rates_to_bf_rates(current_roll_rad,
                                                                        current_pitch_rad,
                                                                        current_yaw_rad,
                                                                        ef_roll_rate, 
                                                                        ef_pitch_rate, 
                                                                        ef_yaw_rate)

   vel = ahrs:get_velocity_NED()
   local target_speed = TRIM_ARSPD_CM:get()*0.01
   throttle = speed_PI.update(target_speed, vel:length())
   throttle = constrain(throttle, 0, 100.0)
   vehicle:set_target_throttle_rate_rpy(throttle, bf_roll_rate, bf_pitch_rate, bf_yaw_rate)
end

function update()
   id, cmd, arg1, arg2 = vehicle:nav_script_time()
   if id then
      if id ~= last_id then
         -- we've started a new command
         running = false
         last_id = id
         initial_yaw_deg = math.deg(ahrs:get_yaw())
         initial_height = ahrs:get_position():alt()*0.01

         -- work out yaw between previous WP and next WP
         local cnum = mission:get_current_nav_index()
         local loc_prev = get_wp_location(cnum-1)
         local loc_next = get_wp_location(resolve_jump(cnum+1))
         wp_yaw_deg = math.deg(loc_prev:get_bearing(loc_next))
      end
      if cmd == 1 then
         do_axial_roll(arg1, arg2)
      elseif cmd == 2 then
         do_loop(arg1, arg2)
      elseif cmd == 3 then
         do_rolling_circle(arg1, arg2)
      elseif cmd == 4 then
         do_path(path_circle, arg1, arg2)
      elseif cmd == 5 then
         do_path(path_figure_eight, arg1, arg2)
      end
   else
      running = false
   end
   return update, 10
end

return update()
