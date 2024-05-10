local PARAM_TABLE_KEY = 7
local PARAM_TABLE_PREFIX = "SHIP_"

local MODE_MANUAL = 0
local MODE_RTL = 11
local MODE_AUTO = 10
local MODE_GUIDED = 15
local MODE_CRUISE = 7

local ALT_FRAME_ABSOLUTE = 0

-- 3 throttle position
local THROTTLE_LOW = 0
local THROTTLE_MID = 1
local THROTTLE_HIGH = 2

-- bind a parameter to a variable
function bind_param(name)
    local p = Parameter()
    assert(p:init(name), string.format('could not find %s parameter', name))
    return p
 end
 
 -- add a parameter and bind it to a variable
 function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return bind_param(PARAM_TABLE_PREFIX .. name)
 end
 
 -- setup SHIP specific parameters
 assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 3), 'could not add param table')
 --[[
   // @Param: SHIP_ENABLE
   // @DisplayName: Ship landing enable
   // @Description: Enable ship landing system
   // @Values: 0:Disabled,1:Enabled
   // @User: Standard
 --]]
 SHIP_ENABLE     = bind_add_param('ENABLE', 1, 0)
 
 --[[
   // @Param: SHIP_LAND_ANGLE
   // @DisplayName: Ship landing angle
   // @Description: Angle from the stern of the ship for landing approach. Use this to ensure that on a go-around that ship superstructure and cables are avoided. A value of zero means to approach from the rear of the ship. A value of 90 means the landing will approach from the port (left) side of the ship. A value of -90 will mean approaching from the starboard (right) side of the ship. A value of 180 will approach from the bow of the ship. This parameter is combined with the sign of the RTL_RADIUS parameter to determine the holdoff pattern. If RTL_RADIUS is positive then a clockwise loiter is performed, if RTL_RADIUS is negative then a counter-clockwise loiter is used.
   // @Range: -180 180
   // @Units: deg
   // @User: Standard
 --]]
 SHIP_LAND_ANGLE = bind_add_param('LAND_ANGLE', 2, 0)
 
 --[[
   // @Param: SHIP_AUTO_OFS
   // @DisplayName: Ship automatic offset trigger
   // @Description: Settings this parameter to one triggers an automatic follow offset calculation based on current position of the vehicle and the landing target. NOTE: This parameter will auto-reset to zero once the offset has been calculated.
   // @Values: 0:Disabled,1:Trigger
   // @User: Standard
 --]]
 SHIP_AUTO_OFS   = bind_add_param('AUTO_OFS', 3, 0)
 
 -- other parameters
 RCMAP_THROTTLE  = bind_param("RCMAP_THROTTLE")
 RTL_ALTITUDE    = bind_param("RTL_ALTITUDE")
 AIRSPEED_CRUISE = bind_param("AIRSPEED_CRUISE")
 AIRSPEED_MIN    = bind_param("AIRSPEED_MIN")
 TECS_LAND_ARSPD = bind_param("TECS_LAND_ARSPD")
 WP_LOITER_RAD   = bind_param("WP_LOITER_RAD")
 RTL_RADIUS      = bind_param("RTL_RADIUS")
 FOLL_OFS_X      = bind_param("FOLL_OFS_X")
 FOLL_OFS_Y      = bind_param("FOLL_OFS_Y")
 FOLL_OFS_Z      = bind_param("FOLL_OFS_Z")

 -- current target
local target_pos = Location()
local landing_target_pos = Location()
local current_pos = Location()
local target_velocity = Vector3f()
local target_heading = 0.0

-- landing stages
local STAGE_HOLDOFF = 0
local STAGE_DESCEND = 1
local STAGE_APPROACH = 2
local STAGE_LAND = 3
local STAGE_IDLE = 2
local landing_stage = STAGE_HOLDOFF

-- other state
local vehicle_mode = MODE_MANUAL
local reached_alt = false
local throttle_pos = THROTTLE_HIGH

local last_home = Location()
local current_home = Location()
local time = 0.0
local x_counter = 1
local y_counter = 1
local c = 1
local update_freq = 20

param:set_and_save('RTL_ALTITUDE', 20)

-- waypoint for landing (and waypoint parameters)
local wp_land = mavlink_mission_item_int_t()
wp_land:command(16)
wp_land:param2(0) --acceptence radius
wp_land:param3(0) --"pass trough"

local wp_land_over = mavlink_mission_item_int_t()
wp_land_over:command(16)
wp_land_over:param2(0) --acceptemce radius

local wp_plane = mavlink_mission_item_int_t()
wp_plane:command(16)
wp_plane:param2(10) --acceptence radius


local air_speed_min = AIRSPEED_MIN:get()
local AIRSPEED_CRUISE_ORIG = AIRSPEED_CRUISE:get()

local land_speed = mavlink_mission_item_int_t()
land_speed:command(178)
land_speed:param1(0) --speed measurment type (0 = air speed) 


-- square a variable
function sq(v)
   return v*v
end

-- check key parameters
function check_parameters()
  --[[
     parameter values which are auto-set on startup
  --]]
   local key_params = {
      FOLL_ENABLE = 1,
      FOLL_OFS_TYPE = 1,
      FOLL_ALT_TYPE = 0,
   }

   for p, v in pairs(key_params) do
    local current = param:get(p)
    assert(current, string.format("Parameter %s not found", p))
    if math.abs(v-current) > 0.001 then
       param:set_and_save(p, v)
       gcs:send_text(0,string.format("Parameter %s set to %.2f was %.2f", p, v, current))
    end
 end
end

-- update the pilots throttle position
function update_throttle_pos()
 local tpos
 if not rc:has_valid_input() then
    tpos = THROTTLE_LOW
 else
    local tchan = rc:get_channel(RCMAP_THROTTLE:get())
    local tval = (tchan:norm_input_ignore_trim()+1.0)*0.5
    if tval >= 0.40 then
       tpos = THROTTLE_HIGH
    elseif tval >= 0.1 then
       tpos = THROTTLE_MID
    else
       tpos = THROTTLE_LOW
    end
 end
 if tpos ~= throttle_pos then
    reached_alt = false
    if landing_stage == STAGE_HOLDOFF and tpos <= THROTTLE_MID then
       landing_stage = STAGE_DESCEND
       gcs:send_text(0, string.format("Descending for approach (hd=%.1fm h=%.1f th=%.1f)",
                                      get_holdoff_distance(), current_pos:alt()*0.01, get_target_alt()))
    end
    if landing_stage == STAGE_DESCEND and tpos == THROTTLE_HIGH then
       gcs:send_text(0,"Climbing for holdoff")
       landing_stage = STAGE_HOLDOFF
    end
 end
 throttle_pos = tpos
end

-- get landing airspeed
function get_land_airspeed()
    if TECS_LAND_ARSPD:get() < 0 then
       return AIRSPEED_CRUISE:get()
    end
    return TECS_LAND_ARSPD:get()
 end
 
 --[[
   calculate stopping distance assuming we are flying at
   TECS_LAND_ARSPD and are approaching the landing target from
   behind. Take account of the wind estimate to get approach
   groundspeed
 --]]
 function stopping_distance()
    -- get the target true airspeed for approach
    local tas = get_land_airspeed() * ahrs:get_EAS2TAS()
 
    -- add in wind in direction of flight
    local wind = ahrs:wind_estimate():xy()
 
    -- rotate wind to be in approach frame
    wind:rotate(-math.rad(target_heading + SHIP_LAND_ANGLE:get()))
 
    -- ship velocity rotated to the approach frame
    local ship2d = target_velocity:xy()
    ship2d:rotate(-math.rad(target_heading + SHIP_LAND_ANGLE:get()))
 
    -- calculate closing speed
    -- use pythagoras theorem to solve for the wind triangle
    local tas_sq = sq(tas)
    local y_sq = sq(wind:y())
    local closing_speed
    if tas_sq >= y_sq then
       closing_speed = math.sqrt(tas_sq - y_sq)
    else
       -- min 1 m/s
       closing_speed = 1.0
    end
 
    -- include the wind in the direction of the ship
    closing_speed = closing_speed + wind:x()
 
    -- account for the ship velocity
    closing_speed = closing_speed - ship2d:x()
 
    -- calculate stopping distance
    return sq(closing_speed) / (2.0 * 1)
 end
 
 -- get holdoff distance
 function get_holdoff_radius()
    if RTL_RADIUS:get() ~= 0 then
       return RTL_RADIUS:get()
    end
    return WP_LOITER_RAD:get()
 end
 
 -- get holdoff distance
 function get_holdoff_distance()
    local radius = get_holdoff_radius()
    local holdoff_dist = math.abs(radius*1.5)
    local stop_distance = stopping_distance()
 
    -- increase holdoff distance by up to 50% to ensure we can stop
    holdoff_dist = math.max(holdoff_dist, math.min(holdoff_dist*2.5, stop_distance*2))
    return holdoff_dist
 end
 
 -- get the holdoff position
 function get_holdoff_position()
    local radius = get_holdoff_radius()
    local heading_deg = target_heading + SHIP_LAND_ANGLE:get()
    local holdoff_dist = get_holdoff_distance()
 
    local ofs = Vector2f()
    ofs:x(-holdoff_dist)
    ofs:y(radius)
    ofs:rotate(math.rad(heading_deg))
    local target = target_pos:copy()
    target:offset(ofs:x(), ofs:y())
    return target
 end

 function get_landing_position()

    local heading_deg = target_heading + SHIP_LAND_ANGLE:get()
    
    local ofs = Vector2f()
    --local ofs_over = Vector2f()
    ofs:x(0)
    ofs:y(0)
    --ofs_over:x(100)
    --ofs_over:y(0)
    ofs:rotate(math.rad(heading_deg))
    --ofs_over:rotate(math.rad(heading_deg))
    local landing_target = target_pos:copy()
    --local landing_target_over = target_pos:copy()
    landing_target:offset(ofs:x(), ofs:y())
    --landing_target_over:offset(ofs_over:x(), ofs_over:y())
 
    return landing_target
 end
 
 function set_landing_mission()
 
    current_pos = ahrs:get_position()
    local new_landing_pos = Location()
    local land_alt = get_wp_alt()
    --local new_landing_pos_over = Location()
    new_landing_pos = get_landing_position()
 
    --Dummy waypoint
    wp_plane:x(current_pos:lat())
    wp_plane:y(current_pos:lng())
    wp_plane:z(current_pos:alt()*0.01)
    mission:set_item(0, wp_plane)
 
    --mission:set_item(1, land_speed)
    --gcs:send_text(0, string.format("Target landing alt (land_alt=%.1fm, lat=%.1f, long=%.1f)", land_alt, new_landing_pos:lat(), new_landing_pos:lng()))
 
    wp_land:x (new_landing_pos:lat())
    wp_land:y (new_landing_pos:lng())
    wp_land:z (land_alt)
    mission:set_item(1, wp_land)
    
   --[[
   if mission:num_commands() > 0 then
      gcs:send_text(0, string.format("HAS A MISSION"))
   else
      gcs:send_text(0, string.format("DOES NOT HAVE A MISSION"))
   end

   wp_land_over:x (new_landing_pos_over:lat())
   wp_land_over:y (new_landing_pos_over:lng())
   wp_land_over:z (0)
   mission:set_item(3, wp_land_over)
   ]]
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

 function latlng_to_m(lat2, lon2, lat1, lon1)
   local meters_per_degree_lat = 111319.9

   local lat1_rad = math.rad(lat1 / 10000000) -- Convert latitude to radians
   local lon1_rad = math.rad(lon1 / 10000000) -- Convert longitude to radians
   local lat2_rad = math.rad(lat2 / 10000000) -- Convert latitude to radians
   local lon2_rad = math.rad(lon2 / 10000000) -- Convert longitude to radians


   local dlat = lat2_rad - lat1_rad
   local dlon = lon2_rad - lon1_rad

   local y = meters_per_degree_lat * dlat
   local x = meters_per_degree_lat * dlon * math.cos((lat1_rad + lat2_rad) / 2)

   return x, y
end
 
 --[[
    check if we have reached the tangent to the landing location
 --]]
 function check_approach_tangent()
    local distance = current_pos:get_distance(target_pos)
    local holdoff_dist = get_holdoff_distance()
    if landing_stage == STAGE_HOLDOFF and throttle_pos <= THROTTLE_MID and distance < 4*holdoff_dist then
       gcs:send_text(0, string.format("Descending for approach (hd=%.1fm)", holdoff_dist))
       landing_stage = STAGE_DESCEND
    end
    if reached_alt and landing_stage == STAGE_DESCEND then
       -- go to approach stage when throttle is low, we are
       -- pointing at the ship and have reached target alt.
       -- Also require we are within 2.5 radius of the ship, and our heading is within 20
       -- degrees of the target heading
       local target_bearing_deg = wrap_180(math.deg(current_pos:get_bearing(target_pos)))
       local ground_bearing_deg = wrap_180(math.deg(ahrs:groundspeed_vector():angle()))
       local margin = 10
       local distance = current_pos:get_distance(target_pos)
       local holdoff_dist = get_holdoff_distance()
       local error1 = math.abs(wrap_180(target_bearing_deg - ground_bearing_deg))
       local error2 = math.abs(wrap_180(ground_bearing_deg - (target_heading + SHIP_LAND_ANGLE:get())))
       logger.write('SLND','TBrg,GBrg,Dist,HDist,Err1,Err2','ffffff',target_bearing_deg, ground_bearing_deg, distance, holdoff_dist, error1, error2)
       if (error1 < margin and
           distance < 2.5*holdoff_dist and
           distance > 0.7*holdoff_dist and
           error2 < 2*margin) then
          -- we are on the tangent, switch to QRTL (changed to auto since using missions to land plane)
          landing_stage = STAGE_APPROACH
          set_landing_mission() --set mission used for landing
          vehicle:set_mode(MODE_AUTO)
          
       end
    end
 end

 --[[
   check if we should abort a RTL landing
--]]
function check_approach_abort()
    local alt = current_pos:alt() * 0.01
    local target_alt = get_target_alt()
    if alt > target_alt then
       gcs:send_text(0, "Aborting landing")
       landing_stage = STAGE_HOLDOFF
       vehicle:set_mode(MODE_RTL)
       AIRSPEED_CRUISE:set(AIRSPEED_CRUISE_ORIG)
    end
 end

 -- update state based on vehicle mode
function update_mode()
    local mode = vehicle:get_mode()
    if mode == vehicle_mode then
       return
    end
    vehicle_mode = mode
    if mode == MODE_RTL then
       landing_stage = STAGE_HOLDOFF
       AIRSPEED_CRUISE:set(AIRSPEED_CRUISE_ORIG)
       reached_alt = false
    end
 end
 
 function estimate_target_velocity()
    x_diff, y_diff = latlng_to_m(current_home:lat(), current_home:lng(), last_home:lat(), last_home:lng()) 
    target_velocity:x(x_diff*100/time_last_update) -- m/s 
    target_velocity:y(y_diff*100/time_last_update) -- m/s
    -- zero vertical velocity to reduce impact of ship movement
    target_velocity:z(0)
 end

 function estimate_target_course()
   local lat1_rad = math.rad(last_home:lat() / 10000000) -- Convert latitude to radians
   local lon1_rad = math.rad(last_home:lng() / 10000000) -- Convert longitude to radians
   local lat2_rad = math.rad(current_home:lat() / 10000000) -- Convert latitude to radians
   local lon2_rad = math.rad(current_home:lng() / 10000000) -- Convert longitude to radians

    local dLon = lon2_rad - lon1_rad
    local y = math.sin(dLon) * math.cos(lat2_rad)
    local x = math.cos(lat1_rad) * math.sin(lat2_rad) - math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(dLon)
    local course = math.atan(y / x) * (180 / math.pi)  -- Convert radians to degrees

    -- Adjust angle based on quadrant
    if x < 0 then
        course = course + 180
    elseif y < 0 then
        course = course + 360
    end

    if course > 180 then
      course = course - 360
    elseif course < -180 then
      course = course + 360
    end

    return course
   
end

 -- update target state
 function update_target()
    local heading_deg = target_heading + SHIP_LAND_ANGLE:get()
    local ofs = Vector2f()
    target = target_pos:copy()
    -- Ensure updates a large enough to be performed and make changes in y and x independent
    if math.abs(target_velocity:x()*1/update_freq*x_counter) > 0.2 then
      ofs:x(target_velocity:x()*1/update_freq*x_counter)
      target:offset(0, ofs:x())
      --gcs:send_text(0,"updated x")
      x_counter = 1
    else
      x_counter = x_counter + 1
    end

    if math.abs(target_velocity:y()*1/update_freq*y_counter) > 0.2 then
      ofs:y(target_velocity:y()*1/update_freq*y_counter)
      target:offset(ofs:y(), 0)
      --gcs:send_text(0,"updated y")
      y_counter = 1
    else
      y_counter = y_counter + 1
    end

    return target
 end

 -- get the alt target for holdoff, AMSL
function get_target_alt()
    local base_alt = target_pos:alt() * 0.01
    if landing_stage == STAGE_HOLDOFF then
       return base_alt + RTL_ALTITUDE:get()
    end
    return base_alt + 15
 end
 
 function get_wp_alt()
    target_no_ofs = target_pos:copy()
    target_no_ofs:change_alt_frame(ALT_FRAME_ABSOLUTE)
    local vel_plane = Vector3f()
    vel_plane = ahrs:get_velocity_NED()
    local dist_ship_plane = target_no_ofs:get_distance_NED(current_pos)
    gcs:send_text(0,string.format("dist_ship_plane (%.2f)", dist_ship_plane ))
    local wp_alt = math.min(15, dist_ship_plane/2)
    return wp_alt
 end
 
 function update_alt()
    local alt = current_pos:alt() * 0.01
    local target_alt = get_target_alt()
    if landing_stage == STAGE_HOLDOFF or landing_stage == STAGE_DESCEND then
       if math.abs(alt - target_alt) < 3 then
          if not reached_alt then
             gcs:send_text(0,"Reached target altitude")
          end
          reached_alt = true
       end
    end
 end

function update_detection()
   if last_home:lat() ~= current_home:lat() or last_home:lng() ~= current_home:lng() then
      time_last_update = c/update_freq
      estimate_target_velocity()
      target_heading = estimate_target_course()
      last_home = current_home:copy()
      target_pos = current_home:copy()
      --reset counters for velocity estimation and x and y interval updates
      c = 1
      x_counter = 1
      y_counter = 1
   else
      target_pos = update_target()
      c = c + 1
   end
end

 function update_auto_offset()
    if arming:is_armed() or math.floor(SHIP_AUTO_OFS:get()) ~= 1 then
       return
    end
 
    -- get target without offsets applied
    target_no_ofs = target_pos:copy()
    vel = target_velocity:copy()
    target_no_ofs:change_alt_frame(ALT_FRAME_ABSOLUTE)
 
    -- setup offsets so target location will be current location
    local new = target_no_ofs:get_distance_NED(current_pos)
    new:rotate_xy(-math.rad(target_heading))
 
    gcs:send_text(0,string.format("Set follow offset (%.2f,%.2f,%.2f)", new:x(), new:y(), new:z()))
    FOLL_OFS_X:set_and_save(new:x())
    FOLL_OFS_Y:set_and_save(new:y())
    FOLL_OFS_Z:set_and_save(new:z())
 
    SHIP_AUTO_OFS:set_and_save(0)
 end
 
 --[[
 update mission used for plane landing
 ]]
 
 function update_landing_mission()
    local new_landing_pos = Location()
    local land_alt = get_wp_alt()
    new_landing_pos = get_landing_position()
    wp_land:x(new_landing_pos:lat())
    wp_land:y(new_landing_pos:lng())
    wp_land:z(land_alt)

    mission:set_item(1, wp_land)
    vehicle:set_mode(MODE_CRUISE)
    vehicle:set_mode(MODE_AUTO)
 end
 
 function update_landing_speed()
    local landing_speed = math.min(math.max(air_speed_min, math.sqrt(sq(target_velocity:x())+sq(target_velocity:y()))+1), AIRSPEED_CRUISE_ORIG)
    AIRSPEED_CRUISE:set(landing_speed)
 end
 -- main update function
 function update()
    if SHIP_ENABLE:get() < 1 then
       return
    end
    current_home = ahrs:get_home()

    update_detection()
    current_pos = ahrs:get_position()
    if not current_pos then
       return
    end
    current_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
 
    update_throttle_pos()
    update_mode()
    update_alt()
    update_auto_offset()
    
    local next_WP = vehicle:get_target_location()
    if not next_WP then
       -- not in a flight mode with a target location
       return
    end
    next_WP:change_alt_frame(ALT_FRAME_ABSOLUTE)
 
    if vehicle_mode == MODE_RTL then
       local holdoff_pos = get_holdoff_position()
       local de_acc_dist = stopping_distance()
       holdoff_pos:change_alt_frame(ALT_FRAME_ABSOLUTE)
       holdoff_pos:alt(math.floor(get_target_alt()*100))
       vehicle:update_target_location(next_WP, holdoff_pos)
 
       if throttle_pos == THROTTLE_LOW then
          check_approach_tangent()
       end
 
    elseif vehicle_mode == MODE_AUTO and landing_stage == STAGE_APPROACH then
       current_pos = ahrs:get_position()
       local distance = current_pos:get_distance(target_pos)
       --gcs:send_text(0, "Vehicle mode loop ran")
 
       update_landing_mission()
       if distance < de_acc_dist then
          landing_stage = STAGE_LAND
       end
 
       if throttle_pos == THROTTLE_HIGH then
          check_approach_abort()
       end
      
    elseif vehicle_mode == MODE_AUTO and landing_stage ~= STAGE_APPROACH then
       local id = mission:get_current_nav_id()
       if id == NAV_VTOL_TAKEOFF or id == NAV_TAKEOFF then
          vehicle:set_velocity_match(target_velocity:xy())
          local tpos = current_pos:copy()
          tpos:alt(next_WP:alt())
          vehicle:update_target_location(next_WP, tpos)
       end
    
    elseif vehicle_mode == STAGE_LAND then
      update_landing_mission()
      update_landing_speed()
    end
 
 end
 
 function loop()
    update()
    
    -- run at 20Hz
    return loop, update_freq
 end
 
 check_parameters()
 
 -- wrapper around update(). This calls update() at 20Hz,
 -- and if update faults then an error is displayed, but the script is not
 -- stopped
 function protected_wrapper()
   local success, err = pcall(update)
   if not success then
      gcs:send_text(0, "Internal Error: " .. err)
      -- when we fault we run the update function again after 1s, slowing it
      -- down a bit so we don't flood the console with errors
      return protected_wrapper, 1000
   end
   return protected_wrapper, 50
 end
 
 -- start running update loop
 return protected_wrapper()