x1 = 0
y1 = 0
z1 = 0
x2 = 0
y2 = 0
z2 = 0
function sysCall_init() 
    -- Inisialisasi objek
    steer_handle = sim.getObject('./steer_joint')
    motor_handle = sim.getObject('./motor_joint')
    fl_brake_handle = sim.getObject('./fl_brake_joint')
    fr_brake_handle = sim.getObject('./fr_brake_joint')
    bl_brake_handle = sim.getObject('./bl_brake_joint')
    br_brake_handle = sim.getObject('./br_brake_joint')
    
    -- Parameter kendaraan
    max_steer_angle = 0.5235987  -- 30 degrees
    motor_torque = 60
    
    -- Inisialisasi variabel input
    
    -- Inisialisasi nilai input

    brake_force = 0

    -- Inisialisasi ROS node
    if (simROS) then
        print("ROS is initialized")
        -- Berlangganan ke topik
        simROS.subscribe('/fnav', 'geometry_msgs/Point', 'navigationCallback')
        simROS.subscribe('/rho', 'geometry_msgs/Point', 'robotCallback')
    else
        print("ROS is not initialized")
    end
end

-- Callback untuk topik /fnav
function navigationCallback(msg)
    x1 = msg.x
    y1 = msg.y
    z1 = msg.z
    x1 = math.abs(x1)
    print("Received from /fnav: x1=" .. x1 .. ", y1=" .. y1 .. ", z1=" .. z1)
end

-- Callback untuk topik /rho
function robotCallback(msg)
    x2 = msg.x
    y2 = msg.y
    z2 = msg.z
    print("Received from /rho: x2=" .. x2 .. ", y2=" .. y2 .. ", z2=" .. z2)
end

function mapValue(x, in_min, in_max, out_min, out_max)
    return out_min + ((x - in_min) * (out_max - out_min)) / (in_max - in_min)
end

function sysCall_actuation() 

    local _temp_x2 = mapValue(math.abs(x2), 180, 0, 0, 30)
    -- Print untuk debugging
    print("Current Steering (x1): " .. x1 .. " | Current Velocity (x2): " .. _temp_x2)
    
    if (x2 > 0) then
    _temp_x2 = _temp_x2*-1
    end
    

    
    steer_angle = _temp_x2 --steering
    motor_velocity = x1  --fnav
    
    
    
    -- Batasan sudut kemudi
    if (steer_angle > max_steer_angle) then
        steer_angle = max_steer_angle
    end
    if (steer_angle < -max_steer_angle) then
        steer_angle = -max_steer_angle
    end
    
    -- Set kemudi dan kecepatan
    sim.setJointTargetPosition(steer_handle, steer_angle)
    sim.setJointTargetForce(motor_handle, motor_torque)
    sim.setJointTargetVelocity(motor_handle, motor_velocity)
    
    -- Set kekuatan rem jika diperlukan
    if (math.abs(motor_velocity) < 0.1) then
        brake_force = 100
    else
        brake_force = 0
    end

    -- Aplikasi rem
    sim.setJointTargetForce(fr_brake_handle, brake_force)
    sim.setJointTargetForce(fl_brake_handle, brake_force)
    sim.setJointTargetForce(bl_brake_handle, brake_force)
    sim.setJointTargetForce(br_brake_handle, brake_force)
end
