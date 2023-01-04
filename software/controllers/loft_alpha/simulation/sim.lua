
function velocityCommandCb(msg)
    --[[
        - description: callback for receiving wheel/joint command
        - input: wheel/joint command message
            + v1: left wheel speed
            + v2: right wheel speed
            + v3: front/rear wheel speed
            + alpha: front/rear wheel angle
        - output: none
    ]]
    velocityCommand[1] = msg.data[1] -- v1
    velocityCommand[2] = msg.data[2] -- v2
    velocityCommand[3] = msg.data[3] -- v3
    velocityCommand[4] = msg.data[4] -- alpha
end

function twistCommandCb(msg)
    v = msg.data[1] -- v
    w = msg.data[2] -- w
    yaw = math.atan2(2*w,v)
    scale = 2*v
    if (v < 0) then 
        yaw = yaw + 3.14159
    else 
        yaw = -yaw
    end
    if scale == 0 then 
        scale = 0.01
    end
    sim.scaleObject(arrowBlueVisualHandle,1,1,1/bluearrow_scale)
    sim.scaleObject (arrowBlueVisualHandle,1,1,scale)
    bluearrow_scale = scale
    bluearrow_yaw = yaw
    

end

function planningCommandCb(msg)
    v = msg.data[1] -- v
    w = msg.data[2] -- w
    yaw = math.atan2(2*w,v)
    scale = 2*v
    if (v < 0) then 
        yaw = yaw + 3.14159
    else 
        yaw = -yaw
    end
    if scale == 0 then 
        scale = 0.01
    end
    sim.scaleObject(arrowRedVisualHandle,1,1,1/redarrow_scale)
    sim.scaleObject (arrowRedVisualHandle,1,1,scale)
    redarrow_scale = scale
    redarrow_yaw = yaw
    

end

function hdetectionsCB(msg)
    --[[
        - description: callback for receiving human detection message
        - input: human detection message
        - output: none
    ]]
    if (obstacle_enable > 0) then
        -- choose only first 5 human (i.e., 10 values)
        size = #msg.data
        if #msg.data > 10 then  
            size = 10
        end
        
        -- place 5 obstacles block at (xi,yi)
        for i=1,5,1 do
            if i <= (size/2) then 
                sim.setObjectPosition(obsHandle[i],robotHandle,{msg.data[i],msg.data[i+1],0.2})
            else
                sim.setObjectPosition(obsHandle[i],robotHandle,{0.0,0.0,-1.0})
            end
        end
    end
end

function expectationCb(msg)
    sim.setObjectPosition(midDummyHandle,worldHandle,{msg.data[1],msg.data[2],0})
    sim.setObjectOrientation(midDummyHandle,worldHandle,{0,0,msg.data[3]-1.57})
    sim.setObjectPosition(expectDummyHandle,worldHandle,{msg.data[4],msg.data[5],0})
    sim.setObjectOrientation(expectDummyHandle,worldHandle,{0,0,msg.data[6]-1.57})
end

function obstacleFlagCb(msg)
    obstacle_enable = msg.data[1]
end

function lidarscanCb(msg)
    obstaclemax = 24
    lidarsize = #msg.ranges
    lidarmin = {100000.0,100000.0,100000.0,100000.0,100000.0,
    100000.0,100000.0,100000.0,100000.0,100000.0,
    100000.0,100000.0,100000.0,100000.0,100000.0,
    100000.0,100000.0,100000.0,100000.0,100000.0,
    100000.0,100000.0,100000.0,100000.0,100000.0}

    lidarangle = {0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,
    0.0,0.0,0.0,0.0,0.0,}
    for i=1,lidarsize,1 do
        nsection = math.floor((i-1)/(lidarsize/obstaclemax))+1
        if msg.ranges[i] < lidarmin[nsection] then 
            lidarmin[nsection] = msg.ranges[i]
            lidarangle[nsection] = (i-1)*(360/lidarsize)
        end 
    end
    _, maxD = simROS.getParamDouble("/sensor_limit",2.1)
    for i=1,obstaclemax,1 do
        if lidarmin[i] < maxD then
            x = lidarmin[i]*math.cos(lidarangle[i]*0.0175)
            y = lidarmin[i]*math.sin(lidarangle[i]*0.0175)
            sim.setObjectPosition(obsHandle[i],robotHandle,{x,y,0.2})
        else
            sim.setObjectPosition(obsHandle[i],robotHandle,{0,0,-10}) 
        end
    end
end

visualizePath=function(path)
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,3,0,-1,99999,{0.2,0.2,0.2})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/3
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*3+1],path[(i-1)*3+2],initPos[3],path[i*3+1],path[i*3+2],initPos[3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

function sysCall_init()

    clock = os.clock

    motorLeft=sim.getObjectHandle("joint_leftwheel")
    motorRight=sim.getObjectHandle("joint_rightwheel")
    motorYawFront=sim.getObjectHandle("joint_frontyaw")
    motorYawBack=sim.getObjectHandle("joint_backyaw")
    motorFront=sim.getObjectHandle("joint_frontwheel")
    motorBack=sim.getObjectHandle("joint_backwheel")
    robotHandle = sim.getObjectHandle("capy_body")
    robotBBHandle = sim.getObjectHandle("RobotBoundingBox")
    robotDummyHandle = sim.getObjectHandle("capy_cg")
    worldHandle = sim.getObjectHandle("ref")
    goalHandle = sim.getObjectHandle("goal")
    targetHandle = sim.getObjectHandle("target")
    goalDummyHandle = sim.getObjectHandle("goalDummy")
    expectDummyHandle = sim.getObjectHandle("expectDummy")
    midDummyHandle = sim.getObjectHandle("midDummy")
    arrowRedHandle = sim.getObjectHandle("arrowRed")
    arrowRedVisualHandle = sim.getObjectHandle("arrowRedVisual")
    arrowBlueHandle = sim.getObjectHandle("arrowBlue")
    arrowBlueVisualHandle = sim.getObjectHandle("arrowBlueVisual")
    
    velocityCommand = {0,0,0,0}
    t0 = 0.0
    logt = 0.0
    coli = 0.0
    goal_count = 0
    obstacle_enable = 0
    redarrow_scale = 1.0
    bluearrow_scale = 1.0
    redarrow_yaw = 0.0
    bluearrow_yaw = 0.0

    -- get lidar values
    visionSensor1Handle=sim.getObjectHandle("SICK_S300_sensor1")
    visionSensor2Handle=sim.getObjectHandle("SICK_S300_sensor2")
    visionSensor3Handle=sim.getObjectHandle("SICK_S300_sensor3")
    joint1Handle=sim.getObjectHandle("SICK_S300_joint1")
    joint2Handle=sim.getObjectHandle("SICK_S300_joint2")
    joint3Handle=sim.getObjectHandle("SICK_S300_joint3")
    sensorBaseHandle=sim.getObjectHandle("capy_body")--sim.getObjectAssociatedWithScript(sim.handle_self)

    worldHandle = sim.getObjectHandle("worldRef")

    
    obsHandle= {0,0,0,0,0}
    for i=1,24,1 do
        obsHandle[i] = sim.getObjectHandle("obstacle"..tostring(i))
    end

    if simROS then
        print("<font color='#0F0'>ROS interface was found.</font>@html")

        subscribers = {}
        advertisers = {}

        table.insert(advertisers, simROS.advertise('/simulation/lidar/angle','std_msgs/Float32MultiArray'))
        table.insert(advertisers, simROS.advertise('/simulation/lidar/distance','std_msgs/Float32MultiArray'))
        table.insert(advertisers, simROS.advertise('/simulation/pose','std_msgs/Float32MultiArray'))
        table.insert(advertisers, simROS.advertise('/planning/goal','std_msgs/Float32MultiArray'))
        table.insert(advertisers, simROS.advertise('/simulation/hit','std_msgs/Float32MultiArray'))
        table.insert(advertisers, simROS.advertise('/simulation/goalcount','std_msgs/Float32MultiArray'))

        table.insert(subscribers, simROS.subscribe('/loft/velocityCommand','std_msgs/Float32MultiArray','velocityCommandCb'),1)
        table.insert(subscribers, simROS.subscribe('/simulation/humanxy','std_msgs/Float32MultiArray','hdetectionsCB'),1)
        table.insert(subscribers, simROS.subscribe('/joystick/obstacleFlag','std_msgs/Float32MultiArray','obstacleFlagCb'),1)
        table.insert(subscribers, simROS.subscribe('/loft/twistCommand','std_msgs/Float32MultiArray','twistCommandCb'),1)
        table.insert(subscribers, simROS.subscribe('/planning/twistCommand','std_msgs/Float32MultiArray','planningCommandCb'),1)
        table.insert(subscribers, simROS.subscribe('/planning/expectation','std_msgs/Float32MultiArray','expectationCb'),1)
        table.insert(subscribers, simROS.subscribe('/loft/lidarscan','sensor_msgs/LaserScan','lidarscanCb'),1)
        
    else
        print("<font color='#F00'>ROS interface was not found. Cannot run.</font>@html")
    end
    
    _, maxD = simROS.getParamDouble("/sensor_limit",2.1)
    maxScanDistance= maxD --sim.getScriptSimulationParameter(sim.handle_self,'maxScanDistance')
    if maxScanDistance>1000 then maxScanDistance=1000 end
    if maxScanDistance<0.1 then maxScanDistance=0.1 end
    sim.setObjectFloatParameter(visionSensor1Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParameter(visionSensor2Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    sim.setObjectFloatParameter(visionSensor3Handle,sim.visionfloatparam_far_clipping,maxScanDistance)
    maxScanDistance_=maxScanDistance*0.9999

    scanningAngle=360--sim.getScriptSimulationParameter(sim.handle_self,'scanAngle')
    --if scanningAngle>270 then scanningAngle=270 end
    --if scanningAngle<2 then scanningAngle=2 end
    scanningAngle=scanningAngle*math.pi/180
    sim.setObjectFloatParameter(visionSensor1Handle,sim.visionfloatparam_perspective_angle,scanningAngle/3)
    sim.setObjectFloatParameter(visionSensor2Handle,sim.visionfloatparam_perspective_angle,scanningAngle/3)
    sim.setObjectFloatParameter(visionSensor3Handle,sim.visionfloatparam_perspective_angle,scanningAngle/3)

    sim.setJointPosition(joint1Handle,-scanningAngle/4)
    sim.setJointPosition(joint2Handle,scanningAngle/4)
    sim.setJointPosition(joint3Handle,scanningAngle/4)
    red={1,0,0}
    lines=sim.addDrawingObject(sim.drawing_lines,1,0,-1,1000,nil,nil,nil,red)

    --initPos=sim.getObjectPosition(robotHandle,-1)
    --initOrient=sim.getObjectOrientation(robotHandle,-1)    

    if (sim.getInt32Parameter(sim.intparam_program_version)<30004) then
        sim.displayDialog("ERROR","This version of the SICK sensor is only supported from CoppeliaSim V3.0.4 and upwards.&&nMake sure to update your CoppeliaSim.",sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    
    csv_write("/home/zubuntu/loft.csv",{os.time(os.date("!*t")),goal_count,0})

    
end


function getLidar(sensor,m_robot,m_lidar)
    p={0,0,0} -- center
    p=sim.multiplyVector(m_lidar,p) -- rotate center
    t={p[1],p[2],p[3],0,0,0}
    t2 = sim.multiplyVector(m_robot,p)
    for j=0,sensor[2]-1,step do
        for i=0,sensor[1]-1,step do
            w=2+4*(j*sensor[1]+i)
            v1=sensor[w+1]
            v2=sensor[w+2]
            v3=sensor[w+3]
            v4=sensor[w+4]
            
            
            p={v1,v2,v3}
            p=sim.multiplyVector(m_lidar,p)
            p2=sim.multiplyVector(m_robot,p)

            t[4] = p[1]
            t[5] = p[2]
            t[6] = p[3]

            t2[4] = p2[1]
            t2[5] = p2[2]
            t2[6] = p2[3]

            d = {t[4]-t[1],t[5]-t[2],t[6]-t[3]}

            angle = math.atan2(d[1],d[2])*180/3.14
            angle = angle+180-30-60
            dist = math.sqrt(d[1]*d[1]+d[2]*d[2])

            --sim.addDrawingObjectItem(lines,t2)

            
            tab_i = getNearestAngle(angle)

            measuredCount[tab_i] = measuredCount[tab_i]+1
            measuredDistance[tab_i] = measuredDistance[tab_i]+ dist
        end
    end
end


function sysCall_sensing()
    -- lidar data
    step = 5 -- number of lidar step
    measuredAngle = {}
    measuredDistance = {}
    measuredCount = {}
    for i=0,360,1 do
        table.insert(measuredAngle, i)
        table.insert(measuredDistance,0.0)
        table.insert(measuredCount,0.0)
    end
    
    if notFirstHere then
        -- We skip the very first reading
        sim.addDrawingObjectItem(lines,nil)
        r,t1,u1=sim.readVisionSensor(visionSensor1Handle)
        r,t2,u2=sim.readVisionSensor(visionSensor2Handle)
        r,t3,u3=sim.readVisionSensor(visionSensor3Handle)

        m1=sim.getObjectMatrix(visionSensor1Handle,robotHandle)
        m2=sim.getObjectMatrix(visionSensor2Handle,robotHandle)
        m3=sim.getObjectMatrix(visionSensor3Handle,robotHandle)
        mr=simGetInvertedMatrix(sim.getObjectMatrix(worldHandle,robotHandle))

        getLidar(u1,mr,m1)
        getLidar(u2,mr,m2)
        getLidar(u3,mr,m3)
    end

    for i=0,360,1 do
        if (measuredCount[i+1] == 0.0) then
            measuredDistance[i+1] = maxScanDistance
        else
            measuredDistance[i+1] = measuredDistance[i+1]/measuredCount[i+1]
        end
    end

    col = sim.checkCollision(robotBBHandle,sim.handle_all)

    startpos=sim.getObjectPosition(robotHandle,worldHandle)
    dx = startpos[1]-goal[1]
    dy = startpos[2]-goal[2]
    dist = math.sqrt(dx*dx+dy*dy)
    if dist < 0.35 then
        goal_count = goal_count + 1;
    end

    notFirstHere=true
    simROS.publish(advertisers[1],{data=measuredAngle}) 
    simROS.publish(advertisers[2],{data=measuredDistance}) 
    position = sim.getObjectPosition(robotDummyHandle,worldHandle)
    orientation = sim.getObjectOrientation(robotHandle,worldHandle)
    simROS.publish(advertisers[3],{data={position[1],position[2],orientation[3]}}) 
    simROS.publish(advertisers[5],{data={col}}) 
    simROS.publish(advertisers[6],{data={goalcount}}) 
end

function sysCall_actuation() 

    goalPos =sim.getObjectPosition(goalHandle,-1)
    goal = {goalPos[1],goalPos[2]}
    sim.setJointTargetVelocity(motorLeft,velocityCommand[1])
    sim.setJointTargetVelocity(motorRight,velocityCommand[2])
    sim.setJointTargetVelocity(motorFront,velocityCommand[3])
    sim.setJointTargetVelocity(motorBack,velocityCommand[3])
    sim.setJointTargetPosition(motorYawFront,velocityCommand[4])
    sim.setJointTargetPosition(motorYawBack,-velocityCommand[4])

    targetpos=sim.getObjectPosition(targetHandle,-1)
    simROS.publish(advertisers[4],{data={targetpos[1],targetpos[2]}}) 

    _, halfspeed = simROS.getParamDouble("max_speed",0.5)
    halfspeed = halfspeed/2
    sim.setObjectPosition(arrowBlueHandle,robotHandle,{0+halfspeed*bluearrow_scale*math.cos(bluearrow_yaw),-halfspeed*bluearrow_scale*math.sin(bluearrow_yaw),-0.05})
    sim.setObjectOrientation(arrowBlueHandle,robotHandle,{0,0,-bluearrow_yaw})
    sim.setObjectPosition(arrowRedHandle,robotHandle,{0+halfspeed*redarrow_scale*math.cos(redarrow_yaw),-halfspeed*redarrow_scale*math.sin(redarrow_yaw),-0.05})
    sim.setObjectOrientation(arrowRedHandle,robotHandle,{0,0,-redarrow_yaw})

end 

function sysCall_cleanup()
    sim.removeDrawingObject(lines)
    sim.scaleObject(arrowRedVisualHandle,1,1,1/redarrow_scale)
    sim.scaleObject(arrowBlueVisualHandle,1,1,1/bluearrow_scale)
    -- Wait for the signal to reach the node
    waitTimer=0
    simROS.publish(advertisers[1],{data=true})  
    print('terminate')
    while( waitTimer < 1000 ) do
        waitTimer = waitTimer+1
        simROS.publish(advertisers[1],{data=true})
    end

    for i=1,table.getn(advertisers),1 do
        simROS.shutdownPublisher(advertisers[i])
    end

    for i=1,table.getn(subscribers),1 do
        simROS.shutdownPublisher(subscribers[i])
    end
end

function getNearestAngle(x)
    if x < 1 then
        x = (360+x)
    end
    xi = math.floor(x)
    if (x-xi) > 0.5 then
        return xi+1
    else
        return xi
    end 
end

function csv_write(path, data)
    sep = ','
    local file = assert(io.open(path, "a"))
    for d=1,table.getn(data) do
      file:write(data[d] .. ",")
    end
    file:write('\n')
    file:close()
end

