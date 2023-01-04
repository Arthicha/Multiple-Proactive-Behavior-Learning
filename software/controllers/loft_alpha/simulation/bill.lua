--[[
created by zumo arthicha.
date: 21 Aug 2022
simulation bill controller
]]


function sysCall_init() 
    BillHandle=sim.getObjectHandle('Bill_base')
    legJointHandles={sim.getObjectHandle('Bill_leftLegJoint'),sim.getObjectHandle('Bill_rightLegJoint')}
    kneeJointHandles={sim.getObjectHandle('Bill_leftKneeJoint'),sim.getObjectHandle('Bill_rightKneeJoint')}
    ankleJointHandles={sim.getObjectHandle('Bill_leftAnkleJoint'),sim.getObjectHandle('Bill_rightAnkleJoint')}
    shoulderJointHandles={sim.getObjectHandle('Bill_leftShoulderJoint'),sim.getObjectHandle('Bill_rightShoulderJoint')}
    elbowJointHandles={sim.getObjectHandle('Bill_leftElbowJoint'),sim.getObjectHandle('Bill_rightElbowJoint')}
    neckJoint=sim.getObjectHandle('Bill_neck')
    pathHandle=sim.getObjectHandle('Bill_path')
    
    legWaypoints={0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188}
    kneeWaypoints={0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092}
    ankleWaypoints={-0.133,0.041,0.244,0.382,0.304,0.232,0.266,0.061,-0.090,-0.145,-0.043,0.041,0.001,0.011,-0.099,-0.127,-0.121,-0.120,-0.107,-0.100,-0.090,-0.009}
    shoulderWaypoints={0.028,0.043,0.064,0.078,0.091,0.102,0.170,0.245,0.317,0.337,0.402,0.375,0.331,0.262,0.188,0.102,0.094,0.086,0.080,0.051,0.058,0.048}
    elbowWaypoints={-1.148,-1.080,-1.047,-0.654,-0.517,-0.366,-0.242,-0.117,-0.078,-0.058,-0.031,-0.001,-0.009,0.008,-0.108,-0.131,-0.256,-0.547,-0.709,-0.813,-1.014,-1.102}
    relativeVel={2,2,1.2,2.3,1.4,1,1,1,1,1.6,1.9,2.4,2.0,1.9,1.5,1,1,1,1,1,2.3,1.5}
    
    
    nominalVelocity=randomSpeed()--sim.getScriptSimulationParameter(sim.handle_self,'walkingSpeed')
    startPosInitialPause=sim.getScriptSimulationParameter(sim.handle_self,'startPositionInitialPause')
    startPosPause=sim.getScriptSimulationParameter(sim.handle_self,'startPositionPause')
    goalPosPause=1--sim.getScriptSimulationParameter(sim.handle_self,'goalPositionPause')
    singleJourneyCount=sim.getScriptSimulationParameter(sim.handle_self,'singleJourneyCount')
    randomColors=sim.getScriptSimulationParameter(sim.handle_self,'randomColors')
    vel=nominalVelocity*0.8/0.56 
    scaling=0
    tl=#legWaypoints
    dl=1/tl
    vp=0
    journeyCount=0
    location=0 -- 0=at start pos waiting,1=at start pos turning, 2=walking towards goal, 3=at goal pos waiting, 4=at goal pos turning, 5=walking towards start
    pauseUntil=sim.getSimulationTime()+startPosInitialPause
    currentPosOnPath=0
    
    HairColors={4,{0.30,0.22,0.14},{0.75,0.75,0.75},{0.075,0.075,0.075},{0.75,0.68,0.23}}
    skinColors={2,{0.61,0.54,0.45},{0.52,0.45,0.35}}
    shirtColors={5,{0.27,0.36,0.54},{0.54,0.27,0.27},{0.31,0.51,0.33},{0.46,0.46,0.46},{0.18,0.18,0.18}}
    trouserColors={2,{0.4,0.34,0.2},{0.12,0.12,0.12}}
    shoeColors={2,{0.12,0.12,0.12},{0.25,0.12,0.045}}
    
    -- Initialize to random colors if desired:
    if (randomColors) then
        -- First we just retrieve all objects in the model:
        previousSelection=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(sim.handle_tree,BillHandle)
        modelObjects=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(previousSelection)
        -- Now we set random colors:
        math.randomseed(sim.getFloatParameter(sim.floatparam_rand)*10000) -- each lua instance should start with a different and 'good' seed
        setColor(modelObjects,'HAIR',HairColors[1+math.random(HairColors[1])])
        setColor(modelObjects,'SKIN',skinColors[1+math.random(skinColors[1])])
        setColor(modelObjects,'SHIRT',shirtColors[1+math.random(shirtColors[1])])
        setColor(modelObjects,'TROUSERS',trouserColors[1+math.random(trouserColors[1])])
        setColor(modelObjects,'SHOE',shoeColors[1+math.random(shoeColors[1])])
    end
end
------------------------------------------------------------------------------ 
-- Following few lines automatically added by CoppeliaSim to guarantee compatibility 
-- with CoppeliaSim 3.1.3 and earlier: 
colorCorrectionFunction=function(_aShapeHandle_) 
  local version=sim.getInt32Parameter(sim.intparam_program_version) 
  local revision=sim.getInt32Parameter(sim.intparam_program_revision) 
  if (version<30104)and(revision<3) then 
      return _aShapeHandle_ 
  end 
  return '@backCompatibility1:'.._aShapeHandle_ 
end 
------------------------------------------------------------------------------ 

function randomSpeed()
    --[[ bell velocity]]
    lower = 0.2
    greater = 0.3
    return lower + math.random()  * (greater - lower);
end
 
setColor=function(objectTable,colorName,color)
    for i=1,#objectTable,1 do
        if (sim.getObjectType(objectTable[i])==sim.object_shape_type) then
            sim.setShapeColor(colorCorrectionFunction(objectTable[i]),colorName,0,color)
        end
    end
end


function sysCall_cleanup() 
    -- Restore to initial colors:
    if (randomColors) then
        previousSelection=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(sim.handle_tree,BillHandle)
        modelObjects=sim.getObjectSelection()
        sim.removeObjectFromSelection(sim.handle_all,-1)
        sim.addObjectToSelection(previousSelection)
        setColor(modelObjects,'HAIR',HairColors[2])
        setColor(modelObjects,'SKIN',skinColors[2])
        setColor(modelObjects,'SHIRT',shirtColors[2])
        setColor(modelObjects,'TROUSERS',trouserColors[2])
        setColor(modelObjects,'SHOE',shoeColors[2])
    end
end 

function sysCall_actuation() 
    dt=sim.getSimulationTimeStep()
    simTime=sim.getSimulationTime()
    
    s=sim.getObjectSizeFactor(BillHandle)
    
    if (simTime>pauseUntil)and(journeyCount<singleJourneyCount) then
    
        pathL=sim.getPathLength(pathHandle)
        if (location==0)or(location==3) then
            location=location+1
        end
    
        walkingDir=1
        if (location==4)or(location==5) then
            walkingDir=-1 
        end
    
        if (location==1)or(location==4) then
            -- For now we turn instantly:
            location=location+1
        end
        
    
    
        if (location==2)or(location==5) then
    
            scaling=1--math.max(math.min(math.min(currentPosOnPath,pathL-currentPosOnPath)*4,1),0.05)
            vp=vp+sim.getSimulationTimeStep()*vel
            p=math.fmod(vp,1)
            indexLow=math.floor(p/dl)
            t=p/dl-indexLow
            oppIndexLow=math.floor(indexLow+tl/2)
            if (oppIndexLow>=tl) then oppIndexLow=oppIndexLow-tl end
            indexHigh=indexLow+1
            if (indexHigh>=tl) then indexHigh=indexHigh-tl end
            oppIndexHigh=oppIndexLow+1
            if (oppIndexHigh>=tl) then oppIndexHigh=oppIndexHigh-tl end
    
            leftLegJointValue=(legWaypoints[indexLow+1]*(1-t)+legWaypoints[indexHigh+1]*t)*scaling
            leftKneeJointValue=(kneeWaypoints[indexLow+1]*(1-t)+kneeWaypoints[indexHigh+1]*t)*scaling
            leftAnkleJointValue=(ankleWaypoints[indexLow+1]*(1-t)+ankleWaypoints[indexHigh+1]*t)*scaling
            leftShoulderJointValue=(shoulderWaypoints[indexLow+1]*(1-t)+shoulderWaypoints[indexHigh+1]*t)*scaling
            leftElbowJointValue=(elbowWaypoints[indexLow+1]*(1-t)+elbowWaypoints[indexHigh+1]*t)*scaling
    
            rightLegJointValue=(legWaypoints[oppIndexLow+1]*(1-t)+legWaypoints[oppIndexHigh+1]*t)*scaling
            rightKneeJointValue=(kneeWaypoints[oppIndexLow+1]*(1-t)+kneeWaypoints[oppIndexHigh+1]*t)*scaling
            rightAnkleJointValue=(ankleWaypoints[oppIndexLow+1]*(1-t)+ankleWaypoints[oppIndexHigh+1]*t)*scaling
            rightShoulderJointValue=(shoulderWaypoints[oppIndexLow+1]*(1-t)+shoulderWaypoints[oppIndexHigh+1]*t)*scaling
            rightElbowJointValue=(elbowWaypoints[oppIndexLow+1]*(1-t)+elbowWaypoints[oppIndexHigh+1]*t)*scaling
    
    
            vvv=s*nominalVelocity*scaling*(relativeVel[indexLow+1]*(1-t)+relativeVel[indexHigh+1]*t)
    
            r=sim.getObjectPosition(BillHandle,-1)
            currentPosOnPath=currentPosOnPath+walkingDir*dt*vvv
            relPosOnPath=currentPosOnPath/pathL
            -- Get Bill's new position:
            position=sim.getPositionOnPath(pathHandle,relPosOnPath)
            -- Now calculate two vectors (for the body direction and head direction):
            position2=sim.getPositionOnPath(pathHandle,(currentPosOnPath+0.0009*walkingDir)/pathL)
            position3=sim.getPositionOnPath(pathHandle,(currentPosOnPath+2*walkingDir)/pathL)
            walkingVector={position2[1]-position[1],position2[2]-position[2]}
            headVector={position3[1]-position[1],position3[2]-position[2]}
            -- Check if Bill arrived at the goal position:
            if (currentPosOnPath>pathL-0.001)and(walkingDir>0) then
                location=3
                pauseUntil=simTime+goalPosPause
                journeyCount=journeyCount+1
                nominalVelocity=randomSpeed()
            end
            -- Check if Bill arrived at the start position:
            if (currentPosOnPath<0.001)and(walkingDir<0) then
                location=0
                pauseUntil=simTime+startPosPause
                journeyCount=journeyCount+1
                nominalVelocity=randomSpeed()
            end
            -- We calculate the Bill's orientation:
            orientation=math.atan2(walkingVector[2],walkingVector[1]) 
            -- We calculate the Bill's head orientation:
            headOrientation=math.atan2(headVector[2],headVector[1])-orientation
            if (math.abs(headOrientation)>math.pi) then 
                headOrientation=headOrientation+math.pi*-2*headOrientation/math.abs(headOrientation) 
            end
            -- Keep Bill's z-position:
            position[3]=sim.getObjectPosition(BillHandle,-1)[3]
            -- Set Bill's new position
            sim.setObjectPosition(BillHandle,-1,position)
            -- Set Bill's new orientation and Bill's new head orientation:
            if (location==2)or(location==5) then
                sim.setObjectOrientation(BillHandle,-1,{0,0,orientation})
                sim.setJointPosition(neckJoint,headOrientation)
            end
        end
    end
    
    if (location~=2)and(location~=5) then
        leftLegJointValue=0
        leftKneeJointValue=0
        leftAnkleJointValue=0
        leftShoulderJointValue=0
        leftElbowJointValue=0
    
        rightLegJointValue=0
        rightKneeJointValue=0
        rightAnkleJointValue=0
        rightShoulderJointValue=0
        rightElbowJointValue=0
    end

    gain = 0.05 -- [[ bill's joint modification gain]]
    
    sim.setJointPosition(legJointHandles[1],leftLegJointValue*gain)
    sim.setJointPosition(kneeJointHandles[1],leftKneeJointValue*gain)
    sim.setJointPosition(ankleJointHandles[1],leftAnkleJointValue*gain)
    sim.setJointPosition(shoulderJointHandles[1],leftShoulderJointValue*gain)
    sim.setJointPosition(elbowJointHandles[1],leftElbowJointValue*gain)
    
    sim.setJointPosition(legJointHandles[2],rightLegJointValue*gain)
    sim.setJointPosition(kneeJointHandles[2],rightKneeJointValue*gain)
    sim.setJointPosition(ankleJointHandles[2],rightAnkleJointValue*gain)
    sim.setJointPosition(shoulderJointHandles[2],rightShoulderJointValue*gain)
    sim.setJointPosition(elbowJointHandles[2],rightElbowJointValue*gain)
end 
