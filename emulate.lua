-- This script is used for realtime emulation of the environment in V-REP

function sysCall_init()

    -- importing math module
    require "math"
   
    -- Adding required handles here

    goal_1 = sim.getObjectHandle('goal_1')                              --taking handle of goal_1 (i.e. dummy 1)
     
    goal_2 = sim.getObjectHandle('goal_2')                              --taking handle of goal_2 (i.e. dummy 2)
   
    goal_3 = sim.getObjectHandle('goal_3')                              --taking handle of goal_3 (i.e. dummy 3)

    goal_4 = sim.getObjectHandle('goal_4')                              --taking handle of goal_4 (i.e. dummy 4)

    goal_5 = sim.getObjectHandle('goal_5')                              --taking handle of goal_5 (i.e. dummy 5)

    goal_6 = sim.getObjectHandle('goal_6')                              --taking handle of goal_6 (i.e. dummy 6)

    goal_7 = sim.getObjectHandle('goal_7')                              --taking handle of goal_7 (i.e. dummy 7)

    boundary = sim.getObjectHandle('boundary')                          --taking handle of boundary

    collection_handles= sim.getCollectionHandle('Obstacles')    --gaining handle of collection of obstacles
   
    Edrone = sim.getObjectHandle('Drone_Pos_Emulation')                 --taking handle of 'Drone_Pos_Emulation'

    pos_hoop_3 = sim.getObjectHandle('Position_hoop_3')                     --taking handle of position_hoop (i.e. red tree)
    
    orient_hoop_3 = sim.getObjectHandle('Orientation_hoop_3')               --taking handle of orientation_hoop (i.e. red tree)

    pos_hoop_2 = sim.getObjectHandle('Position_hoop_2')                     --taking handle of position_hoop (i.e. red tree)
    
    orient_hoop_2 = sim.getObjectHandle('Orientation_hoop_2')               --taking handle of orientation_hoop (i.e. red tree)

    pos_hoop_1 = sim.getObjectHandle('Position_hoop1')                     --taking handle of position_hoop (i.e. red tree)
    
    orient_hoop_1 = sim.getObjectHandle('Orientation_hoop1')               --taking handle of orientation_hoop (i.e. red tree)
    
    obstacle_1 = sim.getObjectHandle('obstacle_1')               --taking handle of orientation_hoop (i.e. red tree)

    obstacle_2 = sim.getObjectHandle('obstacle_2')               --taking handle of orientation_hoop (i.e. red tree)

    arm_num = 1
    ------------Add the path planning task initial details---------------------

    --create a task object, used to represent the motion planning task
    t=simOMPL.createTask('t')

    --create a component of the state space for the motion planning problem
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,goal_1,{-1.92,-1.92,0},{2.09,1.97,2.0},1)}
    
    --set the state space of this task object
    simOMPL.setStateSpace(t,ss)
    
    --using the RRTConnect algorithm to compute path
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)        
    
    --set the collision pairs i.e. collider and collidee for the specified task object
    simOMPL.setCollisionPairs(t,{boundary,collection_handles})

    --------------------------------------------------------------------------

    -- Subscribing to the required topics
    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    
    --subscribes to topic /new_path/request to recieve request of computing path
    sub = simROS.subscribe('/new_path/request', 'std_msgs/Int16', 'path_callback')  

    -- subscribing to topic '/aruco_marker_publisher/markers' to obtain orientation of food trees 
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')

    -- subscribing to topic '/whycon/poses' to obtain position of food and non-food trees 
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')

    -- subscribing to topic '/input_key' to obtain input_key_value
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
    no_of_path_points_required= 40

    x_coor=0        -- x coordinate of whycon marker
    y_coor=0        -- y coordinate of whycon marker
    z_coor=0        -- z coordinate of whycon marker

    scale_factor_x = 0.18292    -- converting the whycon's x-coordinate into v-rep world frame
    scale_factor_y = -0.2078    -- converting the whycon's y-coordinate into v-rep world frame
    prev_time = os.clock()      -- store the time in seconds just after starting the simulation  

end

    -- Callback function for /new_path_request 
function path_callback(msg)
    target = msg.data       --recieves value of path number
    
end

    -- Callback function for /input_key
function key_callback(msg)
    input_key_value = msg.data   -- read the value of input key
    print(input_key_value)
end

    -- This function is used to send the Path computed in the real_world to whycon_world after transformation
    -- Message is to be sent in geometry_msgs/PoseArray as WhyCon node subscribes to that message
function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        pose = {position = b, orientation = a}

        
    -- Converting the coordinates from whycon world frame to vrep world frame
        
        pose.position.x = path[i]/scale_factor_x    
        pose.position.y = path[i+1]/scale_factor_y
        z_coor_after_scaling = translate(path[i+2],0,2,34,5)
        pose.position.z = math.abs(z_coor_after_scaling)
        
        pose.orientation.x =  path[i+4]
        pose.orientation.y =  path[i+5]
        pose.orientation.w =  path[i+6]
        pose.orientation.z =  path[i+7]

        sender.poses[math.floor(i/7) + 1] = pose
        
        --------------------------------------------------------------------------------------------------------------------
    end
    -- Debug if the path computed are correct. Display the computed path and see if the path points moves from drone to the target point
    return sender
end

    -- This function can be used to visualize the path you compute. This function takes path points as the argument...
    -- GO through the code segment for better understanding
function visualizePath(path)

    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end

    sim.addDrawingObjectItem(_lineContainer,nil)

    if path then
        print("Drawinwg path")
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

     -- This function is used to compute and publish the path to progress_task.py
function compute_and_send_path(task)
    
    local r
    local path
    r,path=simOMPL.compute(task,10,-1,no_of_path_points_required)  
    print(r)
    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)
    end
    return r
end

     -- This function recieves the handle of an object and gives its position and orientation
function getpose(handle,ref_handle)
    
    position = sim.getObjectPosition(handle,ref_handle)                           --getting the object's position
    orientation = sim.getObjectQuaternion(handle,ref_handle)                      --getting the object's orientation
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose

end

     -- Convert the z-coordinate from whycon world to vrep world
function translate(value, left_Min, left_Max, right_Min, right_Max)
    
    left_Span = left_Max - left_Min                                   -- Figure out how 'wide' each range is
    right_Span = right_Max - right_Min
    
    value_Scaled = ((value - left_Min)+0.0) / ((left_Span)+0.0)       -- Convert the left range into a 0-1 range (float)
    
    return right_Min + (value_Scaled * right_Span)                    -- Convert the 0-1 range into a value in the right range.

end

     -- This function round off the value upto given decimal places
function round(num, numDecimalPlaces)                            
    ----------------- rounding off upto given decimal places --------------------
    
    if numDecimalPlaces and numDecimalPlaces>0 then                
        local mult = 10^numDecimalPlaces
        return math.floor(num * mult + 0.5) / mult
    end
    
    return math.floor(num + 0.5)
end


function sysCall_actuation()

    -- Checks if the input key is "0"
    if (input_key_value == 15)then
        z_coor = sim.getObjectPosition(pos_hoop_1,-1)[3]                       -- z_coordinate of hoop in vrep world
        number=sim.setObjectPosition(pos_hoop_1,-1,{x_coor,y_coor,z_coor})     -- set the position of hoop
        input_key_value = 0
    end

    -- Checks if the input key is "2"
    if (input_key_value == 30)then
        z_coor = sim.getObjectPosition(pos_hoop_2,-1)[3]                       -- z_coordinate of hoop in vrep world
        number=sim.setObjectPosition(pos_hoop_2,-1,{x_coor,y_coor,z_coor})     -- set the position of hoop
        input_key_value = 0
    end

    -- Checks if the input key is "4"
    if (input_key_value == 40)then
        z_coor = sim.getObjectPosition(pos_hoop_3,-1)[3]                       -- z_coordinate of hoop in vrep world
        number=sim.setObjectPosition(pos_hoop_3,-1,{x_coor,y_coor,z_coor})     -- set the position of hoop
        input_key_value = 0
    end

    -- Checks if the input key is "m"
    if (input_key_value == 110)then
        z_coor = sim.getObjectPosition(obstacle_1,-1)[3]                       -- z_coordinate of obstacle in vrep world
        number=sim.setObjectPosition(obstacle_1,-1,{x_coor,y_coor,z_coor})     -- set the position of obstacle
        input_key_value = 0
    end

    -- Checks if the input key is "n"
    if (input_key_value == 120)then
        z_coor = sim.getObjectPosition(obstacle_2,-1)[3]                       -- z_coordinate of obstacle in vrep world
        number=sim.setObjectPosition(obstacle_2,-1,{x_coor,y_coor,z_coor})     -- set the position of obstacle
        input_key_value = 0
    end

    -- Checks if the input key is "1"
    if (input_key_value == 25)then
        sim.setObjectQuaternion(orient_hoop_1,-1,{y,x,-w,-z})                  -- set the orientation of hoop
        input_key_value = 0
    end

    -- Checks if the input key is "3"
    if (input_key_value == 35)then
        sim.setObjectQuaternion(orient_hoop_2,-1,{y,x,-w,-z})                  -- set the orientation of hoop
        input_key_value = 0
    end

    -- Checks if the input key is "5"
    if (input_key_value == 45)then
        sim.setObjectQuaternion(orient_hoop_3,-1,{y,x,-w,-z})                  -- set the orientation of hoop
        input_key_value = 0
    end

    -- Checks if the input key is "a"
    if (input_key_value == 70 and arm_num==1)then
        arm_num=2
        z_coor = 0.800
        number=sim.setObjectPosition(goal_1,-1,{x_coor,y_coor,z_coor})                  -- set the position of goal_1
        z_coor_after_scaling = translate(z_coor,34,5,0,2)                               -- calling function translate
        z_coor_after_scaling = math.abs(z_coor_after_scaling)                           -- converting coordinates into their absolute value.
        number=sim.setObjectPosition(Edrone,-1,{x_coor,y_coor,z_coor_after_scaling})    -- sets the position of drone according to coordinates of whycon marker   
        input_key_value = 4
    end

    -- Compute and send the path from goal_1 to goal_2
    if (target==1) then

        start_pose = getpose(goal_1,-1)                             -- Getting startpose
        goal_pose = getpose(goal_3,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
        
    -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
       
        target = 0
        input_key_value =4
    end

    -- Compute and send the path from goal_3 to goal_4
    if (target==2) then
       
        start_pose = getpose(goal_3,-1)                             -- Getting startpose
        goal_pose = getpose(goal_2,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
    
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
        
        target = 0
        input_key_value =4
    end

    -- compute and send the path from goal_5 to goal_2
    if (target==3) then
       
        start_pose = getpose(goal_2,-1)                             -- Getting startpose
        goal_pose = getpose(goal_7,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
    
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
        
        target = 0
        input_key_value =4
    end

    if (target==7) then
       
        start_pose = getpose(goal_7,-1)                             -- Getting startpose
        goal_pose = getpose(goal_6,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
    
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
        
        target = 0
        input_key_value =4
    end

    if (target==8) then
       
        start_pose = getpose(goal_6,-1)                             -- Getting startpose
        goal_pose = getpose(goal_3,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
    
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
        
        target = 0
        input_key_value =4
    end

    if (target==9) then
       
        start_pose = getpose(goal_2,-1)                             -- Getting startpose
        goal_pose = getpose(goal_1,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
    
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
        
        target = 0
        input_key_value =4
    end

    -- compute and send the path from goal_3 to goal_6
    if (target==4) then
       
        start_pose = getpose(goal_2,-1)                             -- Getting startpose
        goal_pose = getpose(goal_4,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
    
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
        
        target = 0
        input_key_value =4
    end

    -- compute and send the path from goal_7 to goal_3
    if (target==5) then

        start_pose = getpose(goal_4,-1)                             -- Getting startpose
        goal_pose = getpose(goal_5,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
        
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
       
        target = 0
        input_key_value =4
    end

    -- compute and send the path from goal_2 to goal_1
    if (target==6) then

        start_pose = getpose(goal_5,-1)                             -- Getting startpose
        goal_pose = getpose(goal_3,-1)                              -- Getting the goalpose
        simOMPL.setStartState(t,start_pose)                         -- Setting start state
        
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        
        --calls the function compute_and_send_path to compute the path and send it to topic /vrep/waypoints
        status = compute_and_send_path(t)
        
        if(status == true) then                                     -- path computed
            print('path sent')
        end
       
        target = 0
        input_key_value =4
    end

    -- runs when the every object is set up according to real world
    if (arm_num==2)then

        index = math.floor(z_coor)                                  -- store the integer value of z_coor
        z_coor_after_scaling = translate(z_coor,34,5,0,2)
        z_coor_after_scaling = math.abs(z_coor_after_scaling)
        
        --set the drone position after every 0.02 second and does not set the position of drone when whycon is not detected by camera
        if(os.clock()-prev_time >0.02 and index~=0)then
        
            -- sets the position of drone according to coordinates of whycon marker
            number=sim.setObjectPosition(Edrone,-1,{x_coor,y_coor,z_coor_after_scaling})       

            prev_time = os.clock()                                  -- store previous time in seconds
        end

    end
end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end


function aruco_callback(msg)
    -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
    
    -- obtains the orientation of food tree according to orientation of aruco marker
    x = msg['markers'][1]['pose']['pose']['orientation']['x']
    y = msg['markers'][1]['pose']['pose']['orientation']['y']
    z = msg['markers'][1]['pose']['pose']['orientation']['z']
    w = msg['markers'][1]['pose']['pose']['orientation']['w']
    
end


function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    
    -- obtains the postion of food and non-food tree according to position of whycon marker
    x_coor = msg['poses'][1]['position']['x']*scale_factor_x        --convert the x_coordinate from whycon world to vrep world 
    y_coor = msg['poses'][1]['position']['y']*scale_factor_y        --convert the y_coordinate from whycon world to vrep world
    z_coor = msg['poses'][1]['position']['z']                       
    x_coor = round(x_coor,2)                                        -- round off x_coor upto  decimal places
    y_coor = round(y_coor,2)                                        -- round off y_coor upto  decimal places
end

