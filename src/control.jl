function wrap(θ)
    mod(θ += pi, 2*pi) - pi
end

function clip(x, l)
    max(-l, min(l, x))
end

function keyboard_controller(KEY::Channel, 
                             CMD::Channel, 
                             SENSE::Channel, 
                             EMG::Channel;
                             K1=5, 
                             K2=.5, 
                             disp=false, 
                             V=0.0, 
                             θ = 0.0, 
                             V_max = 100.0, 
                             θ_step=0.1, 
                             V_step = 1.5)
    println("Keyboard controller in use.")
    println("Press 'i' to speed up,")
    println("Press 'k' to slow down,")
    println("Press 'j' to turn left,")
    println("Press 'l' to turn right.")

    while true
        sleep(0)
        @return_if_told(EMG)
        
        key = @take_or_default(KEY, ' ')
        meas = @fetch_or_continue(SENSE)

        speed = meas.speed
        heading = meas.heading
        segment = meas.road_segment_id
        if key == 'i'
            V = min(V_max, V+V_step)
        elseif key == 'j' 
            θ += θ_step
        elseif key == 'k'
            V = max(0, V-V_step)
        elseif key == 'l'
            θ -= θ_step
        end
        err_1 = V-speed
        err_2 = clip(θ-heading, π/2)
        cmd = [K1*err_1, K2*err_2]
        @replace(CMD, cmd)
        if disp
            print("\e[2K")
            print("\e[1G")
            @printf("Command: %f %f, speed: %f, segment: %d", cmd..., speed, segment)
        end
    end
end

# sorts cars and gives closest 
function closest_cars(ids, ego, movables, n, avoid_distance, rear_distance, follow_distance, road, lanewidth)
    dists = []
    for (i, m) ∈ movables
        # return the vehicles that are very close, and if they are in the same lane return them when they're further
        if(curLane(ego, road, lanewidth) == m.target_lane && carAhead(ego, m, road) == false)
            if(norm(ego.position-m.position) < rear_distance)
                push!(dists, (norm(ego.position-m.position), i))
            end
        elseif(curLane(ego, road, lanewidth) == m.target_lane && carAhead(ego, m, road) == true)
            if(norm(ego.position-m.position) < follow_distance)
                push!(dists, (norm(ego.position-m.position), i))
            end
        else
            if(norm(ego.position-m.position) < avoid_distance)
                push!(dists, (norm(ego.position-m.position), i))
            end
        end
    end
    sort!(dists; alg=Base.Sort.PartialQuickSort(n), by=x->x[1])
    for i ∈ 1:length(dists)
        push!(ids, dists[i][2])
    end
end

#takes in ego and lets us know what lane it is currently in (curved segment)
function curLane(ego, road::CurvedSegment, lanewidth)
    x = norm(ego.position - road.center)
    lane1left = road.radius
    lane1right = lane1left + lanewidth
    lane2right = lane1left + 2 * lanewidth
    lane3right = lane1left + 3 * lanewidth
    if(lane1left <= x && x <= lane1right)
        return 1
    elseif(lane1right <= x && x <= lane2right)
        return 2
    elseif(lane2right <= x && x <= lane3right)
        return 3
    end
end

#NEED TO FIX
#takes in ego and lets us know what lane it is currently in (straight segment)
function curLane(ego, road::StraightSegment, lanewidth)
    print(ego.position)
    print(road.start)
    print(road.finish)
    x = ego.position[1]
    lane1left = road.start[1]
    lane1right = lane1left + lanewidth
    lane2right = lane1left + 2 * lanewidth
    lane3right = lane1left + 3 * lanewidth
    if(lane1left <= x && x <= lane1right)
        return 1
    elseif(lane1right <= x && x <= lane2right)
        return 2
    elseif(lane2right <= x && x <= lane3right)
        return 3
    else 
        return 2
    end
end

#takes in ego and fleet and determines which one is actually in front (curved segment)
function carAhead(ego, fleet, road::CurvedSegment)
    center = road.center
    ego_delta = ego.position - center
    fleet_delta = fleet.position - center
    ego_theta = atan(ego_delta[2], ego_delta[1])
    fleet_theta = atan(fleet_delta[2], fleet_delta[1])
    angular_dist = wrap(fleet_theta - ego_theta)
    if(wrap(angular_dist) > 0)
        return true
    else
        return false
    end
end

#takes in ego and fleet and determines which one is actually in front (straight segment)
function carAhead(ego, fleet, road::StraightSegment)
    if(ego.position[1] < fleet.position[1])
        return true
    else
        return false
    end
end

# determines if lane switch should happen to right or left
function switchlanes(ego, fleet, cartoright, cartoleft, road, command, K₁, K₂)  
    lane = curLane(ego, road.segments[1], road.lanewidth)      
    switchlane = 0
    if(cartoright[2] == true && cartoleft[2] == true)
        #both lanes occupied
        #println("need to switch lanes but can't, other lanes occupied")
    elseif(cartoleft[2] == true)
        if(lane == 3)
            #println("lane switch needed to right, impossible")
        else
            #println("lane switch needed to right")
            switchlane = 1
        end
    elseif(cartoright[2] == true)
        if(lane == 1)
            #println("lane switch needed to left, impossible")
        else
            #println("lane switch needed to left")
            switchlane = -1
        end
    else
        # both lanes free
        if(lane == 3)
            switchlane = -1
        elseif(lane == 1)
            switchlane = 1
        else
            if(cartoright[1] > 0 && cartoleft[1] > 0)
                if(norm(ego.position - fleet[cartoright[1]].position) - norm(ego.position - fleet[cartoleft[1]].position) > 0)
                    switchlane = 1
                else
                    switchlane = -1
                end
            elseif(cartoright[1] < 0 && cartoleft[1] > 0)
                switchlane = 1
            elseif(cartoright[1] > 0 && cartoleft[1] < 0)
                switchlane = -1
            elseif(cartoright[1] < 0 && cartoleft[1] < 0)
                switchlane = 1
            end
        end
    end
    switchlane
end

#plots velocity portion of trajectory
function plot_traj(ego, fleet, command, velocity_step, road, step_size, K₁, K₂)
    lane = curLane(ego, road.segments[1], road.lanewidth)
    switchlane = 0
    # getting indices of nearby cars [index of car, whether car has been found behind/infront]
    carbehind = [-1, false]
    carinfront = [-1, false]
    cartoleft = [-1, false]
    cartoright = [-1, false]
    for i in 1:length(fleet)
        if(fleet[i].target_lane == lane)
            #NEED TO FIX to get the actual one that's in front
            if(carAhead(ego, fleet[i], road.segments[1]) == true)
                # pick the one with the lower velocity in the event there are two cars in front 
                if(carinfront[2] == true)
                    if(fleet[carinfront[1]].speed > fleet[i].speed)
                        carinfront = [i, true]
                    end
                else
                    carinfront = [i, true]
                end
            elseif(carAhead(ego, fleet[i], road.segments[1]) == false)
                # pick the one with the higher velocity in the event there are two cars behind 
                if(carbehind[2] == true)
                    if(fleet[carbehind[1]].speed < fleet[i].speed)
                        carbehind = [i, true]
                    end
                else
                    carbehind = [i, true]
                end
            end
        elseif(fleet[i].target_lane == lane + 1)
            cartoright = [i, true]
        elseif(fleet[i].target_lane == lane - 1)
            cartoleft = [i, true]
        end
    end

    #if cars are ahead and behind
    if(carinfront[2] == true && carbehind[2] == true)
        #println("cars ahead and behind")
        frontspeed = fleet[carinfront[1]].speed
        backspeed = fleet[carbehind[1]].speed
        # back car is too fast for the ego
        if(backspeed > ego.speed)
            if(frontspeed > backspeed)
            #if front car is fast enough, accelerate
                command[1] = (frontspeed - ego.speed)*step_size
            else
                switchlane = switchlanes(ego, fleet, cartoright, cartoleft, road, command, K₁, K₂)
                command[1] = (frontspeed + backspeed)/2*step_size
                #println("car behind is too fast for car in front");
            end
        #back car is slower than the ego
        elseif(frontspeed > ego.speed)
            command[1] = velocity_step*step_size
        else
            command[1] -= max(ego.speed - frontspeed, ego.speed - backspeed)*step_size
            switchlane = switchlanes(ego, fleet, cartoright, cartoleft, road, command, K₁, K₂)
            #println("car ahead and behind are too slow")
        end

    # if there is only a car ahead
    elseif(carinfront[2] == true)
        #println("cars ahead only")
        frontspeed = fleet[carinfront[1]].speed
        # if car ahead is faster, speed up a regular amount
        if(frontspeed > ego.speed)
            command[1] = velocity_step*step_size
        else
            # otherwise slow down, maybe change lane?
            command[1] -= (ego.speed - frontspeed)
            #lane change
            switchlane = switchlanes(ego, fleet, cartoright, cartoleft, road, command, K₁, K₂)
        end

    # if there is only a car behind
    elseif(carbehind[2] == true)
        #println("cars behind only")
        backspeed = fleet[carbehind[1]].speed
        # if car behind is faster, speed up by the diff in speeds
        if(backspeed > ego.speed)
            command[1] = backspeed - ego.speed
            switchlane = switchlanes(ego, fleet, cartoright, cartoleft, road, command, K₁, K₂)
        else
            #otherwise, speed up a regular amount
            command[1] = velocity_step*step_size
        end
    # no cars around
    else 
        command[1] = velocity_step*step_size 
    end
    switchlane
end

function controller(CMD::Channel, 
                    SENSE::Channel, 
                    SENSE_FLEET::Channel, 
                    EMG::Channel,
                    road;
                    disp=false, 
                    step_size = 0.02
                    )
    K₁= K₂ =.5
    
    while true
        sleep(step_size)
        @return_if_told(EMG)
        ego_meas = fetch(SENSE)
        fleet_meas = fetch(SENSE_FLEET)

        #get the closest cars
        num_viewed = length(fleet_meas)
        closest_ids = []
        avoid_distance = 8.0
        follow_distance = 18.0
        rear_distance = 24.0
        closest_cars(closest_ids, ego_meas, fleet_meas, num_viewed, avoid_distance, rear_distance, follow_distance, road.segments[1], road.lanewidth)
        fleet = Any[]
        for i in 1:length(closest_ids)
            push!(fleet, fleet_meas[closest_ids[i]])
        end
        command = [0.0,0.0]

        lane = curLane(ego_meas, road.segments[1], road.lanewidth)
        # change velocity and determine if lane change is required
        switchlane = 0
        velocity_step = 15
        switchlane = plot_traj(ego_meas, fleet, command, velocity_step, road, step_size, K₁, K₂)
        
        #calculating heading to follow current lane, or switch to given lane
        seg = road.segments[1]
        if(switchlane == -1 || switchlane == 1)
            newlane = lane + switchlane
            cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, newlane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            command[2] = max(min(δ, π/30.0), -π/30.0)
        else
            cte, ctv = get_crosstrack_error(ego_meas.position, ego_meas.heading, ego_meas.speed, lane, seg, road.lanes, road.lanewidth)
            δ = -K₁*cte-K₂*ctv
            command[2] = max(min(δ, π/4.0), -π/4.0)        
        end
        @replace(CMD, command)

        #if disp
         #   print("\e[2K")
          #  print("\e[1G")
           # @printf("Command: %f %f, speed: %f, segment: %s", command..., ego_meas.speed, seg)
        #end
    end
end