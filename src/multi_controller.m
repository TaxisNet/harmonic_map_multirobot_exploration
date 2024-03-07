    rosshutdown
    
    try
    cellfun(@delete, timer_cell)
    catch
        disp("error deleting previous timers");
    end

%
%for custom ros message
addpath('~/catkin_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')

%clear classes
rehash toolboxcache


rosinit

global hm_cell q_front_cell robot_pos_cell map_frame_cell boundary_msg namespace K_ang K_lin  isMerged
K_ang = 0.2;
K_lin = 0.22;



N = 3;
hm_cell = cell(1,N);
namespace = cell(1,N);
robot_pos_cell = cell(1,N);
q_front_cell = cell(1,N);

boundary_info_sub = cell(1,N);
velocity_pub = cell(1,N);
boundary_msg = cell(1,N);
timer_cell = cell(1,N);

map_frame_cell = cell(1,N);
pos_handle = cell(1,N);
merged_sub = rossubscriber('/map_merge/is_merged', 'std_msgs/UInt16MultiArray', @updateIsMerged, DataFormat='struct');
isMerged = cell(1,N);

%figure position
MP = get(0,'MonitorPositions');
MP = MP(2,:);
SCREEN_WIDTH = MP(1);
SCREEN_HEIGHT = MP(4) + MP(2);
FIG_WIDTH = 600;
FIG_HEIGHT = floor(MP(4)/N);

for k=1:N
hm_cell{k} = HarmonicMap();
hm_cell{k}.samplesPerUnit = 20;
hm_cell{k}.fig = figure(k);
hm_cell{k}.fig.Position = [SCREEN_WIDTH SCREEN_HEIGHT-k*FIG_HEIGHT FIG_WIDTH FIG_HEIGHT-40];
hm_cell{k}.fig.ToolBar = 'none';
hm_cell{k}.fig.MenuBar = 'none';
namespace{k} = strcat("tb3_", string(k-1));
map_frame_cell{k} = strcat(namespace{k}, '/map');
boundary_info_sub{k} = rossubscriber(strcat(namespace{k},'/boundary_info'),'boundary_compute/boundary_info', {@callback, k}, DataFormat='struct');
velocity_pub{k} = rospublisher(strcat(namespace{k},'/cmd_vel'),'geometry_msgs/Twist', DataFormat='struct');


%set timer
timer_cell{k} = timer;
timer_cell{k}.TimerFcn = {@setHMBoundaries, k};
timer_cell{k}.ExecutionMode = "fixedRate";
timer_cell{k}.Period = 2.5; %secs
timer_cell{k}.StartDelay = 4;
timer_cell{k}.start()
end

%format figure order
for k =1:N
    figure(hm_cell{N+1-k}.fig.Number)
end

tftree = rostf("DataFormat","struct");
pause(2)

rate = rosrate(10);
for i=1:N
    robotPosMsg = getTransform(tftree, map_frame_cell{i}, strcat( namespace{i}, '/base_footprint'));
    robotPos = [robotPosMsg.Transform.Translation.X; robotPosMsg.Transform.Translation.Y];
    robot_pos_cell{i}=robotPos;
end

while(1)
    for i=1:N
        robotPosMsg = getTransform(tftree, map_frame_cell{i}, strcat( namespace{i}, '/base_footprint'));
        if(isempty(robotPosMsg))
            continue
        end
        
        msg = findTwist(i, robotPosMsg);
        send(velocity_pub{i},msg);
        % disp('running..'+string(i))
    end
   
    waitfor(rate);
end


function twistMsg = findTwist(robot_num, robotPosMsg)
global hm_cell q_front_cell robot_pos_cell 
    twistMsg = rosmessage('geometry_msgs/Twist', "DataFormat","struct");
    if(~isempty(hm_cell{robot_num}.frontiers_q) && ~isempty(q_front_cell{robot_num}))
        
        robotPos = [robotPosMsg.Transform.Translation.X; robotPosMsg.Transform.Translation.Y];
        robot_pos_cell{robot_num}=robotPos;
        robotQuat = robotPosMsg.Transform.Rotation;
        robotQuat = [robotQuat.W robotQuat.X robotQuat.Y robotQuat.Z];
        try
            [q,J]= hm_cell{robot_num}.compute(robot_pos_cell{robot_num});
        catch ME
            disp(ME.message)
            return 
        end
        if (isnan(rcond(J)))
            return  
        end
        dq = (q-q_front_cell{robot_num});
        dx=-inv(J)* dq;
        desired_vel = dx/(norm(dx)+0.001);
    
    
        %%%%plot pos%%%%%
        % try q
        %     set(0,'CurrentFigure',hm_cell{robot_num}.fig);
        %     subplot(121)
        %     hold on
        %     delete(pos_handle{robot_num})
        %     pos_handle{robot_num} = plot(robotPos(1), robotPos(2), 'rsquare');
        %     %quiver(robotPos(1), robotPos(2), desired_vel(1), desired_vel(2), 1)
        %     hold off
        %     %%%%
        % catch
        % end
        
        [linVel, angVel] = velocityController(robotQuat, desired_vel, norm(dq));
        twistMsg.Linear.X = double(linVel);
        twistMsg.Angular.Z = double(angVel);

    end
    return
end

function callback(~,msg, robot_num)
global boundary_msg
    if(~msg.CompFailed)
        boundary_msg{robot_num} = msg;
        %setHMBoundaries([],[], robot_num)
    end
end


function setHMBoundaries(~, ~, robot_num)
    global hm_cell q_front_cell robot_pos_cell boundary_msg isMerged
    
    disp('Timer '+string(robot_num)                                     )
    if(isempty(boundary_msg{robot_num}))
        disp("No boundary is set for robot"+string(robot_num))
    else
        [boundaries, isFree, ~] = parseBoundaries(boundary_msg{robot_num});
        % check if order is counter clock wise.
        % Looks like the outter is and the inner aren't
        % The order depends on the ouput of cv.findContours
        % But documentation isn't very clear
        % Check out:  https://answers.opencv.org/question/170874/cvfindcontours-orientation/
        try
        for i=1:length(boundaries)
            
                if(~determinePointOrder(boundaries{i}))
                    boundaries{i} = flip(boundaries{i},1);
                    isFree{i} = flip(isFree{i},1);
                end
                 %calculate transform
        %tic
        hm_cell{robot_num}.setBoundaries(boundaries,isFree);
        %toc;
        hm_cell{robot_num}.plotMap()
        end
            catch ME
                disp(ME.message)
            
        end
        
        
        if(isempty(hm_cell{robot_num}.frontiers_q))
            disp("Exporation Done!")
            return
        end
         try
             
           q_d = hm_cell{robot_num}.getNearestFrontier(robot_pos_cell{robot_num});
        catch
            disp("Error finding nearest frontier")
            row_num = size(hm_cell{robot_num}.frontiers_q, 1);
            q_d = hm_cell{robot_num}.frontiers_q(randi(row_num),:)';
         end


       

       
       %chack if frontiers have s
        hasPriority = isMerged{robot_num};
        hasPriority = hasPriority(hasPriority<robot_num);
       
        tol = deg2rad(7);
        angle_d = atan2(q_d(2), q_d(1));
        for i = 1:length(double(hasPriority))
            if(~isempty(q_front_cell{hasPriority(i)}) && abs(angle_d - atan2(q_front_cell{hasPriority(i)}(2),q_front_cell{hasPriority(i)}(1)))<tol)
                
                dist = norm(robot_pos_cell{robot_num}- robot_pos_cell{hasPriority(i)});
                if(dist>5)
                    %to far away so no need to block asigmend
                    continue
                end

                %same front find one that isn't taken
                taken_frontiers = [q_front_cell{hasPriority}]';

                available_frontiers = hm_cell{robot_num}.frontiers_q;
                %remove the taken ones from the available
                angle_avail = atan2(available_frontiers(:,2),available_frontiers(:,1));
                angle_taken = atan2(taken_frontiers(:,2),taken_frontiers(:,1));
                taken_indexs = ismembertol(angle_avail, angle_taken,tol);
                available_frontiers = available_frontiers(~taken_indexs,:);
                %pick one : random
                if size(available_frontiers, 1) == 0
                    q_d = [];
                    disp(string(robot_num)+": no available_frontiers")
                    q_front_cell{robot_num}  = q_d; 
                    return
                else
                    q_d = available_frontiers(randi(size(available_frontiers, 1)), :)';
                    disp(string(robot_num)+": random frontier")
                end
                
                break
            end
        end

        
        q_front_cell{robot_num}  = q_d;   
        set(0,'CurrentFigure',hm_cell{robot_num}.fig);
        subplot(122)
        hold on
        plot(q_d(1), q_d(2), 'bx', MarkerSize=12)
        hold off
       

    end
end

function updateIsMerged(~,msg)
    global isMerged map_frame_cell
    
    arr = msg.Data;
    arr = arr+1;
    map_frame_cell{arr(1)}= 'world';
     
    %set isMerged cell
    isMerged{arr(1)} = unique([isMerged{arr(1)}, arr(2)]);

    disp('Map merged')
end

function isCounterClockwise = determinePointOrder(points)
% Ensure that the input matrix has at least 3 points
if size(points, 1) < 3
    error('At least 3 points are required to determine the order.');
end

% Calculate the signed area of the polygon
x= points(:,1);
y=points(:,2);
% Shift the arrays to compute the cross product efficiently
x_shifted = circshift(x, -1);
y_shifted = circshift(y, -1);

signedArea = sum(x .* y_shifted - x_shifted .* y);

% Determine the order based on the sign of the signed area
isCounterClockwise =  (signedArea > 0);
end

function [boundaries, isFree, pos] = parseBoundaries(msg)
indxs = [0; msg.BoundaryIndex];
n = length(msg.BoundaryIndex);
boundaries = cell(n,1);
isFree= cell(n,1);

for i=1:n
    boundaries{i} = [msg.Xl(indxs(i)+1:indxs(i+1)), msg.Yl(indxs(i)+1:indxs(i+1));
        msg.Xl(indxs(i)+1), msg.Yl(indxs(i)+1)];
    boundaries{i}= double(boundaries{i});
    boundaries{i} = msg.MapResolution*boundaries{i}  - double([msg.MapX0, msg.MapY0]);
    
    isFree{i} = [msg.Isfree(indxs(i)+1:indxs(i+1));  msg.Isfree(indxs(i)+1)];
    
    pos = msg.MapResolution*double([msg.PosX; msg.PosY]) - double([msg.MapX0; msg.MapY0]);
    
end
end


function [linVel, angVel] = velocityController(quat, desired_vel, q_dist)
    global K_ang K_lin

    eul_angles = quat2eul(quat, 'XYZ');
    yaw = eul_angles(3);
    yaw_d = atan2(desired_vel(2),desired_vel(1));

    delta_yaw = yaw_d - yaw;

    if(abs(delta_yaw)>pi)
        delta_yaw = -sign(delta_yaw)*mod(abs(delta_yaw),pi);
    end

    %disp(rad2deg(delta_yaw));

    angVel = K_ang*delta_yaw; % sin(delta_yaw)

    %linVel = K_lin*norm(desired_vel);

    %turningCoef = max((1-((delta_yaw)/(pi/2)).^4),0);
    turningCoef = abs(delta_yaw)<(pi/6);
    linVel = K_lin* turningCoef*norm(desired_vel);

    if(turningCoef)
        angVel = 2*K_ang*delta_yaw;
    end
        
    if nargin>2
        damping_dist = 0.05;
        damping_factor = (1/(2*damping_dist))*q_dist;
        damping_factor = min(damping_factor,1);
        linVel = damping_factor*linVel;
    end
end
