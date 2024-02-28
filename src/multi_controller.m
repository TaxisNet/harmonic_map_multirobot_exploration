    rosshutdown
%
%for custom ros message
addpath('../../matlab_msg_gen_ros1/glnxa64/install/m')

clear classes
rehash toolboxcache


rosinit

global hm_cell q_front_cell robot_pos_cell map_frame_cell boundary_msg namespace K_ang K_lin  isMerged
K_ang = 0.4;
K_lin = 0.25;

isMerged = false;

N = 2;
hm_cell = cell(1,N);
namespace = cell(1,N);
robot_pos_cell = cell(1,N);
q_front_cell = cell(1,N);

boundary_info_sub = cell(1,N);
velocity_pub = cell(1,N);
boundary_msg = cell(1,N);


map_frame_cell = cell(1,N);
pos_handle = cell(1,N);
merged_sub = rossubscriber('/map_merge/is_merged', 'std_msgs/UInt16MultiArray', @updateIsMerged, DataFormat='struct');

for k=1:N
hm_cell{k} = HarmonicMap();
hm_cell{k}.samplesPerUnit = 30;
hm_cell{k}.fig = figure(k);
namespace{k} = strcat("tb3_", string(k-1));
map_frame_cell{k} = strcat(namespace{k}, '/map');
boundary_info_sub{k} = rossubscriber(strcat(namespace{k},'/boundary_info'),'boundary_compute/boundary_info', {@callback, k}, DataFormat='struct');
velocity_pub{k} = rospublisher(strcat(namespace{k},'/cmd_vel'),'geometry_msgs/Twist', DataFormat='struct');
end


tftree = rostf("DataFormat","struct");
pause(1)

rate = rosrate(10);

while(1)
    for i=1:N
        twistMsg = rosmessage(velocity_pub{i});
        if(~isempty(hm_cell{i}.frontiers_q) && ~isempty(q_front_cell{i}))
            robotPosMsg = getTransform(tftree, map_frame_cell{i}, strcat( namespace{i}, '/base_footprint'));
            robotPos = [robotPosMsg.Transform.Translation.X; robotPosMsg.Transform.Translation.Y];
            robot_pos_cell{i}=robotPos;
            robotQuat = robotPosMsg.Transform.Rotation;
            robotQuat = [robotQuat.W robotQuat.X robotQuat.Y robotQuat.Z];
            
            % robotPos=[0;0];
            % robotQuat= zeros(1,4);
            desired_vel = hm_cell{i}.getFieldVelocity(robot_pos_cell{i},q_front_cell{i});
            %no q given-> gets nearest (in q-space)frontier
             
        %%%%plot pos%%%%%
        set(0,'CurrentFigure',hm_cell{i}.fig);
        subplot(121)
        hold on
        delete(pos_handle{i})
        pos_handle{i} = plot(robotPos(1), robotPos(2), 'rsquare');
        %quiver(robotPos(1), robotPos(2), desired_vel(1), desired_vel(2), 1)
        hold off
        %%%%
            
            [linVel, angVel] = velocityController(robotQuat, desired_vel);
            
            if(~isnan(linVel+angVel))
                twistMsg.Linear.X = double(linVel);
                twistMsg.Angular.Z = double(angVel);
            end
        end
        send(velocity_pub{i},twistMsg);
    end
    waitfor(rate);
end


function callback(~,msg, robot_num)
global boundary_msg
    if(~msg.CompFailed)
        boundary_msg{robot_num} = msg;
    end
    OLD_callback(robot_num) 
end


function OLD_callback(robot_num)
    global hm_cell q_front_cell robot_pos_cell boundary_msg

    if(~boundary_msg{robot_num}.CompFailed)
        [boundaries, isFree, ~] = parseBoundaries(boundary_msg{robot_num});
        % check if order is counter clock wise.
        % Looks like the outter is and the inner aren't
        % The order depends on the ouput of cv.findContours
        % But documentation isn't very clear
        % Check out:  https://answers.opencv.org/question/170874/cvfindcontours-orientation/
        for i=1:length(boundaries)
            try
                if(~determinePointOrder(boundaries{i}))
                    boundaries{i} = flip(boundaries{i},1);
                    isFree{i} = flip(isFree{i},1);
                end
            catch ME
                disp(ME.message)
            end
        end
        
        %calculate transform
        tic
        hm_cell{robot_num}.setBoundaries(boundaries,isFree);
        toc
        hm_cell{robot_num}.plotMap()

      

        
        if(isempty(hm_cell{robot_num}.frontiers_q))
            disp("Exporation Done!")
            return
        end
        
        %todo: intergrate


        % if(isMerged && (norm(q_front_cell{1}-q_front_cell{robot_num}) < 1e-9))
        %     if(size(hm_cell{robot_num}.frontiers_q,2) == 1)
        %         disp("Last Frontier is explored by other robot!")
        %         return
        %     else
        %         for i=1:size(hm_cell{robot_num}.frontiers_q,2)
        %             if(norm(hm_cell{robot_num}.frontiers_q(i,:)'- q_front_cell{robot_num}) >1e-9)
        %                  q_front_cell{robot_num} = hm_cell{robot_num}.frontiers_q(i,:);
        %                  return
        %             end
        %         end
        %     end
        % end
        

        try
            robotPosMsg = getTransform(tftree, map_frame_cell{i}, strcat( namespace{i}, '/base_footprint'));
            robotPos = [robotPosMsg.Transform.Translation.X; robotPosMsg.Transform.Translation.Y];
            q_front_cell{robot_num} = hm_cell{robot_num}.getNearestFrontier(robot_pos_cell{robot_num}, true);
        catch
            disp("Error finding nearest frontier")
            q_front_cell{robot_num} = hm_cell{robot_num}.frontiers_q(1,:)';
        end        
    end
end

function updateIsMerged(~,msg)
    global isMerged map_frame_cell
    isMerged = true;
    arr = msg.Data;
    arr = arr+1;
    map_frame_cell{arr(1)}= 'world';
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


function [linVel, angVel] = velocityController(quat, desired_vel)
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
    turningCoef = abs(delta_yaw)<(pi/3);
    linVel = K_lin* turningCoef*norm(desired_vel);
end
