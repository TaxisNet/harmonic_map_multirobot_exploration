rosshutdown

%for custom ros message 
addpath('../../matlab_msg_gen_ros1/glnxa64/install/m')

clear classes
rehash toolboxcache


rosinit

% globals
global hm K_ang K_lin q_front robotPos
hm = HarmonicMap();
K_ang = 0.5;
K_lin = 0.08;

%ROS
% node = ros.Node('/matlab_node');
% sub = ros.Subscriber(node,'boundary_info','boundary_compute/boundary_info', @callback , DataFormat='struct');
% vel_pub = ros.Publisher(node, '/cmd_vel','geometry_msgs/Twist');

sub = rossubscriber('/boundary_info','boundary_compute/boundary_info', @callback , DataFormat='struct');
vel_pub = rospublisher('/cmd_vel','geometry_msgs/Twist', DataFormat='struct');


tftree = rostf("DataFormat","struct");
pause(0.5)

rate = rosrate(10);
while(1)
    twistMsg = rosmessage(vel_pub);
    if(~isempty(hm.frontiers_q))
        robotPosMsg = getTransform(tftree, 'map', 'base_footprint');
        
        robotPos = [robotPosMsg.Transform.Translation.X; robotPosMsg.Transform.Translation.Y];
        
        robotQuat = robotPosMsg.Transform.Rotation;
        robotQuat = [robotQuat.W robotQuat.X robotQuat.Y robotQuat.Z];
        
        
        desired_vel = hm.getFieldVelocity(robotPos,q_front); 
        %no q given-> gets nearest (in q-space)frontier
        

        [linVel, angVel] = velocityController(robotQuat, desired_vel);
        
        if(~isnan(linVel+angVel))
            twistMsg.Linear.X = double(linVel);
            twistMsg.Angular.Z = double(angVel);
        end
    end
    send(vel_pub,twistMsg);
    waitfor(rate);
end


function callback(~,msg)
    global hm q_front robotPos

    if(~msg.CompFailed)
        [boundaries, isFree, ~] = parseBoundaries(msg);
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
        hm.setBoundaries(boundaries,isFree);
        toc
        hm.plotMap
       
        if(isempty(hm.frontiers_q))
            disp("Exporation Done!")
            return
        end

        try
            q_front = hm.getNearestFrontier(robotPos, true);
        catch
            disp("Error finding nearest frontier")
            q_front = hm.frontiers_q(1,:)';
        end
        
    end
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
    
    turningCoef = max((1-((delta_yaw)/(pi/2)).^4),0);
    linVel = K_lin* turningCoef*norm(desired_vel);
end



