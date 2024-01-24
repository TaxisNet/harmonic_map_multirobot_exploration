classdef HarmonicMap < handle
    properties
        %states
        boundaries;
        centers;
        normals;
        controlPoints;
        
        %for plotting frontiers
        isFree  

        %no of elements per boundary
        totalElements;
        
        %augmented matrix
        A;
        %weights 
        Cx;
        Cy;
        
        %Outter boundary mapping angle
        theta;
        %the frontier points on the unit circle outter boundary
        frontiers_q = double.empty(0,2);
        
        %obstacles transform Nx2 matrix
        innerObst_q = double.empty(0,2);
        
        %figure
        fig

        %maximun time for movement simulation
        maxTime = 10
        
        % Controls samples per unit. Outter has more that inner by 
        % innerObstacleSampleModifier
        % this might need tuning
        samplesPerUnit = 60;
        innerObstacleSampleModifier = 0.25;
        
        
        %to avoid errors
        isMapSolved logical
    end
    
    methods (Access = private)
        function upscaledBoundaries(obj, lowDefinitionBoundaryCell)
            %upscales boundaries and calculates theta
            upscaledBoundary = cell(0);

            for i =1:length(lowDefinitionBoundaryCell)
                %upscale the boundary
                currentBoundary = lowDefinitionBoundaryCell{i};

                len = [0; cumsum(sqrt(sum(diff(currentBoundary,1,1).^2, 2)))];
                total_len = len(end);
                if(i==1)
                    NofS = max(round(obj.samplesPerUnit*total_len),4);
                else
                    NofS = max(round(obj.innerObstacleSampleModifier*obj.samplesPerUnit*total_len),4);
                end
                t = linspace(0,total_len,NofS);

                upscaledBoundary{i} = interp1(len,currentBoundary,t,'linear');

                if(i==1)
                    upscaledCumulativeLen = [cumsum(sqrt(sum(diff(upscaledBoundary{i} ,1,1).^2, 2)))];
                    obj.theta = 2*pi*(upscaledCumulativeLen./upscaledCumulativeLen(end));
                end
            end
            obj.boundaries = upscaledBoundary;
        end

        function findFrontiers(obj, lowDefinitionBoundaryCell, isFree)
            %upscales boundaries, calculates theta AND finds the frontier
            %points
            upscaledBoundary = cell(0);
            upscaledIsFree = cell(0);

            for i =1:length(lowDefinitionBoundaryCell)
                %upscale the boundary
                currentBoundary = lowDefinitionBoundaryCell{i};

                len = [0; cumsum(sqrt(sum(diff(currentBoundary,1,1).^2, 2)))];
                total_len = len(end);
                if(i==1)
                    NofS = max(round(obj.samplesPerUnit*total_len),4);
                else
                     NofS = max(round(obj.innerObstacleSampleModifier*obj.samplesPerUnit*total_len),4);
                end
                t = linspace(0,total_len,NofS);

                upscaledBoundary{i} = interp1(len,currentBoundary,t,'linear');
                
                %find frontier points for outter boundary
                %TODO update transform to incude inner boundarie frontiers
                upscaledIsFree{i} = interp1(len, single(isFree{i}),t,"nearest");
                upscaledIsFree{i} = logical(upscaledIsFree{i});

                if(i==1)
                    upscaledLen = [0; (sqrt(sum(diff(upscaledBoundary{i} ,1,1).^2, 2)))];
                    upscaledLen(upscaledIsFree{i}) = 0;
                    upscaledCumulativeLen = [cumsum(upscaledLen)];
                    obj.theta = 2*pi*(upscaledCumulativeLen(2:end)./upscaledCumulativeLen(end));

                    %get angle of tranformed collapsed frontier points q
                    frontiers_theta = obj.theta(upscaledIsFree{i}(2:end));
                    %clear the duplicate
                    frontiers_theta = uniquetol(frontiers_theta);

                    obj.frontiers_q = [cos(frontiers_theta), sin(frontiers_theta)];
                end
            end
            obj.boundaries = upscaledBoundary;
            obj.isFree = upscaledIsFree;            
        end
           
        function set_states(obj)
            %set states: centers, normals, control_points, outer_lengths, total_elements
            dist = 1e-5;
        
            obj.centers = cell(1,length( obj.boundaries));
            obj.normals = cell(1,length( obj.boundaries));
            obj.controlPoints = cell(1,length( obj.boundaries));
            
            for i = 1:length( obj.boundaries)
                
                a =  obj.boundaries{i}(1:end-1,:);
                b =  obj.boundaries{i}(2:end,:);
        
               
                obj.centers{i} = 0.5*(a+b);
        
                if(i == 1)
                    %clockwise for outer
                    rot_matrix = [0 -1; 1 0];
                else
                    %counter-clockwise for inner
                    rot_matrix = [0 1; -1 0];
                end
        
        
                
                norm_vec = (b-a)*rot_matrix';
                %normalize
                norm_vec = norm_vec./sqrt(sum(norm_vec.^2,2));
                obj.normals{i} = norm_vec;

                %add adjacent normals and normalize again
                point_normals = norm_vec+circshift(norm_vec,1);
                point_normals = point_normals./sqrt(sum(point_normals.^2,2));
                
                %last & first point have the same normal
                point_normals = [point_normals; point_normals(1,:)];
                    
                obj.controlPoints{i} =  obj.boundaries{i}-dist*point_normals;
                
            end
            obj.totalElements = [0 cumsum(cellfun(@length,obj.centers))];
                
        end

        function geom_mtx = get_geom_mtx(obj)
            %calculate geometry matrix

            geom_mtx = zeros(obj.totalElements(end));

            for i = 1:length(obj.controlPoints)
                for j = 1:length(obj.controlPoints{i})-1
                    pa = obj.controlPoints{i}(j,:);
                    pb = obj.controlPoints{i}(j+1,:);
                    vec = pb - pa;
                    a = vec(1); b = vec(2);
                    a2b2 = a^2 + b^2;
                    len = sqrt(a2b2);
                    for k = 1:length(obj.centers)
                        points((obj.totalElements(k)+1):obj.totalElements(k+1),:)...
                            = obj.centers{k};
                    end
                        %points is 2 vectors of all points p* 
                        %so we calculate H ij(k) for all points
                        % and save them in the matrix geom_mtx

                        %G ij is assumend to be constant
                        x = points(:,1) - pa(1);
                        y = points(:,2) - pa(2);

                        axby = a*x + b*y; 
                        aybx = abs((a*y - b*x));
                        dist_sq = (x-a).^2 + (y-b).^2;

                        wc = log(dist_sq) + ...
                            axby./a2b2 .* log((x.^2+y.^2) ./ dist_sq) + ...
                            2.0 * aybx./a2b2 .* ... 
                            (atan2(axby, aybx) + atan2(a2b2-axby,aybx)) ...
                            -2.0;
                        wc = wc*len;

                        geom_mtx(:,obj.totalElements(i)+j) = wc;

                end
            end
        end

        function univ_mtx = get_univ_mtx(obj)
            %calculate universe matrix
            univ_mtx = zeros(length(obj.centers)-1,  obj.totalElements(end));

            if isempty(univ_mtx)
                return 
            end

            for i = 1:length(obj.controlPoints)
                for j = 1: length(obj.controlPoints{i})-1
                    xa = obj.controlPoints{i}(j,1);
                    ya = obj.controlPoints{i}(j,2);
                    xb = obj.controlPoints{i}(j+1,1);
                    yb = obj.controlPoints{i}(j+1,2);

                    a = xb - xa; b = yb - ya;

                    a2b2 = a^2+ b^2;
                    len = sqrt(a2b2);
                    R = [ xb-xa, -yb+ya; yb-ya, xb-xa];
                    R  = R./len;
                    for k = 2:length(obj.centers)
                        points = obj.centers{k};
                        po = (points - obj.centers{i}(j,:))*R;

                        xop = po(:,1) + (len/2);
                        xom = po(:,1) - (len/2);
                        yo = po(:,2);

                        gradl =  [
                            log((xom.^2+yo.^2)./(yo.^2+xop.^2))/2,...
                            atan(xom./yo)-atan(xop./yo)
                            ];

                        gradw = gradl*R';
                        delta = sum(obj.normals{k}.* gradw, 2);
                        if(i == k)
                                delta(j) = abs(delta(j));
                        end
                        univ_mtx(k-1, obj.totalElements(i)+j) = sum(delta);
                    end
                end
            end                   
        end
        
        function aug_mtx = get_aug_mtx(obj,geom_mtx,univ_mtx)
            %combines geometry and universe matric into augmented matrix
            ne = size(geom_mtx, 1);
            np = size(univ_mtx, 1);
            nt = ne + np;

            if np == 0
                aug_mtx = geom_mtx;
            else
                agm = max(abs(geom_mtx),[],'all'); 
                aum = max(abs(univ_mtx),[],'all');

                aug_mtx = zeros(nt, nt);
                aug_mtx(1:ne, 1:ne) = geom_mtx;
                aug_mtx(ne+1:end, 1:ne) = (agm/aum) * univ_mtx;


                for k =1:length(obj.totalElements)-2       %-1 in k th row
                    len = obj.totalElements(k+2) - obj.totalElements(k+1);
                    aug_mtx(obj.totalElements(k+1)+1:obj.totalElements(k+2), ne+k) =...
                        -ones(len, 1);
                end
            end
        end
        
        function solveMap(obj)
            %Solves the transfomation problem and finds
            %the weights Cx,Cy and the positions on inner obstacles
            obj.A = get_aug_mtx(obj, obj.get_geom_mtx(),obj.get_univ_mtx());
            bx = zeros(size(obj.A,1),1);
            by = zeros(size(obj.A,1),1);
            
            bx(1:obj.totalElements(1+1)) = cos(obj.theta);%actual centres
            by(1:obj.totalElements(1+1))= sin(obj.theta);

            X = linsolve(obj.A,bx);
            Y = linsolve(obj.A,by);

            obj.Cx = X(1:obj.totalElements(end));
            obj.Cy = Y(1:obj.totalElements(end));


            u = X(obj.totalElements(end)+1:end);
            v = Y(obj.totalElements(end)+1:end);
            obj.innerObst_q = [u,v];
        end
    end
    
    
    
    methods
        function obj = HarmonicMap()
            obj.isMapSolved = false;
        end
        
        function setBoundaries(obj,boundaries, isFree)
            % recalculates the map
            % boundaries must be  1 x N cell
            % each containing m x 2 points that define the boundaries
            % BOUNDARY POINTS MUST BE IN A COUNTER CLOCKWISE ORDER.
            % Optinal argument: isFreeCell cell array that is the same 
            % size as boundaries and its elemets are a mx1
            % logical vector that tell you if the boundary is free

            if(nargin==2)
                obj.upscaledBoundaries(boundaries);
            else
                obj.findFrontiers(boundaries,isFree);
            end

            obj.set_states();
            obj.solveMap();

            obj.isMapSolved = true;

            if (~isempty(obj.fig) && isvalid(obj.fig))
                clf(obj.fig)
            end

        end
        
        function q = map(obj,p)
            %the 2x1 vector q is the mapping of point (x,y) 
            %from workspace to disk space
            
            if(~obj.isMapSolved)
                disp('Set boundaries first!')
                return
            end

            xo = p(1); yo = p(2); 
            wc_vec = zeros(obj.totalElements(end),1);

            for i = 1:length(obj.controlPoints)
                xa = obj.controlPoints{i}(1:end-1,1);
                ya = obj.controlPoints{i}(1:end-1,2);
                xb = obj.controlPoints{i}(2:end,1);
                yb = obj.controlPoints{i}(2:end,2);

                a = xb-xa;
                b = yb-ya;
                x = xo-xa;
                y = yo-ya;

                axby = a.*x+b.*y;
                aybx = a.*y-b.*x;
                x2y2 = x.^2+y.^2;
                a2b2 = a.^2+b.^2;

                %length of element
                len  = sqrt(a2b2);
                dist_sq = (x-a).^2 + (y-b).^2;
                wc = log(dist_sq) + ...
                    (axby./a2b2).*log(x2y2./(dist_sq)) + ...
                    2.0 * (aybx./a2b2).* ...
                    (atan(axby./aybx) + atan((a2b2-axby)./aybx))...
                    -2.0;

                    wc = len.*wc;
                    wc_vec(obj.totalElements(i)+1:obj.totalElements(i+1),:) = wc;
            end

            u = obj.Cx' * wc_vec;
            v = obj.Cy' * wc_vec;
            q = [u;v];
        end
        
        function jacob = jacobian(obj,p)
            %returns the jacobian point p = (x,y)

            if(~obj.isMapSolved)
                disp('Set boundaries first!')
                return
            end

            x = p(1); y=p(2);
            gradu = [0,0];
            gradv = [0,0];
            for i=1:length(obj.controlPoints)
                             
                xa = obj.controlPoints{i}(1:end-1,1);
                ya = obj.controlPoints{i}(1:end-1,2);
                xb = obj.controlPoints{i}(2:end,1);
                yb = obj.controlPoints{i}(2:end,2);
                xv = xb-xa;
                yv = yb-ya;

                %Length of element.
                len = sqrt(xv.*xv+yv.*yv);
                xn = xv ./ len;
                yn = yv ./ len;

                %Rotation matrix from local to global coordinate frame.
                R = {xn -yn; yn xn};

                %Center of linear segment.
                xc = 0.5*(xa+xb);
                yc = 0.5*(ya+yb);

                %Point (x,y) w.r.t. to element's local coordinate frame.
                xl = x-xc;
                yl = y-yc;
                xo = R{1,1}.*xl + R{2,1}.*yl;
                yo = R{1,2}.*xl + R{2,2}.*yl;

                %Intermediate values.
                xop = xo+(len/2);
                xom = xo-(len/2);
                xop2 = xop.*xop;
                xom2 = xom.*xom;
                yo2 = yo.*yo;


                ku = obj.Cx(...
                obj.totalElements(i)+1 : obj.totalElements(i+1));

                kv = obj.Cy(...
                obj.totalElements(i)+1 : obj.totalElements(i+1));


                %Unweighted normal gradient expressed in local coordinate frame.
                gradl_x = -0.5*log((yo2 + xom2)./(yo2 + xop2));
                gradl_y = -atan(yo.*(xom-xop)./(yo2 + xom.*xop));
                %Append element's ij components to gradient of u w.r.t. (x, y).
                gradu(1) = gradu(1) +...
                    sum(ku .* ( R{1,1}.*gradl_x + R{1,2}.*gradl_y),1);
                gradu(2) = gradu(2) +... 
                    sum(ku .* ( R{2,1}.*gradl_x + R{2,2}.*gradl_y),1);
                %Append element's ij components to gradient of v w.r.t. (x, y).
                gradv(1) = gradv(1) +...
                    sum(kv .* ( R{1,1}.*gradl_x + R{1,2}.*gradl_y),1);
                gradv(2) = gradv(2) +...
                    sum(kv .* ( R{2,1}.*gradl_x + R{2,2}.*gradl_y),1);


            end
            jacob = [gradu; gradv];
        end
        
        function [q,J] = compute(obj,p)

            %returns both the transformed point q
            %and the jacobian of the point (x,y)

            if(~obj.isMapSolved)
                disp('Set boundaries first!')
                return
            end

            q = obj.map(p);
            J = obj.jacobian(p);
        end

        function v = getFieldVelocity(obj,pos,q_d) 
            [q,J]= obj.compute(pos);
            if(nargin==2)
                %if no dest is given use nearest in trasform
                q_d = obj.getNearestFrontier(q);
            end

                % The robot kinematics
                dx=-inv(J)*(q-q_d);
                %q = obj.map(x)

                %v = norm([q(1)-q_d(1),q(2)-q_d(2)])*dx/(norm(dx)+0.001);
                v = dx/(norm(dx)+0.001);
        end
        
        function nearestFront_q = getNearestFrontier(obj, pos, viz)
            sz = size(pos);
            if (sz(2)~=2) 
                pos = pos';
            end

            frontPoints = obj.boundaries{1}(obj.isFree{1},:);
            frontDist = sqrt(sum((frontPoints-pos).^2,2));
            [~,indx] = min(frontDist);
            nearestFront = frontPoints(indx,:)';
            
            nearest_q  = obj.map(nearestFront)';
            
            temp_dist = sqrt(sum((obj.frontiers_q-nearest_q).^2,2));
            [~,indx] = min(temp_dist);
            nearestFront_q = obj.frontiers_q(indx,:)';

            if(nargin==3 && viz)
                obj.fig;
                subplot(121)
                hold on
                plot(nearestFront(1), nearestFront(2), 'bx','MarkerSize',10)
                hold off
                subplot(122)
                hold on
                plot(nearestFront_q(1), nearestFront_q(2), 'bx','MarkerSize',10)
                hold off
            end
    
        end
        
        function plotMap(obj)

            if(~obj.isMapSolved)
                disp('Set boundaries first!')
                return
            end

            if (isempty(obj.fig) || ~isvalid(obj.fig))
                obj.fig = figure();
            else
                clf(obj.fig)
            end
                
            obj.fig.Name = 'Harmonic Map Transformation';
            figure(obj.fig)

            min_x = min(obj.boundaries{1}(:,1));
            max_x = max(obj.boundaries{1}(:,1));
            dx = max_x-min_x; 
            min_y = min(obj.boundaries{1}(:,2));
            max_y = max(obj.boundaries{1}(:,2));
            dy = max_y-min_y;
            D = 0.015;
            x_l = min_x-D*dx;
            x_u = max_x+D*dx;
            y_l = min_y-D*dy;
            y_u = max_y+D*dy;
            
            
            subplot(121)
            hold on
     
            for i = 1:length(obj.boundaries)
               plot(obj.boundaries{i}(:,1),obj.boundaries{i}(:,2),...
                   'k-','Linewidth',2)

               if(~isempty(obj.isFree))
                   plot(obj.boundaries{i}(obj.isFree{i},1),obj.boundaries{i}(obj.isFree{i},2),...
                       '.g', 'MarkerSize',10)
               end
            end

            axis equal
            axis([x_l x_u y_l y_u])
            box on
            hold off

           
            subplot(122)
            hold on 
            plot(cos(obj.theta),sin(obj.theta),'k-','LineWidth', 2)
            plot(obj.frontiers_q(:,1),obj.frontiers_q(:,2),'ro', 'MarkerSize', 6)
            plot(obj.innerObst_q(:,1),obj.innerObst_q(:,2), ...
                'r.','MarkerSize', 15);                
            axis equal
            axis([-1-D 1+D -1-D 1+D])
            box on
            hold off                
        end
        
        function [t,p_path,q_path] = navigate(obj,p_0,p_d, vis)
            % A safe control scheme to navigate from any point [x_0; y_0]
            %towards a desired point [x_d;y_d] is simply calculated by 
            %[ux,uy]=-inv(J(p))*(f(p)-f(p_d)) for any p the belongs
            %to the interior of the workspace.
            %
            %Outputs:
            %t is time vector,
            %p_path is the nx2 matrix of points in the workspace path that 
            %correspond to t 
            %and q_path is the nx2 matrix of the transformed points 
            %in diskspace.
            %
            %Inputs: 
            %1. obj.navigation() makes you choose start and
            %destination points on the workspace plot (left).
            %2. obj.navigation(x_0,y_0,x_d,y_d) finds the paths without any
            %visual plots.
            %3. obj.navigate(obj,x_0,y_0,x_d,y_d, vis) does the same as 2
            %but also visualizes in the map plot if vis is true.
            
            if(~obj.isMapSolved)
                disp('Set boundaries first!')
                return
            end

            if (nargin == 1)
                obj.plotMap()

                %Starting point
                p_0 = ginput(1);
                obj.fig;
                subplot(121)
                plot(p_0(1),p_0(2),'ko','MarkerSize', 7)
                q_0 = obj.map(p_0);
                % q_d is the image of p_d=[x_d;y_d] and J_d is the Jacobian of the mapping
                subplot(122)
                plot(q_0(1),q_0(2),'ko','MarkerSize', 7)


                %Destination point
                p_d = ginput(1);
                obj.fig;
                subplot(121)
                plot(p_d(1),p_d(2),'kx','MarkerSize', 10)
                q_d = obj.map(p_d);
                % q_d is the image of p_d=[x_d;y_d]
                subplot(122)
                plot(q_d(1),q_d(2),'kx','MarkerSize', 10)

                options = odeset('abstol',1e-6,'reltol',1e-6,'events',@my_event);
                [t,p] = ode15s(@system_kin,[0,obj.maxTime],[p_0(1);p_0(2)],options);
                disp(['Elapsed Time: ',num2str(t(end)),' sec']);
                
                q_points = zeros(size(p));
                for i=1:length(p)
                    qtemp = obj.map(p(i,:));
                    q_points(i,:) = qtemp';
                end

                obj.fig;
                subplot(121)
                plot(p(:,1),p(:,2),'k','linewidth',1)
                subplot(122)
                plot(q_points(:,1),q_points(:,2),'r','linewidth',1)
            else
                q_0 = obj.map(p_0);
                q_d = obj.map(p_d);
                options=odeset('abstol',1e-6,'reltol',1e-6,'events',@my_event);
                [t,p] = ode15s(@system_kin,[0,obj.maxTime],p_0,options);
                disp(['Elapsed Time: ',num2str(t(end)),' seconds']);
                q_points = zeros(size(p));
                for i=1:length(p)
                    qtemp = obj.map(p(i,:));
                    q_points(i,:) = qtemp;
                end
                
                if (nargin==4 && vis)
                    obj.plotMap()
                    obj.fig;
                    %plot p0 & q0
                    subplot(121)
                    plot(p_0(1),p_0(2),'ko','MarkerSize', 7)
                    subplot(122)
                    plot(q_0(1),q_0(2),'ko','MarkerSize', 7)
                    
                    %plot pd & qd
                    subplot(121)
                    plot(p_d(1),p_d(2),'kx','MarkerSize', 10)
                    subplot(122)
                    plot(q_d(1),q_d(2),'kx','MarkerSize', 10)
                    
                    %plot p path and q path
                    subplot(121)
                    plot(p(:,1),p(:,2),'k','linewidth',1)
                    subplot(122)
                    plot(q_points(:,1),q_points(:,2),'r','linewidth',1)
                end
            end
            p_path = p;
            q_path = q_points;
            function dx=system_kin(~,x) 
                % The robot kinematics
                [q,J]= obj.compute(x);
                dx=-inv(J)*(q-q_d);
                %dx = norm([x(1)-p_d(1),x(2)-p_d(2)])*dx/(norm(dx)+0.001);
                dx = dx/(norm(dx)+0.001);
            end
            
            function [value,isterminal,direction] = my_event(~,x) 
                % Event function to detect when we have arrived closed to the desired point 
                %and terminate earlier the simulation
                
                value = norm([x(1)-p_d(1);x(2)-p_d(2)])-1e-2;
                isterminal = 1;
                direction = -1;
            end
        end

        function [t,p_path,q_path] = explore(obj,p_0, vis)
            % TODO change

            if(~obj.isMapSolved)
                disp('Set boundaries first!')
                return
            end

            
            if isempty(obj.frontiers_q)
                disp('Map fully explored!')
                return
            end
            %maybe use other metric
            q_d = obj.frontiers_q(1,:)';
            q_0 = obj.map(p_0);
            
            options=odeset('abstol',1e-6,'reltol',1e-6,'events',@my_event);
            [t,p] = ode15s(@system_kin,[0,obj.maxTime],p_0,options);
            disp(['Elapsed Time: ',num2str(t(end)),' seconds']);
            q_points = zeros(size(p));
            for i=1:length(p)
                qtemp = obj.map(p(i,:));
                q_points(i,:) = qtemp;
            end
            
            if (nargin==3 && vis)
                obj.plotMap()
                obj.fig;
                %plot p0 & q0
                subplot(121)
                plot(p_0(1),p_0(2),'ko','MarkerSize', 7)
                subplot(122)
                plot(q_0(1),q_0(2),'ko','MarkerSize', 7)
                
                %plot pd & qd
                subplot(121)
                p_d = p(end,:);
                plot(p_d(1),p_d(2),'kx','MarkerSize', 10)
                subplot(122)
                plot(q_d(1),q_d(2),'kx','MarkerSize', 10)
                
                %plot p path and q path
                subplot(121)
                plot(p(:,1),p(:,2),'k','linewidth',1)
                subplot(122)
                plot(q_points(:,1),q_points(:,2),'r','linewidth',1)
            end

        p_path = p;
        q_path = q_points;


        function dx=system_kin(~,x) 
            % The robot kinematics
            [q,J]= obj.compute(x);
            dx=-inv(J)*(q-q_d);
            %normize 
            dx =dx/(norm(dx)+0.001);             
        end
        
        function [value,isterminal,direction] = my_event(~,x) 
            % Event function to detect when we have arrived closed to the desired point 
            %and terminate earlier the simulation
            q = obj.map(x);
            value = norm([q(1)-q_d(1);q(2)-q_d(2)])-5e-2;
            isterminal = 1;
            direction = -1;
        end
        end
    end
end