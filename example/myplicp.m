function [tform, tfply] = myplicp(curr_ply, base_ply, ...
    tform_init, max_iter, tol)
% myplicp: point to line ICP algorithm
% input:
%   curr_ply: current point cloud to be transformed, size: n*3
%   base_ply: base point cloud, which is the target, size: n*3
%   tform_init: initial transformation matrix, size: 4*4
%   max_iter: maximum number of iterations
%   tol: tolerance
% output:
%   tform: transformation matrix, size: 4*4
%   tfply: transformed point cloud, size: n*3
    % init
    step = 0;
    last_err = inf;
    err = 1;
    tform_iter = eye(4, 4); % iteratively update tform

    base_points  =  base_ply.Location;  %info of base point cloud
    curr_points  =  curr_ply.Location;  %info of current point cloud

    % transform curr_points to base_points
    curr_points = (tform_init(1:3,1:3)* curr_points' + ...
                                    tform_init(1:3,4))';
    %iteration
    while step<max_iter && err>tol 
        % init
        base_points_temp = zeros(180,3);
        
        % find nearest neighbor using knnsearch
        [index,~]=knnsearch(base_points,curr_points,"K",1);
        for j=1:length(index)
            base_points_temp(j,:)=base_points(index(j),:);
        end

        % calculate center of mass
        base_p_center = mean(base_points_temp);  % mass center of base
        curr_p_center = mean(curr_points);  % mass center of current
        % get relative position
        new_base_p = base_points_temp - base_p_center;
        new_curr_p = curr_points - curr_p_center;
       
        % calculate unit normal vector
        base_norm_vector = cal_2Dpc_norm_vector(new_curr_p,new_base_p,'0');  
        
        % calculate R and t using pointToLineMetric
        [R, t] = pointToLineMetric(new_curr_p, new_base_p, base_norm_vector);

        % calculate tform between two point clouds
        single_tform = [ R      t;
                         0 0 0  1];
        tform_iter = single_tform * tform_iter;
        
        % transform curr_points using tform
        curr_points = (single_tform(1:3,1:3)* curr_points' + single_tform(1:3,4))';
    
        % calculate error
        curr_err = norm(curr_points - base_points_temp);
        err = abs(last_err - curr_err);
        last_err = curr_err;
    
        % update step
        step = step+1;
    end
    % return [tform, tfply]
    tform = tform_init * tform_iter;
    tfply = pointCloud(curr_points);
end


%--------------------------------------------------------------------------
% Solve the following minimization problem:
%       min_{R, T} sum(|dot(R*p+T-q,nv)|^2)
%
% p, q, nv are all N-by-3 matrix, and nv is the unit normal at q
%
% Inspired by point2plane algorithm, here the problem is solved by
% linear approximation to the rotation matrix when the angle is small.
%--------------------------------------------------------------------------
function [R, T] = pointToLineMetric(p, q, nv)
    % Set up the linear system
    A = [p(:,2).*nv(:,3)-p(:,3).*nv(:,2),...
         p(:,3).*nv(:,1)-p(:,1).*nv(:,3),...
         p(:,1).*nv(:,2)-p(:,2).*nv(:,1),...
            nv(:,1),nv(:,2),nv(:,3)];
    
    b = sum((q-p).*nv,2);
  
    % X is [alpha, beta, gamma, Tx, Ty, Tz]
    % solve the linear equation
    X = A\b;
    % X = pinv(A)*b;

    % R = eul2rotm(X(1:3)');
    R=[1,-X(3),X(2)
       X(3),1,-X(1)
      -X(2),X(1),1];
    
    T = X(4:6);
end

function unit_norm_vector = cal_2Dpc_norm_vector(pc_new,pc,mode)
% calculate the line normal vector of point cloud
%input:
%   pc: point cloud, size: n*3
%output:
%   norm_vector: unit line normal vector of point cloud, size: n*3
    n = length(pc);
    norm_vector = zeros(n,3);

    if(nargin<2)
        mode = 'default';
    end
    switch mode
    case '2'
        % Use 2 neighboring points to calculate a normal vector. 
        % find nearest neighbor using knnsearch
        [index,~]=knnsearch(pc,pc_new,"K",2);
        for i=1:n
            % calculate the vector of each point
            norm_vector(i,:) = (pc(index(i,2),:)-pc(index(i,1),:));
        end
        % change the direction of the normal vector, [-pi,pi]->[0,pi]
        phi = atan(norm_vector(:,2)./norm_vector(:,1)) + pi/2;
        % delete the points with large change of angle
        phi_diff = abs([phi(2:end);phi(1)]-phi);
        norm_vector = [cos(phi),sin(phi),zeros(n,1)];
        norm_vector(phi_diff>pi/2 * 0.8,:) = zeros(size(norm_vector(phi_diff>pi/2 * 0.8,:)));
    otherwise
        % Use 6 neighboring points to estimate a normal vector. 
        % find nearest neighbor using knnsearch
        [index,~]=knnsearch(pc,pc,"K",7);
        for i=1:n
            % calculate the vector of each point
            norm_vector(i,:) = (pc(index(i,2),:)-pc(index(i,1),:)) + ...
                            (pc(index(i,3),:)-pc(index(i,1),:)) + ...
                            (pc(index(i,4),:)-pc(index(i,1),:)) + ...
                            (pc(index(i,5),:)-pc(index(i,1),:)) + ...
                            (pc(index(i,6),:)-pc(index(i,1),:)) + ...
                            (pc(index(i,7),:)-pc(index(i,1),:));
        end
        % change the direction of normal vector
        norm_vector = [norm_vector(:,2),-norm_vector(:,1),norm_vector(:,3)];
    end
    
    % % delete the inflection point
    % norm_err = norm_vector-[norm_vector(2:end,:);norm_vector(1,:)];
    % norm_err = sqrt(sum(norm_err.^2,2));

    % % % Assume that the change follows the Gaussian distribution
    % % norm_gauss = normpdf(norm_err,mean(norm_err),std(norm_err))
    % % % delete the point with a large change
    % % norm_vector(norm_gauss<0.15,:)=0;

    % % delete the point with a large change
    % norm_vector(norm_err>10,:)=0;

    % normalize the normal vector
    unit_norm_vector = norm_vector./sqrt(sum(norm_vector.^2,2));
    % delete the nan value
    unit_norm_vector(isnan(unit_norm_vector))=0;
end