function [tform, tfply] = myicp(curr_ply, base_ply, ...
    tform_init, max_iter, tol, method)
% myicp: Iterative Closest Point (ICP) algorithm
% input:
%   curr_ply: current point cloud to be transformed, size: n*3
%   base_ply: base point cloud, which is the target, size: n*3
%   tform_init: initial transformation matrix, size: 4*4
%   max_iter: maximum number of iterations
%   tol: tolerance
%   method: method of finding nearest neighbor,
%       'bf':brute force,'kd': kd-tree, 'kn': knnsearch
% output:
%   tform: transformation matrix, size: 4*4
%   tfply: transformed point cloud, size: n*3
    % init
    step = 0;
    last_err = 100;
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
        W = zeros(3,3);
        base_points_temp = zeros(180,3);
        
        switch method
        case 'bf'
            % find nearest neighbor using brute force
            for j=1:length(curr_points)
                min_dist = 100;
                for k=1:length(base_points)
                    dist = norm(curr_points(j,:) - ...
                                     base_points(k,:));
                    if dist < min_dist
                        min_dist = dist;
                        index = k;
                    end
                end
                base_points_temp(j,:) = base_points(index,:);
            end
        case 'kd'
            % find nearest neighbor using kd-tree
            kdtree = KDTreeSearcher(base_points);
            [index,~]=knnsearch(kdtree,curr_points,"K",1);
            for j=1:length(index)
                base_points_temp(j,:)=base_points(index(j),:);
            end
        case 'kn'
            % find nearest neighbor using knnsearch
            [index,~]=knnsearch(base_points,curr_points,"K",1);
            for j=1:length(index)
                base_points_temp(j,:)=base_points(index(j),:);
            end
        end

        % calculate center of mass
        base_p_center = mean(base_points_temp);  % mass center of base
        curr_p_center = mean(curr_points);  % mass center of current
        % get relative position
        new_base_p = base_points_temp - base_p_center;
        new_curr_p = curr_points - curr_p_center;
       
        % calculate W
        W = W + new_curr_p' * new_base_p;  
        
        % calculate R and t using SVD
        [U,~,V] = svd(W);
        R = V'*U';
        t =  base_p_center' - R * curr_p_center'; 
    
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