function [tform, tfply, debug_info] = myicp(curr_ply, base_ply, tform_init, max_iter, mse_threshold, max_corr_dist)
% myicp: 简洁版原始 ICP
% 参考 syk IPC/raw_icp.m 的实现思路：
% 1. 先用上一帧累计位姿做粗配准
% 2. 最近邻建立对应
% 3. SVD 求刚体变换
% 4. 直到 MSE 足够小或迭代收敛

    if nargin < 6
        max_corr_dist = inf;
    end

    if isa(tform_init, 'affine3d')
        tform = double(tform_init.T);
    else
        tform = double(tform_init);
    end

    p = double(base_ply.Location);
    p_ = double(curr_ply.Location);

    % 先用已有初值把当前帧变到比较接近地图的位置。
    R0 = tform(1:3, 1:3);
    t0 = tform(1:3, 4);
    p_ = (R0 * p_.' + t0).';

    mse_history = zeros(max_iter, 1);
    valid_count_history = zeros(max_iter, 1);

    for k = 1:max_iter
        [p_match, valid_mask] = match_points(p, p_, max_corr_dist);
        if nnz(valid_mask) < 3
            valid_mask = true(size(valid_mask));
        end

        [R, t] = solve_svd(p_match(valid_mask, :), p_(valid_mask, :));
        step_tform = [R, t; 0, 0, 0, 1];

        tform = step_tform * tform;
        p_ = (R * p_.' + t).';

        [p_match, valid_mask] = match_points(p, p_, max_corr_dist);
        if nnz(valid_mask) < 3
            valid_mask = true(size(valid_mask));
        end

        distance = sqrt(sum((p_match(valid_mask, :) - p_(valid_mask, :)).^2, 2));
        mse_history(k) = mean(distance.^2);
        valid_count_history(k) = nnz(valid_mask);

        if mse_history(k) < mse_threshold
            break;
        end
        if k > 1 && abs(mse_history(k) - mse_history(k - 1)) < 1e-6
            break;
        end
    end

    mse_history = mse_history(1:k);
    valid_count_history = valid_count_history(1:k);
    tfply = pointCloud(p_);

    debug_info.iteration_count = k;
    debug_info.valid_correspondence_count = valid_count_history(end);
    debug_info.mean_residual = sqrt(mse_history(end));
    debug_info.mse_history = mse_history;
end

function [p_match, valid_mask] = match_points(p, p_, max_corr_dist)
    index = knnsearch(p, p_, 'K', 1);
    p_match = p(index, :);
    dist = sqrt(sum((p_match - p_).^2, 2));
    valid_mask = dist < max_corr_dist;
end

function [R, t] = solve_svd(p, p_)
% 输入 p 为目标点，p_ 为源点，输出把 p_ 变到 p 的刚体变换。

    p_center = mean(p_, 1);
    q_center = mean(p, 1);
    q_ = p_ - p_center;
    q = p - q_center;

    H = zeros(3, 3);
    for i = 1:size(p_, 1)
        H = H + q_(i, :)' * q(i, :);
    end

    [U, ~, V] = svd(H);
    R = V * U';
    if det(R) < 0
        V(:, 3) = -V(:, 3);
        R = V * U';
    end
    t = q_center' - R * p_center';
end
