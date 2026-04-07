function [tform, tfply, debug_info] = myplicp(curr_ply, base_ply, tform_init, max_iter, mse_threshold, dist_threshold, cos_threshold)
% myplicp: 参考 SYK_ICP/pl_icp_kd.m 的简洁版 PL-ICP
%
% 当前版本专门针对“点到线 MSE 下降了，但地图反而更散”的问题做修正：
% 1. 法向估计改回 SYK 的大邻域方式，避免法向过于抖动。
% 2. 点线匹配改回更贴近 SYK 的构造方式。
% 3. 不再用过早的“前后两次 MSE 变化很小就停”，避免停在坏局部极小。

    if nargin < 7
        cos_threshold = 0.1;
    end
    if nargin < 6
        dist_threshold = 0.6;
    end

    if isa(tform_init, 'affine3d')
        tform = double(tform_init.T);
    else
        tform = double(tform_init);
    end

    p = double(base_ply.Location);
    p_ = double(curr_ply.Location);

    R0 = tform(1:3, 1:3);
    t0 = tform(1:3, 4);
    p_ = (R0 * p_.' + t0).';

    mse_history = zeros(max_iter, 1);
    valid_count_history = zeros(max_iter, 1);

    [p_match, pp_] = point_line_pairs(p, p_, dist_threshold, cos_threshold);
    if size(p_match, 1) < 3
        tfply = pointCloud(p_);
        debug_info.iteration_count = 0;
        debug_info.valid_correspondence_count = 0;
        debug_info.mean_residual = inf;
        debug_info.mse_history = [];
        return;
    end

    for k = 1:max_iter
        [R, t] = solve_svd(p_match, pp_);
        step_tform = [R, t; 0, 0, 0, 1];

        tform = step_tform * tform;
        p_ = (R * p_.' + t).';

        [p_match, pp_] = point_line_pairs(p, p_, dist_threshold, cos_threshold);
        if size(p_match, 1) < 3
            mse_history = mse_history(1:max(k - 1, 1));
            valid_count_history = valid_count_history(1:max(k - 1, 1));
            break;
        end

        mse_history(k) = mean(sum((p_match - pp_).^2, 2));
        valid_count_history(k) = size(p_match, 1);

        if mse_history(k) < mse_threshold
            break;
        end
    end

    if numel(mse_history) == max_iter
        last_valid = find(valid_count_history > 0, 1, 'last');
        if isempty(last_valid)
            last_valid = 1;
        end
        mse_history = mse_history(1:last_valid);
        valid_count_history = valid_count_history(1:last_valid);
    end

    tfply = pointCloud(p_);
    debug_info.iteration_count = numel(mse_history);
    debug_info.valid_correspondence_count = valid_count_history(end);
    debug_info.mean_residual = sqrt(mse_history(end));
    debug_info.mse_history = mse_history;
end

function [nearest_points, min_index] = match_kd(p, p_, k)
    kd_tree = KDTreeSearcher(p, 'BucketSize', 10);
    [min_index, ~] = knnsearch(kd_tree, p_, 'K', k);
    temp = p(min_index, :);
    nearest_points = reshape(temp, [size(temp, 1) / k, k, 3]);
end

function [R, t] = solve_svd(p, p_)
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

function normal_vector = estimate_normals(points, radius)
% 这里保留 SYK 的大邻域法向估计风格：
% 先找 80 个近邻，再用局部协方差的最小特征向量作为法向。

    near_points = match_kd(points, points, 80);
    normal_vector = zeros(size(points));

    for i = 1:size(points, 1)
        local_points = reshape(near_points(i, :, :), [size(near_points, 2), 3]);
        distance = sum((local_points - points(i, :)).^2, 2);
        valid_num = sum(distance < radius^2);
        if valid_num < 3
            valid_num = size(local_points, 1);
        end

        [~, order] = sort(distance);
        local_points = local_points(order, :);
        local_points = local_points(1:valid_num, :);

        center = mean(local_points, 1);
        err = local_points - center;
        covar = (err' * err) / valid_num;
        [V, D] = eig(covar);
        [~, idx] = min(diag(D));
        normal_vector(i, :) = V(:, idx)';
    end
end

function [p_inter, pp_] = point_line_pairs(p, p_, dist_threshold, cos_threshold)
% 这里直接按 SYK 的思路构造点到线对应：
% 对每个当前点找地图中最近的两个点，用它们构成局部直线，
% 再用距离和法向夹角双重约束筛选。

    p_inter = [];
    pp_ = [];

    nvector_p = estimate_normals(p, 0.25);
    nvector_p_ = estimate_normals(p_, 0.25);

    [Npoints, min_index] = match_kd(p, p_, 2);
    nvector_p = nvector_p(min_index, :);
    nvector_p = reshape(nvector_p, [size(p_, 1), 2, 3]);

    idx = 1;
    for i = 1:size(Npoints, 1)
        line_vec = reshape(Npoints(i, 1, :) - Npoints(i, 2, :), [1, 3]);
        h = p_(i, :) - reshape(Npoints(i, 2, :), [1, 3]);
        line_len = norm(line_vec);

        if line_len < 1e-6
            continue;
        end

        dist = norm(cross(line_vec, h)) / line_len;
        cos_1 = abs(reshape(nvector_p(i, 1, :), [1, 3]) * nvector_p_(i, :)');
        cos_2 = abs(reshape(nvector_p(i, 2, :), [1, 3]) * nvector_p_(i, :)');

        if dist < dist_threshold && cos_1 > cos_threshold && cos_2 > cos_threshold
            rate = (line_vec * h') / (line_len^2);
            p_inter(idx, :) = reshape(rate * Npoints(i, 2, :) + (1 - rate) * Npoints(i, 1, :), [1, 3]); %#ok<AGROW>
            pp_(idx, :) = p_(i, :); %#ok<AGROW>
            idx = idx + 1;
        end
    end
end
