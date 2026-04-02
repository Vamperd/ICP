function [tform, tfply, debug_info] = myplicp(curr_ply, base_ply, tform_init, max_iter, mse_threshold, dist_threshold, cos_threshold)
% myplicp: 简洁版 PL-ICP
% 参考 syk IPC/pl_icp_kd.m：
% 1. 地图中为每个点估计法向
% 2. 对当前点找地图中最近的两个点构成局部线段
% 3. 若点到线距离和法向夹角都满足阈值，则取投影点参与 SVD

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

    for k = 1:max_iter
        p_normal = estimate_normals(p, 20, 0.25);
        p__normal = estimate_normals(p_, 20, 0.25);
        [p_proj, p_src] = point_line_match(p, p_, p_normal, p__normal, dist_threshold, cos_threshold);

        if size(p_proj, 1) < 3
            break;
        end

        [R, t] = solve_svd(p_proj, p_src);
        step_tform = [R, t; 0, 0, 0, 1];

        tform = step_tform * tform;
        p_ = (R * p_.' + t).';

        mse_history(k) = mean(sum((p_proj - p_src).^2, 2));
        valid_count_history(k) = size(p_proj, 1);

        if mse_history(k) < mse_threshold
            break;
        end
        if k > 1 && abs(mse_history(k) - mse_history(k - 1)) < 1e-6
            break;
        end
    end

    mse_history = mse_history(1:max(k, 1));
    valid_count_history = valid_count_history(1:max(k, 1));
    tfply = pointCloud(p_);

    debug_info.iteration_count = numel(mse_history);
    debug_info.valid_correspondence_count = valid_count_history(end);
    debug_info.mean_residual = sqrt(mse_history(end));
    debug_info.mse_history = mse_history;
end

function [p_proj, p_src] = point_line_match(p, p_, p_normal, p__normal, dist_threshold, cos_threshold)
    p_proj = [];
    p_src = [];

    [index2, ~] = knnsearch(p, p_, 'K', 2);
    for i = 1:size(p_, 1)
        q1 = p(index2(i, 1), :);
        q2 = p(index2(i, 2), :);
        n1 = p_normal(index2(i, 1), :);
        n2 = p_normal(index2(i, 2), :);
        ns = p__normal(i, :);

        line_vec = q2 - q1;
        line_len = norm(line_vec);
        if line_len < 1e-6 || norm(ns) < 1e-6
            continue;
        end

        h = p_(i, :) - q1;
        dist = norm(cross(line_vec, h)) / line_len;
        cos1 = abs(dot(n1, ns)) / (norm(n1) * norm(ns) + eps);
        cos2 = abs(dot(n2, ns)) / (norm(n2) * norm(ns) + eps);

        if dist < dist_threshold && cos1 > cos_threshold && cos2 > cos_threshold
            alpha = dot(h, line_vec) / (line_len^2);
            alpha = min(max(alpha, 0), 1);
            q_proj = q1 + alpha * line_vec;

            p_proj(end + 1, :) = q_proj; %#ok<AGROW>
            p_src(end + 1, :) = p_(i, :); %#ok<AGROW>
        end
    end
end

function normal_vector = estimate_normals(points, k, radius)
    [index, distance] = knnsearch(points, points, 'K', k);
    normal_vector = zeros(size(points));

    for i = 1:size(points, 1)
        keep = distance(i, :) < radius;
        if nnz(keep) < 3
            keep = true(1, k);
        end

        near_points = points(index(i, keep), :);
        center = mean(near_points, 1);
        err = near_points - center;
        covar = (err' * err) / size(near_points, 1);
        [V, D] = eig(covar);
        [~, idx] = min(diag(D));
        normal_vector(i, :) = V(:, idx)';
    end
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
