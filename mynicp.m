function [tform, tfply, debug_info] = mynicp(curr_ply, base_ply, tform_init, max_iter, mse_threshold, dist_threshold, cos_threshold)
% mynicp: 简洁版法向约束 ICP
% 思路比 PL-ICP 更直接：
% 1. 给地图和当前帧都估计法向
% 2. 每个当前点找一个最近地图点
% 3. 若法向相近，则保留这对点参与刚体求解
% 4. 用“法向筛选后的点到点对应”做 SVD

    if nargin < 7
        cos_threshold = 0.5;
    end
    if nargin < 6
        dist_threshold = 0.5;
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
        [p_match, p_src] = normal_match(p, p_, p_normal, p__normal, dist_threshold, cos_threshold);

        if size(p_match, 1) < 3
            break;
        end

        [R, t] = solve_svd(p_match, p_src);
        step_tform = [R, t; 0, 0, 0, 1];

        tform = step_tform * tform;
        p_ = (R * p_.' + t).';

        mse_history(k) = mean(sum((p_match - p_src).^2, 2));
        valid_count_history(k) = size(p_match, 1);

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

function [p_match, p_src] = normal_match(p, p_, p_normal, p__normal, dist_threshold, cos_threshold)
    p_match = [];
    p_src = [];

    index = knnsearch(p, p_, 'K', 1);
    for i = 1:size(p_, 1)
        q = p(index(i), :);
        n = p_normal(index(i), :);
        ns = p__normal(i, :);

        if norm(n) < 1e-6 || norm(ns) < 1e-6
            continue;
        end

        dist = norm(p_(i, :) - q);
        cos_value = abs(dot(n, ns)) / (norm(n) * norm(ns) + eps);
        if dist > dist_threshold || cos_value < cos_threshold
            continue;
        end

        p_match(end + 1, :) = q; %#ok<AGROW>
        p_src(end + 1, :) = p_(i, :); %#ok<AGROW>
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
