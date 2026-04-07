function [tform, tfply, debug_info] = mynicp(curr_ply, base_ply, tform_init, ...
    max_iter, mse_threshold, dist_threshold, cos_threshold, tangent_threshold, candidate_k)
% mynicp: 法向引导的点到线 ICP
%
% 实现思路：
% 1. 对地图点和当前帧点都估计法向；
% 2. 每个当前点在地图里找多个候选锚点；
% 3. 每个锚点再和它在地图中的局部邻点构成一条局部线段；
% 4. 同时检查点到线距离、法向一致性、切向位移，再选出最优对应；
% 5. 最后对“投影点 - 源点”做 SVD。
%
% 这样做比“最近邻点到点 + 法向过滤”更稳，也比“单点切平面投影”更不容易退化。

    if nargin < 9
        candidate_k = 4;
    end
    if nargin < 8
        tangent_threshold = 1.0;
    end
    if nargin < 7
        cos_threshold = 0.15;
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
        p_normal = estimate_normals(p, 0.25);
        p__normal = estimate_normals(p_, 0.25);
        [p_match, p_src] = normal_guided_line_pairs( ...
            p, p_, p_normal, p__normal, dist_threshold, cos_threshold, tangent_threshold, candidate_k);

        if size(p_match, 1) < 3
            if k == 1
                mse_history = [];
                valid_count_history = [];
            else
                mse_history = mse_history(1:k - 1);
                valid_count_history = valid_count_history(1:k - 1);
            end
            break;
        end

        [R, t] = solve_svd(p_match, p_src);
        step_tform = [R, t; 0, 0, 0, 1];

        tform = step_tform * tform;
        p_ = (R * p_.' + t).';

        mse_history(k) = mean(sum((p_match - p_src).^2, 2));
        valid_count_history(k) = size(p_match, 1);

        if mse_history(k) < mse_threshold
            mse_history = mse_history(1:k);
            valid_count_history = valid_count_history(1:k);
            break;
        end

        if k == max_iter
            last_valid = find(valid_count_history > 0, 1, 'last');
            mse_history = mse_history(1:last_valid);
            valid_count_history = valid_count_history(1:last_valid);
        end
    end

    tfply = pointCloud(p_);
    if isempty(mse_history)
        debug_info.iteration_count = 0;
        debug_info.valid_correspondence_count = 0;
        debug_info.mean_residual = inf;
        debug_info.mse_history = [];
    else
        debug_info.iteration_count = numel(mse_history);
        debug_info.valid_correspondence_count = valid_count_history(end);
        debug_info.mean_residual = sqrt(mse_history(end));
        debug_info.mse_history = mse_history;
    end
end

function [p_match, p_src] = normal_guided_line_pairs(p, p_, p_normal, p__normal, ...
    dist_threshold, cos_threshold, tangent_threshold, candidate_k)

    p_match = [];
    p_src = [];

    % 给每个源点找多个候选锚点。
    [candidate_points, candidate_index] = match_kd(p, p_, candidate_k);

    % 给地图中的每个点准备一个局部线段邻点。
    [local_line_neighbor, ~] = knnsearch(p, p, 'K', 2);
    line_neighbor_index = local_line_neighbor(:, 2);

    for i = 1:size(p_, 1)
        ns = p__normal(i, :);
        if norm(ns) < 1e-6
            continue;
        end

        best_score = inf;
        best_projection = [];

        for j = 1:candidate_k
            idx1 = candidate_index(i, j);
            idx2 = line_neighbor_index(idx1);

            q1 = reshape(candidate_points(i, j, :), [1, 3]);
            q2 = p(idx2, :);
            n1 = p_normal(idx1, :);
            n2 = p_normal(idx2, :);

            line_vec = q2 - q1;
            line_len = norm(line_vec);
            if line_len < 1e-6 || norm(n1) < 1e-6 || norm(n2) < 1e-6
                continue;
            end

            h = p_(i, :) - q1;
            line_dist = norm(cross(line_vec, h)) / line_len;
            if line_dist > dist_threshold
                continue;
            end

            cos1 = abs(dot(n1, ns)) / (norm(n1) * norm(ns) + eps);
            cos2 = abs(dot(n2, ns)) / (norm(n2) * norm(ns) + eps);
            if cos1 < cos_threshold || cos2 < cos_threshold
                continue;
            end

            rate = (line_vec * h') / (line_len^2);
            rate = min(max(rate, 0), 1);
            q_proj = q1 + rate * line_vec;

            tangent_shift = norm(q_proj - q1);
            if tangent_shift > tangent_threshold
                continue;
            end

            score = line_dist + 0.20 * tangent_shift + 0.05 * (2 - cos1 - cos2);
            if score < best_score
                best_score = score;
                best_projection = q_proj;
            end
        end

        if ~isempty(best_projection)
            p_match(end + 1, :) = best_projection; %#ok<AGROW>
            p_src(end + 1, :) = p_(i, :); %#ok<AGROW>
        end
    end
end

function [nearest_points, min_index] = match_kd(p, p_, k)
    kd_tree = KDTreeSearcher(p, 'BucketSize', 10);
    [min_index, ~] = knnsearch(kd_tree, p_, 'K', k);
    temp = p(min_index, :);
    nearest_points = reshape(temp, [size(temp, 1) / k, k, 3]);
end

function normal_vector = estimate_normals(points, radius)
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
