function is_safe = is_path_safe_RRT(q_from, q_to, KnownObstacles, UnknownPoints, R_obs_check)
    
    num_steps = 100; % チェックの分解能 (高いほど正確だが遅い)
    is_safe = true;

    for k = 0:num_steps
        q_check = q_from + (q_to - q_from) * (k / num_steps);
        
        % --- A. 既知の障害物との衝突判定 (直方体チェック) ---
        for j = 1:size(KnownObstacles, 1)
            O_j = KnownObstacles(j, :);
            % is_in_obstacle 関数（工程1-1で作成したもの）を使用
            if is_in_obstacle(q_check, O_j) 
                is_safe = false;
                return;
            end
        end
        
        % --- B. 未知の障害物との衝突判定 (点群チェック) ---
        if is_collision_with_unknown(q_check, UnknownPoints, R_obs_check)
            is_safe = false;
            return;
        end
    end
end