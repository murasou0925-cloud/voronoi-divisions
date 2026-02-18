function is_hit = is_path_collision_unknown(q_start, q_end, UnknownPoints, R_obs_check)
    is_hit = false;
    
    % 1. チェックする間隔を決める（安全半径の半分くらいが目安）
    step_size = R_obs_check / 2; 
    
    % 2. スタートからゴールまでの距離とベクトル
    vec = q_end - q_start;
    dist = norm(vec);
    
    % 3. 何回チェックするか計算
    num_checks = ceil(dist / step_size);
    
    % 4. 線分上を少しずつ進みながらチェック
    for k = 0:num_checks
        % 検査する座標を作る（線形補間）
        % k=0ならスタート地点、k=num_checksならゴール地点
        curr_pt = q_start + (vec * (k / num_checks));
        
        % ★ここで、あなたがアップロードした「点の判定関数」を再利用！
        if is_collision_with_unknown(curr_pt, UnknownPoints, R_obs_check)
            is_hit = true; % 途中でぶつかった！
            return;        % 即終了
        end
    end
end