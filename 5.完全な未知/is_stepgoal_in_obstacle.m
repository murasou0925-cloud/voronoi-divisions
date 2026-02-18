function is_blocked = is_stepgoal_in_obstacle(P_step, KnownObs, UnknownPoints)
% P_step: 現在の一時目標点 (secpoint)
% KnownObs: 既知障害物リスト (N_known x 6)
% UnknownPoints: 未知障害物点群 (N_unknown x 3)

    % --- 既知障害物との判定 ---
    for j = 1:size(KnownObs, 1)
        if is_in_obstacle(P_step, KnownObs(j, :))
            is_blocked = true;
            return;
        end
    end
    
    % --- 未知障害物との判定 ---
    % 未知の点群を代表する領域がないため、近傍距離 (R_obs_check) を設けて判定
    R_obs_check = 0.3; % 0.3m以内に未知の点があればブロックと見なす (調整可能)
    
    if ~isempty(UnknownPoints)
        % StepGoalと全てのUnknownPointsの距離を計算
        distances = sqrt(sum((UnknownPoints - P_step).^2, 2));
        
        if min(distances) < R_obs_check
            is_blocked = true;
            return;
        end
    end

    is_blocked = false;
end