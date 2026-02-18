function is_collision = is_collision_with_unknown(q_check, UnknownPoints, R_obs_check)
    is_collision = false;
    
    if ~isempty(UnknownPoints)
        % q_check と UnknownPoints のすべての点との距離を計算
        distances = sqrt(sum((UnknownPoints - q_check).^2, 2));
        
        % 最短距離が安全マージン R_obs_check より小さいかチェック
        if min(distances) < R_obs_check 
            is_collision = true; % 衝突
            return;
        end
    end
end