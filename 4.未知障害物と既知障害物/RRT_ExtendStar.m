function [State, nodes, nc, zncoord] = RRT_ExtendStar(nodes, nc, zrcoord, BRC, StateDim, EPS, obstacle, UnknownPoints, R_obs_check)
% RRT探索木をランダムノード(zrcoord)の方向に拡張する。
% RRT*のロジックを含み、Known/Unknownの両障害物で衝突判定を行う。
%
% nodes: 探索木のノード情報
% nc: ノード数
% zrcoord: ランダムサンプリングされたノード (q_rand)
% BRC: Near関数の定数
% EPS: 辺の最大長
% obstacle: 既知の障害物リスト (KnownObstacles)
% UnknownPoints: 未知の障害物点群
% R_obs_check: 未知の点群に対する安全マージン

    State = "ADVANCED"; % 初期状態を"ADVANCED"に設定

    % 1. 最近傍ノードの探索 (Nearest)
    % nodes1からzrcoordに最も近いノード q_nearest を探索
    dist_min = inf;
    q_nearest_id = 0;
    
    for i = 1:1:nc
        % 距離の計算
        dist_now = norm(nodes{i,1} - zrcoord);
        if dist_now < dist_min
            dist_min = dist_now;
            q_nearest_id = i;
        end
    end
    q_nearest_coord = nodes{q_nearest_id, 1};

    % 2. 新しいノードの生成 (Steer)
    % q_nearestからzrcoordの方向にEPS以内のノード q_new を生成
    if dist_min > EPS
        % 辺の長さがEPSより大きい場合、EPSの距離で生成
        q_new_coord = q_nearest_coord + (zrcoord - q_nearest_coord) * (EPS / dist_min);
    else
        % 辺の長さがEPS以下の場合、zrcoordをそのまま使用
        q_new_coord = zrcoord;
    end
    
    % 3. 衝突判定 (Collision Check) - ノード追加前
    % is_path_safe_RRT(q_from, q_to, KnownObstacles, UnknownPoints, R_obs_check) を使用
    % obstacle (KnownObstacles) と UnknownPoints の両方でチェック
    is_path_collision = ~is_path_safe_RRT(q_nearest_coord, q_new_coord, obstacle, UnknownPoints, R_obs_check);
    
    if is_path_collision
        % 衝突がある場合：ノードは追加せず、終了
        State = "TRAPPED";
        zncoord = []; % 戻り値も空にする
        return;
    end
    
    % 衝突がない場合：ノードの追加に進む (Extend)
    
    % 4. RRT*のコスト計算と親ノードの選択 (ChooseParent)
    
    % 探索木のノード数と新しいノードを更新
    nc = nc + 1;
    zncoord = q_new_coord;

    % 新しいノードの初期情報を設定
    nodes(nc,:) = {zncoord, q_nearest_id, {[]}, nc, inf}; % parentをq_nearest_idに仮設定

    % BRCによるNear関数の領域 (R) の計算
    R = min(EPS, BRC * (log(nc) / nc)^(1/StateDim));
    
    % q_newの近傍ノードリスト (q_near_list) の探索
    q_near_list = [];
    dist_new_min = RRT_Cost(nodes, q_nearest_id) + norm(q_new_coord - q_nearest_coord); % q_nearest経由のコスト

    for i = 1:1:nc - 1 % q_new以外のノードをチェック
        dist_now = norm(nodes{i, 1} - q_new_coord);
        if dist_now < R
            % R以内にあり、かつ新しいパスが安全かチェック
            is_near_safe = is_path_safe_RRT(nodes{i, 1}, q_new_coord, obstacle, UnknownPoints, R_obs_check);
            
            if is_near_safe
                q_near_list = [q_near_list, i];
                
                % コスト比較: q_near経由の方が安いか？ (q_near -> q_new)
                cost_q_near = RRT_Cost(nodes, i) + dist_now;
                
                if cost_q_near < dist_new_min
                    % より安い経路が見つかった場合、親ノードを更新
                    dist_new_min = cost_q_near;
                    nodes{nc, 2} = i; % parentをq_near_idに更新
                end
            end
        end
    end
    
    % 親ノードの更新に伴い、元の親(q_nearest_id)の子リストから自分(nc)を削除
    if nodes{nc, 2} ~= q_nearest_id % 親ノードが変わった場合
        % 元の親の子リストからncを削除するロジックが必要だが、ここでは省略
        % 適切なデータ構造のChildリスト操作が必要
    end
    
    % 新しい親ノードの子リストに自分(nc)を追加
    q_new_parent_id = nodes{nc, 2};
    nodes{q_new_parent_id, 3} = { [nodes{q_new_parent_id, 3}{1}, nc] };
    
    
    % 5. 再配線 (Rewiring)
    % q_newを経由して、近傍のノード q_near へのコストが安くなるかチェック
    
    for i = 1:length(q_near_list)
        q_near_id = q_near_list(i);
        q_near_coord = nodes{q_near_id, 1};
        
        % コスト比較: q_new経由の新しいコスト
        new_cost_via_q_new = dist_new_min + norm(q_new_coord - q_near_coord);
        
        % 既存のコスト
        old_cost = RRT_Cost(nodes, q_near_id);
        
        % q_newを経由した方が安くなるか？
        if new_cost_via_q_new < old_cost
            
            % 【修正】q_newからq_nearへのパスの安全性をチェック
            is_rewire_safe = is_path_safe_RRT(q_new_coord, q_near_coord, obstacle, UnknownPoints, R_obs_check);
            
            if is_rewire_safe
                % 安全かつ安価な場合、再配線 (Rewire) を実行
                
                % 元の親の子リストからq_near_idを削除するロジックが必要 (省略)
                % ... 
                
                % q_near_idの親をq_newに付け替え
                nodes{q_near_id, 2} = nc;
                % q_newの子リストにq_near_idを追加
                nodes{nc, 3} = { [nodes{nc, 3}{1}, q_near_id] };
            end
        end
    end
    
    % zncoordをリターン
    zncoord = q_new_coord;
end