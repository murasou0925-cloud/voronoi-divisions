function [truepath,goalflag] = RRT_unKnownobstacle(start,goal,KnownObstacles,UnknownPoints)
%RRT_UNKNOWNOBSTACLE この関数の概要をここに記述
%   詳細説明をここに記述
seed =3;
BRC = 500;
Continueflag = 0;
numNodes = 1000;
EPS = 1.5;
R_unknown_check=0.3;

rng(seed, 'twister');
%状態の次元数
StateDim = 3;
%最終的に得られた最適な経路
SP = [];

% ノード群のメモリ事前割り当て
nodes = table('Size', [numNodes, 7],'VariableTypes', ["double", "double", "double", "double", "double", "cell", "double"],'VariableNames',["c1", "c2", "c3", "cost", "parent","child", "id"]);
nodes = mergevars(nodes,{'c1', 'c2', 'c3'},'NewVariableName','coord');

%初期状態
nodes(1,:) = {start,0,0,{[]},1};

%目標状態
zgcoord = goal;
zgid = inf;

%グラフの描画限界
limitation = [-3.7, 3.7, -7.5, 7.5, 0, 5];

obstacle = KnownObstacles;

R_obs_check = 0.3; % 0.3m以内に未知の点があれば衝突と見なす



goalflag = 0;
nc = 1;

% Algorithm 1 : Line 2 : while i < N do
tic
while nc < numNodes
    clearvars znear znearest;
    % Algorithm 1 : Line 3 : x_rand ← Sample(i); i ← i + 1;
    px = makedist("Uniform", "Lower", limitation(1), "Upper", limitation(2));
    py = makedist("Uniform", "Lower", limitation(3), "Upper", limitation(4));
   
    zrcoord = [random(px), random(py), 3.0];
    %--------------------------------Extend--------------------------------%
    % Algorithm 1 : Line 4 : x_nearest ← Nearest(G,x);
    [val, znearest] = RRT_NearestNode(nodes, zrcoord, nc);
    % Algorithm 1 : Line 5 : x_new = Steer(x_nearest,x);
    zncoord = RRT_steer(zrcoord, znearest{1,1}, val, EPS);
  % 4. 衝突判定: 既知障害物と未知点群の両方をチェック
        is_known_free = RRT_ObstacleFree(zncoord, znearest{1,1}, obstacle);
        
      % B. 未知障害物（★ここを修正★）
        % is_path_collision_unknown は「衝突したらtrue」なので、「~」をつけて反転させる
        is_unknown_hit = is_path_collision_unknown(znearest{1,1}, zncoord, UnknownPoints, R_unknown_check);
        is_unknown_free = ~is_unknown_hit;
            
        % 両方クリア（既知フリー かつ 未知フリー）ならノード追加
        if is_known_free && is_unknown_free
            nc = nc + 1;
            
            % 新しいノードのコスト計算
            new_cost = znearest{1,2} + norm(zncoord - znearest{1,1});
            
            % 新しいノードの追加: {coord, cost, parent, child, id}
           
            nodes(nc, :) = {zncoord, new_cost, znearest{1,5}, {[]}, nc}; 
            
            % 親ノードの子リストを更新 (childは4列目)
            nodes{znearest{1,5}, 4} = {[cell2mat(znearest{1,4}), nc]};
       % 5. 目標状態への到達判定
            is_goal_known_free = RRT_ObstacleFree(nodes{nc,1}, zgcoord, obstacle);
            % ★修正箇所★ ゴールへ向かう線分も同様にチェック
                is_goal_unknown_hit = is_path_collision_unknown(nodes{nc,1}, zgcoord, UnknownPoints, R_unknown_check);
                is_goal_unknown_free = ~is_goal_unknown_hit;

            if is_goal_known_free && is_goal_unknown_free && ...
                    norm(nodes{nc,1} - zgcoord) < EPS && ...
                    goalflag == 0
                
                % ゴールノードの追加
                nc = nc + 1;
                zgid = nc;
                new_cost = nodes{nc-1,2} + norm(nodes{nc-1,1} - zgcoord);
                nodes(nc,:) = {zgcoord, new_cost, nc-1, {[]}, nc};
                
                % 親ノードの子リストを更新
                nodes{nc-1, 4} = {[cell2mat(nodes{nc-1, 4}), nc]};
                goalflag = 1;
                break;
                end
        end
    end
CT = toc;
    fprintf("RRT探索時間：%f秒\n", CT);




% 6. 経路の導出 (Backtracking)
    truepath = [];
    if goalflag == 1
        q_end = nodes(zgid,:);
        truepath = [q_end{1,1}; truepath]; % ゴールノード
        
        while q_end.parent ~= 0
            parent_id = q_end{1,3}; % parent IDは3列目
            parent_coord = nodes{parent_id, 1};
            truepath = [parent_coord; truepath]; 
            q_end = nodes(parent_id,:);
        end
    end

end