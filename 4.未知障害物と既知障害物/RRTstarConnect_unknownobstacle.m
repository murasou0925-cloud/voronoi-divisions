%% voronoiRRT_main_shuron用経路探索

% RRTStarConnect_shuron.m の関数定義を書き換える
function truepath = RRTstarConnect_unknownobstacle(start, goal, KnownObstacles, UnknownPoints)

% ... (初期設定：ツリーT_A, T_B の初期化、パラメータ設定など)
% Param.R_obs_check = 0.3; % 未知の点群に対する安全マージン（適宜設定）

% ... (中略)

% nodes内の情報（table）     : {[x, y, z], parent, child, id, otherid}
% [x, y, z]                 : ノードの座標                           1×3 double
% parent                    : 親ノードのid                           1×1 double
% child                     : 子ノードのid                           1×1 cell
% id                        : ノードのid                             1×1 double
% otherid                   : もう一つのノード群におけるノードのid     1×1 double

% 設定する値
% seed                      : 乱数のシード値
% BRC                       : Near関数の定数
% numNodes                  : ノードを追加する最大反復回数
% EPS                       : 辺の最大長

seed = 2;
BRC = 500;
numNodes = 100;
EPS = 5;

% 乱数生成器
rng(seed, 'twister');
% 状態の次元数
StateDim = 3;

% ノード群のメモリ事前割り当て
nodes1 = table('Size', [numNodes, 7],'VariableTypes', ["double", "double", "double", "double", "cell", "double", "double"],'VariableNames',["c1", "c2", "c3", "parent", "child", "id", "otherid"]);
nodes1 = mergevars(nodes1,{'c1', 'c2', 'c3'},'NewVariableName','coord');
nodes2 = nodes1;
% 初期状態を格納
nodes1(1,:) = {[start(1),start(2),start(3)],0,{[]},1,inf};
% 目標状態を格納
nodes2(1,:) = {[goal(1),goal(2),goal(3)],0,{[]},1,inf};

% グラフの描画限界
x_min =  -4;   x_max =4 ;
y_min = -7.5;   y_max = 7.5;
z_min =    0;   z_max =   5;

obstacle = KnownObstacles;

R_obs_check = 0.3; % 0.3m以内に未知の点があれば衝突と見なす

% 描画するとき用 =================================
% [vert1, fac1]   = vertObstacle(obstacle1);
% [vert2, fac2]   = vertObstacle(obstacle2);
% [vert3, fac3]   = vertObstacle(obstacle3);
% [vert4, fac4]   = vertObstacle(obstacle4);
% [vert5, fac5]   = vertObstacle(obstacle5);
% [vert6, fac6]   = vertObstacle(obstacle6);
%================================================



% 描画するとき用 =================================
% f = figure('Position',[100 100 600 500]); hold on; box on;
% ax = gca;
% ax.FontSize = 12;
% view(2)
% axis([x_min x_max y_min y_max z_min z_max])
% axis([-3 11 -3 8 0 5])
% daspect([1 1 1]);
% hold on
% xlabel('X','FontSize',15)
% ylabel('Y','FontSize',15)
% zlabel('Z','FontSize',15)
% plot3(nodes1{1,1}(1), nodes1{1,1}(2), nodes1{1,1}(3), 'o', 'MarkerEdgeColor','r', 'MarkerFaceColor','r', 'MarkerSize',6)
% plot3(nodes2{1,1}(1), nodes2{1,1}(2), nodes2{1,1}(3), 'p', 'MarkerEdgeColor','r', 'MarkerSize',10)
% O(1)  = patch('Vertices',vert1,'Faces',fac1,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
% O(2)  = patch('Vertices',vert2,'Faces',fac2,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
% O(3)  = patch('Vertices',vert3,'Faces',fac3,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
% O(4)  = patch('Vertices',vert4,'Faces',fac4,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
% O(5)  = patch('Vertices',vert5,'Faces',fac5,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
% O(6)  = patch('Vertices',vert6,'Faces',fac6,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
% for n=1:6
%     alpha(O(n), 0.3)
% end
% drawnow
%================================================

nc1 = 1; % z_init側の探索木のノード数
nc2 = 1; % z_goal側の探索木のノード数
nc = 2;  % z_init側とz_goal側の探索木のノードの総数
SWAP = 1;
while nc < numNodes
    % 探索範囲
    %壁と衝突しないために探索範囲にバッファを設けている
    px = makedist("Uniform", "Lower", x_min, "Upper", x_max);
    py = makedist("Uniform", "Lower", y_min, "Upper", y_max);
    pz = makedist("Uniform", "Lower", z_min, "Upper", z_max);
    %zrcoord = [random(px), random(py), random(pz)];
    zrcoord = [random(px), random(py), 3];   % 初期位置も目標位置もz座標が3であるため，探索範囲も3固定
   switch SWAP
        case 1
            % 【修正】引数を RRT_ExtendStar に合うように調整し、R_obs_check を追加
            % RRT_ExtendStar(nodes, nc, zrcoord, BRC, StateDim, EPS, obstacle, UnknownPoints, R_obs_check)
            [State, nodes1, nc1, zncoord] = RRT_ExtendStar(nodes1, nc1, zrcoord, BRC, StateDim, EPS, obstacle, UnknownPoints, R_obs_check); 
            nc = nc1 + nc2; % 【修正】ノード総数 nc を更新

            if State ~= "TRAPPED"
                % 【修正】RRT_ConnectStar にも R_obs_check を追加 (引数11個)
                [nodes1, nodes2, nc1, nc2, nc] = RRT_ConnectStar(nodes1, nodes2, nc1, nc2, nc, zncoord, BRC, StateDim, EPS, obstacle, UnknownPoints, R_obs_check);
            end

        case 2
            % 【修正】引数を RRT_ExtendStar に合うように調整
            [State, nodes2, nc2, zncoord] = RRT_ExtendStar(nodes2, nc2, zrcoord, BRC, StateDim, EPS, obstacle, UnknownPoints, R_obs_check);
            nc = nc1 + nc2; % 【修正】ノード総数 nc を更新

            if State ~= "TRAPPED"
                % 【修正】RRT_ConnectStar にも R_obs_check を追加 (引数11個)
                [nodes2, nodes1, nc2, nc1, nc] = RRT_ConnectStar(nodes2, nodes1, nc2, nc1, nc, zncoord, BRC, StateDim, EPS, obstacle, UnknownPoints, R_obs_check);
            end
            SWAP = 1;
    end
end

% 解となる経路の導出
cost = inf;
relaynum = 0;
for i = 1:1:nc1
    if nodes1{i,5} ~= inf
        cost_dash = RRT_Cost(nodes1,i) + RRT_Cost(nodes2,nodes1{i,5});
        if cost_dash < cost
            cost = cost_dash;
            relaynum = i;
        end
    end
end

% 解となる経路のz_init側のノードID
trueid1 = nodes1{relaynum,4};
pid1 = trueid1;
while pid1 ~= 1
    trueparent1 = nodes1{pid1,2};
    trueid1 = [trueparent1 ; trueid1];
    pid1 = trueparent1;
end
trueid1(1,:) = [];
% 解となる経路のz_goal側のノードID
trueid2 = nodes1{relaynum,5};
pid2 = trueid2;
while pid2 ~= 1
    trueparent2 = nodes2{pid2,2};
    trueid2 = [trueid2 ; trueparent2];
    pid2 = trueparent2;
end
trueid2(1,:) = [];
% 解となる経路の各ノードの座標
truepath = [];
for i = 1:length(trueid1)
    truepath = [truepath ; nodes1{trueid1(i),1}];
end
for i = 1:length(trueid2)
    truepath = [truepath ; nodes2{trueid2(i),1}];
end

% 描画するとき用 =================================
% 探索木を描画
    % for i = 2:1:nc1
    %    parentid = nodes1{i,2};
    %    line([nodes1{parentid,1}(1), nodes1{i,1}(1)], [nodes1{parentid,1}(2), nodes1{i,1}(2)], [nodes1{parentid,1}(3), nodes1{i,1}(3)], 'Color', '#0072BD', 'LineWidth', 0.5);
    %    plot3(nodes1{i,1}(1),nodes1{i,1}(2), nodes1{i,1}(3), '.', 'Color', '#0072BD')
    %    if movieflag == 1
    %         Frame(animenum) = getframe(1);
    %         animenum = animenum + 1;
    %    end
    %    hold on
    % end
    % for i = 2:1:nc2
    %    parentid = nodes2{i,2};
    %    line([nodes2{parentid,1}(1), nodes2{i,1}(1)], [nodes2{parentid,1}(2), nodes2{i,1}(2)], [nodes2{parentid,1}(3), nodes2{i,1}(3)], 'Color', '#0072BD', 'LineWidth', 0.5);
    %    plot3(nodes2{i,1}(1),nodes2{i,1}(2), nodes2{i,1}(3), '.', 'Color', '#0072BD')
    %    if movieflag == 1
    %         Frame(animenum) = getframe(1);
    %         animenum = animenum + 1;
    %    end
    %    hold on
    % end

% 解となる経路を描画
% child1 = nodes1(relaynum,:);
% child2 = nodes2(nodes1{relaynum,5},:);
% SP = child1;
% while child1.parent ~= 0
%     parent1 = child1{1,2};
%     line([child1{1,1}(1), nodes1{parent1,1}(1)],[child1{1,1}(2), nodes1{parent1,1}(2)],[child1{1,1}(3), nodes1{parent1,1}(3)],'Color','r','LineWidth',2)
%      % ノードを描画
%     plot3(child1{1,1}(1), child1{1,1}(2), child1{1,1}(3), ...
%           'o', 'MarkerSize', 6, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     child1 = nodes1(parent1,:);
%     SP = [child1; SP]; %#ok<AGROW>
% end
% while child2.parent ~= 0
%     parent2 = child2{1,2};
%     line([child2{1,1}(1), nodes2{parent2,1}(1)],[child2{1,1}(2), nodes2{parent2,1}(2)],[child2{1,1}(3), nodes2{parent2,1}(3)],'Color','r','LineWidth',2)
%      % ノードを描画
%     plot3(child2{1,1}(1), child2{1,1}(2), child2{1,1}(3), ...
%           'o', 'MarkerSize', 5, 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b');
%     child2 = nodes2(parent2,:);
%     SP = [SP; child2]; %#ok<AGROW>
% end
%================================================

fprintf("経路長：%f\n",cost);