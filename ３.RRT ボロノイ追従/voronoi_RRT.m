%% voronoiRRT_main_shuron用経路探索

function [truepath] = voronoi_RRT(start, goal)

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
numNodes = 300;
EPS = 1;


% 乱数生成器
rng(seed, 'twister');
% 状態の次元数
StateDim = 3;
Continueflag = 0;
SP = [];

% ノード群のメモリ事前割り当て
nodes = table('Size', [numNodes, 7],'VariableTypes', ["double", "double", "double", "double", "double", "cell", "double"],'VariableNames',["c1", "c2", "c3", "cost", "parent","child", "id"]);
nodes = mergevars(nodes,{'c1', 'c2', 'c3'},'NewVariableName','coord');
% 初期状態を格納
nodes(1,:) = {start, 0, 0, {[]}, 1};%目標状態
zgcoord = goal;
zgid = inf;



% グラフの描画限界
x_min =  -8;   x_max = 8;
y_min = -7;   y_max = 7;
z_min =    0;   z_max =   5;

% 障害物の情報

obstacle1 = [-7, -5.5,-5 ,  5, 0, 5];
obstacle2 = [ 5.5, 7, -5,  5, 0, 5];
obstacle3 = [-1.75, -1.25, -1.5, 0.5, 0, 5];
obstacle4 = [1.25, 1.75,  -1.5,  0.5, 0, 5];
obstacle5 = [ -1.75,  1.75, 0.5, 1.25 , 0, 5];


% 描画するとき用 =================================
% [vert1, fac1]   = vertObstacle(obstacle1);
% [vert2, fac2]   = vertObstacle(obstacle2);
% [vert3, fac3]   = vertObstacle(obstacle3);
% [vert4, fac4]   = vertObstacle(obstacle4);
% [vert5, fac5]   = vertObstacle(obstacle5);
% [vert6, fac6]   = vertObstacle(obstacle6);
%================================================

obstacle = [ obstacle1;  obstacle2;  obstacle3;  obstacle4;  obstacle5;];

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

goalflag = 0;
nc = 1;
tic
while nc < numNodes
    % 探索範囲
    %壁と衝突しないために探索範囲にバッファを設けている
    px = makedist("Uniform", "Lower", x_min, "Upper", x_max);
    py = makedist("Uniform", "Lower", y_min, "Upper", y_max);
    pz = makedist("Uniform", "Lower", 2.5, "Upper", 3.5);
    %zrcoord = [random(px), random(py), random(pz)];
    zrcoord = [random(px), random(py), 3];   % 初期位置も目標位置もz座標が3であるため，探索範囲も3固定
    %--------------------------------Extend--------------------------------%
    % Algorithm 1 : Line 4 : x_nearest ← Nearest(G,x);
    [val, znearest] = RRT_NearestNode(nodes, zrcoord,  nc);
    % Algorithm 1 : Line 5 : x_new = Steer(x_nearest,x);
    zncoord = RRT_steer(zrcoord, znearest{1,1}, val, EPS);
    % Algorithm 1 : Line 6 : if RRTObstacleFree(x_nearest, x_new) then
    if RRT_ObstacleFree(zncoord, znearest{1,1}, obstacle)
        nc = nc + 1;
        % Algorithm 2 : Line 5-6
        cost = znearest{1,2} + norm(zncoord-znearest{1,1});
        nodes(nc, :) = {zncoord, cost, znearest{1,5}, {[]}, nc};
        nodes{znearest{1,5}, 4} = {[cell2mat(znearest{1,4}), nc]};
    %---------------------------------------------------------------------
    % 目標状態への到達判定
        if RRT_ObstacleFree(nodes{nc,1}, zgcoord, obstacle) && ...
                norm(nodes{nc,1}-zgcoord) < EPS && ...
                goalflag == 0
            % z_goalをノード群に加える
            nc = nc + 1;
            zgid = nc;
            nodes(nc,:) = {zgcoord, nodes{nc-1,2}+norm(nodes{nc-1,1}-zgcoord),nc-1,{[]},nc};
            % z_newの子にz_goalを加える
            nodes{nc-1, 4} = {[cell2mat(nodes{nc-1, 4}), nc]};
            % 目標状態の到達判定のフラグをtrueにする
            goalflag = 1;
            if Continueflag ~= 1
                break
            end
        end
    end
end
CT = toc;
fprintf("計算時間：%f秒\n", CT);




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

        % --- 追加：スタート地点（1番目の要素）を削除 ---
        if size(truepath, 1) > 1
            truepath(1, :) = []; 
        end
    end

end