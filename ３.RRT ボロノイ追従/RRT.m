clearvars
close all

% 動画を保存するフラグ（true : 動画を保存する）
movieflag = 1;

seed =3;
BRC = 500;
Continueflag = 0;
numNodes = 1000;
EPS = 20;

rng(seed, 'twister');
%状態の次元数
StateDim = 3;
%最終的に得られた最適な経路
SP = [];

% ノード群のメモリ事前割り当て
nodes = table('Size', [numNodes, 7],'VariableTypes', ["double", "double", "double", "double", "double", "cell", "double"],'VariableNames',["c1", "c2", "c3", "cost", "parent","child", "id"]);
nodes = mergevars(nodes,{'c1', 'c2', 'c3'},'NewVariableName','coord');

%初期状態
nodes(1,:) = {[0,-150,50],0,0,{[]},1};

%目標状態
zgcoord = [0, 150, 50];
zgid = inf;

%グラフの描画限界
limitation = [-100, 100, -200, 200, 0, 100];

%障害物
obstacle1  = [-60, -30, -60, 20, 0, 100];      [vert1, fac1]   = vertObstacle(obstacle1);
obstacle2  = [ 30,  60, -60, 20, 0, 100];      [vert2, fac2]   = vertObstacle(obstacle2);
obstacle3  = [-30,  30, -20, 20, 0, 100];      [vert3, fac3]   = vertObstacle(obstacle3);
obstacle = [obstacle1; obstacle2; obstacle3];

% 初期状態,目標状態,障害物の描画
f = figure('Position',[100 100 650 650]);
ax = gca;
ax.FontSize = 12;
view(2)
axis(limitation)
daspect([1 1 1]);
grid on
hold on
xlabel('X','FontSize',15)
ylabel('Y','FontSize',15)
zlabel('Z','FontSize',15)
plot3(nodes{1,1}(1,1), nodes{1,1}(1,2), nodes{1,1}(1,3), 'o', 'MarkerEdgeColor','g', 'MarkerFaceColor','g', 'MarkerSize',5)
plot3(zgcoord(1), zgcoord(2), zgcoord(3), 'o', 'MarkerEdgeColor','r', 'MarkerFaceColor','r', 'MarkerSize',5)
patch('Vertices',vert1,'Faces',fac1,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969')
patch('Vertices',vert2,'Faces',fac2,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969')
patch('Vertices',vert3,'Faces',fac3,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969')
alpha(0.3)
drawnow

goalflag = 0;
nc = 1;

%--------------------------------動画の設定--------------------------------%
if movieflag == 1
    animenum = 1;
    Frame(animenum) = getframe(1);
    animenum = animenum + 1;
end
%-------------------------------------------------------------------------%

% Algorithm 1 : Line 2 : while i < N do
tic
while nc < numNodes
    clearvars znear znearest;
    % Algorithm 1 : Line 3 : x_rand ← Sample(i); i ← i + 1;
    px = makedist("Uniform", "Lower", limitation(1), "Upper", limitation(2));
    py = makedist("Uniform", "Lower", limitation(3), "Upper", limitation(4));
    pz = makedist("Uniform", "Lower", 30, "Upper", limitation(6));
    zrcoord = [random(px), random(py), random(pz)];
    %--------------------------------Extend--------------------------------%
    % Algorithm 1 : Line 4 : x_nearest ← Nearest(G,x);
    [val, znearest] = NearestNode(nodes, zrcoord, zgid, nc);
    % Algorithm 1 : Line 5 : x_new = Steer(x_nearest,x);
    zncoord = steer(zrcoord, znearest{1,1}, val, EPS);
    % Algorithm 1 : Line 6 : if RRTObstacleFree(x_nearest, x_new) then
    if ObstacleFree(zncoord, znearest{1,1}, obstacle)
        nc = nc + 1;
        % Algorithm 2 : Line 5-6
        cost = znearest{1,2} + norm(zncoord-znearest{1,1});
        nodes(nc, :) = {zncoord, cost, znearest{1,5}, {[]}, nc};
        nodes{znearest{1,5}, 4} = {[cell2mat(znearest{1,4}), nc]};
    %----------------------------------------------------------------------%
        % 目標状態への到達判定
        if ObstacleFree(nodes{nc,1}, zgcoord, obstacle) && ...
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

% 探索木を描画
for i = 2:1:nc
    if nodes{i,2} ~= inf
        parentid = nodes{i,3};
        line([nodes{parentid,1}(1), nodes{i,1}(1)], [nodes{parentid,1}(2), nodes{i,1}(2)], [nodes{parentid,1}(3), nodes{i,1}(3)], 'Color', '#0072BD', 'LineWidth', 0.1);
        plot3(nodes{i,1}(1),nodes{i,1}(2), nodes{i,1}(3), '.', 'Color', '#0072BD', 'MarkerSize',5)
        if movieflag == 1
            Frame(animenum) = getframe(1);
            animenum = animenum + 1;
        end
    end
end

% 探索木の中で最適な経路を描画
if goalflag == 1
    q_end = nodes(zgid,:);
    SP = [q_end; SP];
    while q_end.parent ~= 0
        start = q_end{1,3};
        line([q_end{1,1}(1), nodes{start,1}(1)], [q_end{1,1}(2), nodes{start,1}(2)], [q_end{1,1}(3), nodes{start,1}(3)], 'Color', 'r', 'LineWidth', 3)
        q_end = nodes(start,:);
        SP = [q_end; SP];%#ok<AGROW>
        drawnow
    end
end

fprintf("経路長：%f\n", SP{end,2})

if movieflag == 1
    Frame(animenum:animenum+100) = getframe(1);
    %animenum = animenum + 46;
end
% アニメーション書き出し用の処理
if movieflag == 1
    Frate = 90;
    v = VideoWriter('RRT_xy.mp4','MPEG-4');
    v.FrameRate = Frate; % Framerate
    open(v);
    writeVideo(v,Frame);
    close(v);
end