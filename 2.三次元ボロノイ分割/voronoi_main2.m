clearvars
close all

% 初期設定
f = figure('Position',[100 100 800 800]); hold on; box on;
xlim([-0.5 17.5]); ylim([0 12]); zlim([0 5]);
animenum = 1;

% 各種設定
Param = Init;

Param.start = [ 3, 6, 3];    % 機体の初期位置
Param.goal  = [15, 6, 3];    % 機体の目標位置

% ゴールの描画
for qnum = 1:1
    scatter3(Param.goal(qnum,1),Param.goal(qnum,2),Param.goal(qnum,3), 100, 'p','MarkerEdgeColor',Param.Color(qnum));
end

% 障害物の描画 ------------------------%
obstacle1 = [0, 17,  8,  9,  0,  5];      [vert1, fac1]   = vertObstacle(obstacle1);
obstacle2 = [0, 17,  3,  4,  0,  5];      [vert2, fac2]   = vertObstacle(obstacle2);
O(1)  = patch('Vertices',vert1,'Faces',fac1,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(2)  = patch('Vertices',vert2,'Faces',fac2,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
for n=1:length(O)
    alpha(O(n), 0.3)
end
% 円柱の設定
r = 0.5;   % 半径;
h = 5;   % 高さ
%円柱1
x_center1 = 6;  % x座標の中心
y_center1 = 7;  % y座標の中心
z_base = 0;    % z座標の底面
%円柱2
x_center2 = 11;
y_center2 = 5.5;
% 円柱の座標を生成
[X, Y, Z] = cylinder(r);
Z = Z * h + z_base;  % 高さ調整（z方向）

% 円柱を描画
axis equal
cylinderPlot = surf(X + x_center1, Y + y_center1, Z, 'FaceColor', 'k', 'EdgeColor', 'none');
cylinderPlot2 = surf(X + x_center2, Y + y_center2, Z, 'FaceColor', 'k', 'EdgeColor', 'none');
alpha(cylinderPlot, 0.3);
alpha(cylinderPlot2, 0.3);

% 円をプロットするための角度
theta = linspace(0, 2*pi, 100);
x_circle1 = r * cos(theta) + x_center1;
y_circle1 = r * sin(theta) + y_center1;
x_circle2 = r * cos(theta) + x_center2;
y_circle2 = r * sin(theta) + y_center2;

fill3(x_circle1, y_circle1, (z_base + h) * ones(size(theta)), 'k', 'FaceAlpha', 0.3);
fill3(x_circle2, y_circle2, (z_base + h) * ones(size(theta)), 'k', 'FaceAlpha', 0.3);
%--------------------------------------%

%% CoppeliaSimとの通信の初期設定
sim = remApi("remoteApi"); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1',19997,true,true,5000,5);
sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);

if clientID > -1
    disp('Connected to remote API server');
    sim.simxAddStatusbarMessage(clientID, 'Simulation Started!', sim.simx_opmode_oneshot);
    sim.simxSynchronous(clientID, true); 
else
    disp('Connection Error');
    quit
end

% Handle Setting
[~, Quad]   = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_blocking);
[~, target] = sim.simxGetObjectHandle(clientID, 'Quadcopter_target', sim.simx_opmode_blocking);

% Get position and orientation data Setting
[~, QuadPos] = sim.simxGetObjectPosition(clientID, Quad, -1, sim.simx_opmode_streaming);
[~, QuadAng] = sim.simxGetObjectOrientation(clientID, Quad, -1, sim.simx_opmode_streaming);
pause(0.1);

% Get sensor data
[~, ~] = sim.simxGetStringSignal(clientID, 'ranges11', sim.simx_opmode_streaming);
pause(0.1);

%% メインループ
% deltaを0.05とし，tが1からLIMまで更新を行う
for t = 1:Param.delta:Param.LIM
    % ボロノイ境界と一時目標点を更新のたびに消して再描画する
    if t > 1 
        delete(drawcurrpos); delete(drawsectp); delete(drawline); 
        delete(td);
        if exist('drawvoronoi')
            delete(drawvoronoi);
        end
        if exist('sensd1')
            delete(sensd1);
        end
    end

    % 時間の描画
    td=annotation('textbox', Param.tboxdim, 'String', ['{\it t} = ' num2str(t-1)],'EdgeColor','k','BackgroundColor','w','FontName','Times New Roman', 'FontSize',20);

    % Get position and orientation
    [~, QuadPos] = sim.simxGetObjectPosition(clientID, Quad, -1, sim.simx_opmode_buffer);
    [~, QuadAng] = sim.simxGetObjectOrientation(clientID, Quad, -1, sim.simx_opmode_buffer);
    pause(0.1);
    
    Param.X = QuadPos;
    % tが1のとき，もしくは0.3のとき，ボロノイ分割を行う
    if (rem(t-1,0.3) == 0 || t == 1)
        % Get sensor data
        [~, tmp_ranges] = sim.simxGetStringSignal(clientID, 'ranges11', sim.simx_opmode_buffer);
        ranges = sim.simxUnpackFloats(tmp_ranges);

        % ボロノイ分割に用いる母点を計算
        [Param, ranges] = voronoi_Seed(Param, ranges, QuadAng, QuadPos, animenum);

        % ボロノイ分割の計算
        tic   % 計算時間を計測．ticで開始，tocで終了
        [Param, alshp, secpoint, dist, vec2] = voronoi_Generator(Param);
        elapsedTime(qnum,animenum) = toc;  % ボロノイ分割の計算にかかった時間を保存
        eT = (elapsedTime)';
        eT_max = max(eT);   % 計算時間の最大値
        eT_min = min(eT);   % 計算時間の最小値
        eT_ave = mean(eT);  % 計算時間の平均値
    end
   
    %% 位置の更新（Coppeliasim上のクワッドロータの目標位置を動かす）

    % ゴールが近ければ一時目標位置もターゲットボールもゴールに固定
    if norm(Param.goal(1,:) - Param.X(1,:)) < 0.4
        nexttarget = Param.goal(1,:);
        sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
    % t=1のとき（初回）と，ゴールが近くにない場合はターゲットボールの位置を更新
    elseif t == 1 || norm(Param.X(1,:) - nexttarget) < 0.20
        CheckDist = dist > Param.velocity; % サブゴールとの距離が次の１ステップサイズより大きいか確認
        if CheckDist % サブゴールとの距離が遠ければ動かす
            nexttarget = Param.X(1,:) + Param.velocity .* vec2; % 位置を更新(一定速度)
            sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
        else % サブゴールとの距離が近ければ一時目標位置をサブゴールに固定
            nexttarget = secpoint;
            sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
        end
    end

     %% 描画
    xlim([-0.5 17.5]); ylim([0 12]); zlim([0 5]);
    hold on; box on;
    % 三次元ボロノイ領域の描画
    if rem(t-1,0.3) == 0 || t == 1
        drawvoronoi = plot(alshp, 'FaceAlpha', .5, 'EdgeAlpha', .2);
    end
    % 現在位置の描画
    drawcurrpos = scatter3(Param.X(1,1), Param.X(1,2), Param.X(1,3), 100, '^', 'filled', 'MarkerFaceColor', Param.Color(1)); 
    scatter3(Param.X(1,1), Param.X(1,2), Param.X(1,3), 3, 'filled', 'MarkerFaceColor', Param.Color(1));
    % サブゴールの描画
    drawsectp = scatter3(secpoint(1), secpoint(2), secpoint(3), 5, 'MarkerEdgeColor', Param.Color(1));  
    % 直線の描画
    drawline = plot3([secpoint(1) Param.X(1,1)], [secpoint(2) Param.X(1,2)], [secpoint(3) Param.X(1,3)], 'Color', Param.Color(1));
    % センサの描画
    if ~isempty(ranges)
        if rem(t-1,0.3) == 0 || t == 1
            sensd1 = scatter3(Param.newnearP(:,1), Param.newnearP(:,2), Param.newnearP(:,3), 10, 'filled', Param.Color(1), 'MarkerFaceAlpha', 1);
        end
    end

    % 軸範囲を再設定
    xlim([-0.5 17.5]); ylim([0 12]); zlim([0 5]);

    drawnow

   % --- 指定時刻での画像保存処理 (ここを追加) ---
    % 保存したい時刻のリスト
    target_times = [0, 6, 12, 17.05];
    
    % 現在の表示時刻 (ループのtは1から始まるため、表示はt-1)
    current_disp_time = t - 1;
    
    % 浮動小数点の誤差を考慮し、現在時刻がターゲットに近い場合に保存を実行
    % (Param.delta の半分以下の誤差なら一致とみなす)
    if any(abs(current_disp_time - target_times) < Param.delta / 2)
        % ファイル名を生成 (例: snapshot_t_17.05.png)
        img_filename = sprintf('snapshot_t_%g.png', current_disp_time);
        
        % 画像をPNG形式で保存
        saveas(gcf, img_filename);
        
        % もしMATLABのバージョンがR2020a以降で、余白をカットして高画質で保存したい場合は
        % 上の saveas の代わりに以下を使うと綺麗に出力されます
        % exportgraphics(gcf, img_filename, 'Resolution', 300);
        
        disp(['Image saved: ', img_filename]);
    end
    % ------------------------------------------ 

    Frame(animenum)=getframe(1);
    animenum=animenum+1;   

    % ゴールに到達したらループを抜ける
    if norm(Param.goal(1,:) - Param.X(1,:)) < 0.5
        break;
    end

   

    % シミュレーションのステップを進める
    sim.simxSynchronousTrigger(clientID);
end

% stop the simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
% 終了処理
sim.simxFinish(clientID);
sim.delete();

%% アニメーション書き出し用の処理
Frate = 20;
v = VideoWriter('voronoi3D.avi');
v.FrameRate = Frate; % Framerate
open(v);
writeVideo(v,Frame);
close(v);
savefig('stepfinal.fig');
%%