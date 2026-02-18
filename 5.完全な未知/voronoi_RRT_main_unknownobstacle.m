

clearvars
close all

% 初期設定
f = figure('Position',[100 100 800 800]); hold on; box on;
xlim([-8 8]); ylim([-9 9]); zlim([0 5]);
animenum = 1;


% 各種設定
Param = Init;


Param.R_replan = 1.0; % 例: StepGoal到達と見なす距離閾値



% スタートとゴール，RRTで探索した経路
Param.start = [ 0, -6, 3];    % 機体の初期位置
Param.goal  = [ -2, 5, 3];    % 機体の目標位置

% 障害物の描画
 obstacle1 = [-6, -4,-7.5 ,  7.5, 0, 5];      [vert1, fac1]   = vertObstacle(obstacle1);
 obstacle2 = [4 , 6, -7.5,  7.5, 0, 5];      [vert2, fac2]   = vertObstacle(obstacle2);
 obstacle3 = [-6, 6,- 9.5, -7.5, 0, 5];      [vert3, fac3]   = vertObstacle(obstacle3);
 obstacle4= [-6, 6,7.5 , 9.5, 0, 5];          [vert4, fac4]   = vertObstacle(obstacle4);
 obstacle5 = [-4 , 1, -3.6, -3.1 , 0, 5];      [vert5, fac5]   = vertObstacle(obstacle5);
 obstacle6 = [-0.175, 0.025,0.05, 3.05, 0, 5];      [vert6, fac6]   = vertObstacle(obstacle6);
 obstacle7= [0, 4,3.075, 3.275, 0, 5];          [vert7, fac7]   = vertObstacle(obstacle7);


O(1)  = patch('Vertices',vert1,'Faces',fac1,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(2)  = patch('Vertices',vert2,'Faces',fac2,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(3)  = patch('Vertices',vert3,'Faces',fac3,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(4)  = patch('Vertices',vert4,'Faces',fac4,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(5)  = patch('Vertices',vert5,'Faces',fac5,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(6)  = patch('Vertices',vert6,'Faces',fac6,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');
O(7)  = patch('Vertices',vert7,'Faces',fac7,'FaceVertexCData',hsv(6),'FaceColor','#696969', 'EdgeColor', '#696969');


for n=1:length(O)
    alpha(O(n), 0.05)
end

%既知障害物リストの定義
KnownObstacles = [
    % obstacle1; 
    % obstacle2; 
    % obstacle3; 
   
];

UnknownPoints=[];
% RRT*-Connectでスタートからゴールまで経路を探索し，経由したノードをtruepathに保存
[truepath,initial_goalflag] = RRT_unKnownobstacle(Param.start, Param.goal, KnownObstacles,UnknownPoints ); % 初期はUnknownPoints=[]で実行
% ゴールの描画
for qnum = 1:1
    scatter3(Param.goal(qnum,1),Param.goal(qnum,2),Param.goal(qnum,3), 100, 'p','MarkerEdgeColor',Param.Color(qnum));
end





%% CoppeliaSimとの通信の初期設定
sim = remApi("remoteApi"); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections

clientID = sim.simxStart('127.0.0.1',19997,true,true,5000,5);
sim.simxStartSimulation(clientID,sim.simx_opmode_blocking);

if clientID > -1
    disp('Connected to remote API server');
    pause(0.1);
    sim.simxAddStatusbarMessage(clientID, 'Simulation Started!', sim.simx_opmode_oneshot);
    sim.simxSynchronous(clientID, true); 
else
    disp('Connection Error');
    quit
end

% Handle Setting
[~, Quad]   = sim.simxGetObjectHandle(clientID, 'Quadcopter', sim.simx_opmode_blocking);
[~, target] = sim.simxGetObjectHandle(clientID, 'target', sim.simx_opmode_blocking);

% Get position and orientation data Setting
[~, QuadPos] = sim.simxGetObjectPosition(clientID, Quad, -1, sim.simx_opmode_streaming);
[~, QuadAng] = sim.simxGetObjectOrientation(clientID, Quad, -1, sim.simx_opmode_streaming);
pause(0.1);

% Get sensor data
[~, ~] = sim.simxGetStringSignal(clientID, 'ranges11', sim.simx_opmode_streaming);
%% 
pause(0.1);

%% メインループ
g = 2;    % truepath(g,:)とし，まずはtruepath(1,:)とする



for t = 1:Param.delta:Param.LIM
    % ボロノイ境界と一時目標点を更新のたびに消して再描画する
    if t > 1 
        delete(drawcurrpos); delete(drawsectp); delete(drawline); 
        delete(td);
        if exist('drawstepgoal')
            delete(drawstepgoal);
        end
        if exist('drawvoronoi')
            delete(drawvoronoi);
        end
        if exist('sensd1')
            delete(sensd1);
        end
        if exist('drawnextstepgoal')
            delete(drawnextstepgoal);
        end
    end

    % 時間の描画
    td=annotation('textbox', Param.tboxdim, 'String', ['{\it t} = ' num2str(t-1)],'EdgeColor','k','BackgroundColor','w','FontName','Times New Roman', 'FontSize',20);

    % Get position and orientation
    [~, QuadPos] = sim.simxGetObjectPosition(clientID, Quad, -1, sim.simx_opmode_buffer);
    [~, QuadAng] = sim.simxGetObjectOrientation(clientID, Quad, -1, sim.simx_opmode_buffer);
    pause(0.1);
    
    Param.X = QuadPos;
    Param.stepgoal = truepath(g,:);
    % tが1のとき，もしくは0.3のとき，ボロノイ分割を行う
    if (rem(t-1,0.3) == 0 || t == 1)
        % Get sensor data
        [~, tmp_ranges] = sim.simxGetStringSignal(clientID, 'ranges11', sim.simx_opmode_buffer);
        ranges = sim.simxUnpackFloats(tmp_ranges);

        % ボロノイ分割に用いる母点を計算
        [Param, ranges] = voronoi_Seed(Param, ranges, QuadAng, QuadPos, animenum);


        %未知障害物識別

      
        % --- 設定値（事前に定義しておくか、ここで書く） ---
        SensorMaxRange = 3.0;  % ★センサの最大計測距離（CoppeliaSimの設定に合わせる）
        RangeThreshold = 0.1;  % 誤差許容範囲（最大距離より0.1m手前なら除外）
        % ---------------------------------------------
        numPoints=size(Param.newnearP,1);
        numKnownObstacles=size(KnownObstacles,1);

        for i = 1:numPoints
            P_i = Param.newnearP(i, :);

            % 【修正1】「何もない点（最大距離の点）」を除去
            % ドローン位置(Param.X)からの距離を計算
            dist_from_sensor = norm(P_i - Param.X(1,:));

            % 距離が最大計測距離に近ければ「空振り」とみなして無視
            if dist_from_sensor >= (SensorMaxRange - RangeThreshold)
                continue; % 次の点へ（これは障害物ではない）
            end


            isKnown = false;

            % すべての既知障害物と比較
            for j = 1:numKnownObstacles
                O_j = KnownObstacles(j, :);

            if is_in_obstacle(P_i,O_j)
                isKnown=true;
                break;
            end
            end

            %どの障害物にも含まれないなら未知障害物として追加
            if isKnown
               continue
            end

            % ------------------------------------
            % 【結果】ここまで残ったのが「未知障害物」
            % ------------------------------------
            UnknownPoints = [UnknownPoints; P_i];
        end



        % ボロノイ分割の計算
        tic   % 計算時間を計測．ticで開始，tocで終了
        [Param, alshp, secpoint, dist, vec2] = voronoi_Generator(Param);
        elapsedTime(qnum,animenum) = toc;  % ボロノイ分割の計算にかかった時間を保存
        eT = (elapsedTime)';
        eT_max = max(eT);   % 計算時間の最大値
        eT_min = min(eT);   % 計算時間の最小値
        eT_ave = mean(eT);  % 計算時間の平均値


        disp(['t=', num2str(t)]);
        disp(['Dist: ', num2str(dist)]);
       
  
   
    end
   
   %% 位置の更新（Coppeliasim上のクワッドロータの目標位置を動かす）

    % ゴールが近ければ一時目標位置もターゲットボールもゴールに固定
    if norm(Param.goal(1,:) - Param.X(1,:)) < 0.75
        nexttarget(1,:) = Param.goal(1,:);
        sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
        disp(['ゴール付近 ', num2str(nexttarget)]);
    % t=1のとき（初回）と，ゴールが近くにない場合はターゲットボールの位置を更新
    elseif t == 1 || norm(Param.X(1,:) - nexttarget) < 0.20
         disp(['dist=', num2str(dist)]);
         disp(['Pos  = ', num2str(Param.X(1,:))]); % ここに追加
        CheckDist = dist > Param.velocity; % サブゴールとの距離が次の１ステップサイズより大きいか確認
        if CheckDist % サブゴールとの距離が遠ければ動かす
            nexttarget = Param.X(1,:) + Param.velocity .* vec2; % 位置を更新(一定速度)
            sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
            disp('サブゴールを追従');
        else % デッドロック発生
           [new_truepath,initial_goalflag] = RRT_unKnownobstacle(Param.X(1,:), Param.goal, KnownObstacles, UnknownPoints);
           
               truepath = new_truepath; 
               g=2;
               Param.stepgoal = truepath(g,:); 
               % 2. 【重要】新しいstepgoalに基づいて、再度ボロノイ生成を行う
           % これにより、新しいルートへ向かうための正しいベクトル(vec2)と距離(dist)を再計算する

           % Get sensor data
              [~, tmp_ranges] = sim.simxGetStringSignal(clientID, 'ranges11', sim.simx_opmode_buffer);
              ranges = sim.simxUnpackFloats(tmp_ranges);

             % ボロノイ分割に用いる母点を計算
             [Param, ranges] = voronoi_Seed(Param, ranges, QuadAng, QuadPos, animenum);

              [Param, alshp, secpoint, dist, vec2] = voronoi_Generator(Param);
           
           % 3. その場で待機せず、新しい方向に少し動かす（スタック回避のため）
           % もし「1ステップ目は待機」が良い場合は、ここの移動命令をコメントアウトしても良いが、
           % vec2が更新されていないと次ステップでも動かない可能性があるため、ここで更新しておくのが安全。
           nexttarget = Param.X(1,:) + Param.velocity .* vec2;
           sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
          

                

            %     % CoppeliaSimに移動命令を送信（これをしないと動き出さない）
            %    sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
               disp('デッドロック発生');
            % 
            % end
            
        end
    % stepgoalとXの距離が1.0未満，次のstepgoalが存在し，stepgoalとgoalが一致していないとき，位置を更新せず，g=g+1とする
    elseif (norm(Param.stepgoal(1,:) - Param.X(1,:)) < 1) && (g < size(truepath, 1) && any(Param.stepgoal~=Param.goal))
        nexttarget = Param.X(1,:) + Param.velocity .* vec2;
        sim.simxSetObjectPosition(clientID, target, -1, [nexttarget(1), nexttarget(2), nexttarget(3)], sim.simx_opmode_oneshot);
        g = g + 1;    % g=g+1とすることで，一時目標点を次の点に更新する
         disp('次のステップに移行');
    end

     %% 描画
   xlim([-8 8]); ylim([-9 9]); zlim([0 5]);
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
    % stepgoalの描画
    for o = g+1:1:size(truepath,1) - 1
        drawstepgoal(o,:) = scatter3(truepath(o,1),truepath(o,2),truepath(o,3), 50, 'o', 'MarkerEdgeColor',Param.Color(1));
    end
    % 次のstepgoalの描画（次に目指すstepgoalは色を塗りつぶす）
    if any(Param.stepgoal~=Param.goal)
        drawnextstepgoal = scatter3(truepath(g,1),truepath(g,2),truepath(g,3), 50, 'o', 'MarkerEdgeColor', Param.Color(1), 'MarkerFaceColor',Param.Color(1));
    end

    % 軸範囲を再設定
   xlim([-8 8]); ylim([-9 9]); zlim([0 5]);

    drawnow

    Frame(animenum)=getframe(1);
    animenum=animenum+1;   

    % ゴールに到達したらループを抜ける
    if norm(Param.goal(1,:) - Param.X(1,:)) < 0.3
        break;
    end

    % ステップ描画用
    if(rem(t-1,0.3)==0)
        %savefig(['step',num2str(t-1),'.fig']);
    end

   %  % -----------------------------------------------------------
   %  % ★追加：1.5秒ごとに画像を保存する処理
   %  % -----------------------------------------------------------
   %  % 保存先フォルダの設定
   %  save_dir = 'C:\Users\Murakami\村上\先輩いじる\プログラム\完全な未知\figure';
   % 
   %  % フォルダがなければ作成
   %  if ~exist(save_dir, 'dir')
   %      mkdir(save_dir);
   %  end
   % 
   %  % 現在のシミュレーション時刻
   %  current_time = t - 1;
   % 
   %  % 1.5秒間隔かどうかを判定（誤差を考慮）
   %  % ※ t=0 も保存したい場合は「 || current_time == 0」を追加してください
   %  if abs(rem(current_time, 1.5)) < Param.delta / 2
   %      % ファイル名を生成 (例: sim_t_01.5.png)
   %      % %04.1f は「全体4桁・小数1桁」でゼロ埋めする指定です（整列用）
   %      fname = sprintf('sim_t_%04.1fv2.png', current_time);
   %      full_path = fullfile(save_dir, fname);
   % 
   %      % 画像を保存
   %      saveas(gcf, full_path);
   % 
   %      % (オプション) 余白をカットして高画質で保存したい場合は以下を使用
   %      % exportgraphics(gcf, full_path, 'Resolution', 300);
   % 
   %      disp(['Image saved: ', full_path]);
   %  end
   % % -----------------------------------------------------------

    % シミュレーションのステップを進める
    sim.simxSynchronousTrigger(clientID);
end
% stop the simulation:
sim.simxStopSimulation(clientID,sim.simx_opmode_blocking);
% 終了処理
sim.simxFinish(clientID);
sim.delete();

% -----------------------------------------------------------
% ★追加：終了時点の画像を保存
% -----------------------------------------------------------
% 保存先フォルダの設定
save_dir = 'C:\Users\Murakami\村上\先輩いじる\プログラム\完全な未知\figure';

% フォルダがなければ作成
if ~exist(save_dir, 'dir')
    mkdir(save_dir);
end

% 最終状態の画像ファイル名 (例: sim_final.png)
final_fname = 'sim_finalv2.png';
full_final_path = fullfile(save_dir, final_fname);

% 画像を保存 (PNG形式)
saveas(gcf, full_final_path);
disp(['Final image saved: ', full_final_path]);

%% アニメーション書き出し用の処理
Frate = 20;
v = VideoWriter('C:\Users\Murakami\村上\先輩いじる\プログラム\完全な未知\figure');
v.FrameRate = Frate; % Framerate
open(v);
writeVideo(v,Frame);
close(v);
savefig('stepfinal.fig');
%%