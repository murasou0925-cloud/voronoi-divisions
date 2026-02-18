clear
clc
close all

% --- 保存したい時間のリスト設定 ---
save_target_times = [0, 1.6, 3.6, 10]; 
% -------------------------------

% 描画設定
Parameter = Init(4);

figure('Position',[10 50 800 600]); hold on; box on;
%xlim([min(reshape([Parameter.position ; Parameter.target], 1, [])) - 1, max(reshape([Parameter.position ; Parameter.target], 1, [])) + 1]);
%ylim([min(reshape([Parameter.position ; Parameter.target], 1, [])) - 1, max(reshape([Parameter.position ; Parameter.target], 1, [])) + 1]);
xlim([0,8]);
ylim([0,6]);
set(gca, 'Fontsize', 24, 'FontName', 'Times New Roman');
xlabel('X-position');
ylabel('Y-position');
colornumber = ['m','g','b','y','r'];
animenumber = 1;
% 障害物の描画
plot(Parameter.object(:,1),Parameter.object(:,2),'.k','MarkerSize',50);

prevposition = Parameter.position;

Parameter.fakegoal = Parameter.target;

t = 1;
flag  = true;
while(flag)
    if(t > 1)
        delete(mplot);
        delete(WayPointplot);
        delete(vd);
        delete(timedraw);
        delete(nowposition);
        if exist("fakegoal")
            delete(fakegoal);
        end
    end

    % 現在の表示時間を計算
    current_disp_time = t - 1;

    timedraw = annotation('textbox', Parameter.textboxdimention, 'String', ['\it t = ' num2str(t - 1)], 'EdgeColor', 'k', 'BackgroundColor', 'w', 'FontName', 'Times New Roman', 'FontSize', 22);

    Voronoi_Data = VoronoiGenerator(Parameter);
    vd = plot(Voronoi_Data.X, Voronoi_Data.Y, '-k', 'Linewidth', 2);
    Voronoi_Data = MarginGenerator(Voronoi_Data, Parameter);
    [Voronoi_Data, Parameter] = SubgoalGenerator(Voronoi_Data, Parameter);

    finish = 1;
    for n = 1:Parameter.dronenum
        traceline(n) = line([prevposition(n,1) Parameter.position(n,1)], [prevposition(n,2) Parameter.position(n,2)], 'Color', colornumber(rem(n,5) + 1), 'LineWidth', 2);

        finish2 = finish + Voronoi_Data.linenumber(n) - 1;
        dpoint = finish : finish2;
        mplot(n) = patch(Voronoi_Data.MarginPoints(dpoint,1), Voronoi_Data.MarginPoints(dpoint,2), colornumber(rem(n,5) + 1), 'Facealpha', .3);
        finish = finish2 + 1;

        scatter(Parameter.position(n,1), Parameter.position(n,2), 10, colornumber(rem(n,5) + 1), 'filled', 'MarkerEdgeColor', colornumber(rem(n,5) + 1));
        nowposition(n) = scatter(Parameter.position(n,1), Parameter.position(n,2), 300, colornumber(rem(n,5) + 1), 'filled', '^', 'MarkerEdgeColor','k');
        scatter(Parameter.target(n,1), Parameter.target(n,2), 500, colornumber(rem(n,5) + 1), "filled", 'p', 'MarkerEdgeColor', 'k');
        WayPointplot(n) = scatter(Parameter.Way_Point(n,1), Parameter.Way_Point(n,2), 100, colornumber(rem(n,5) + 1), 'filled', 'MarkerEdgeColor', 'k');
        if(Parameter.fakegoal~=Parameter.target)
            fakegoal(n) = scatter(Parameter.fakegoal(n,1), Parameter.fakegoal(n,2), 100, colornumber(rem(n,5) + 1), 'filled', 'd', 'MarkerEdgeColor', 'k');
        end
        prevposition(n,:) = Parameter.position(n,:);
    end

    %position_log(1 : Parameter.dronenum, animenumber * 2 - 1 : animenumber * 2) = Parameter.position;


    % 速度に応じてマージンの幅を調整

for n = 1:Parameter.dronenum
    % 現在のドローンの速度を計算
    % hypotは2点間の距離を計算します
    current_velocity = hypot(Parameter.position(n,1) - prevposition(n,1), Parameter.position(n,2) - prevposition(n,2));

    % 速度が速いほどマージン幅を大きく、遅いほど小さくする
    % ベースとなるマージン幅（P.width_base）に速度を掛け合わせる
    % P.width_baseはInit.mで新たに定義すると良い
    P.width(n) = Parameter.width_base * current_velocity;

    % 最小マージン幅を設定して急激な変化を防ぐ
    P.width(n) = max(P.width(n), 0.1); 
end

    %機体位置の更新
    for n = 1:Parameter.dronenum
        goaldis = hypot(Parameter.position(n,1) - Parameter.target(n,1), Parameter.position(n,2) - Parameter.target(n,2));
        %goaldis2 = goaldis > 1 / Parameter.gain;
         %距離に応じてゲインを調整
        % 距離が遠いほどgainを小さく（速度を速く）、近いほどgainを大きく（速度を遅く）
        dynamic_gain = max(0.5, 15 / (goaldis + 0.1)); % 0除算を避けるため0.1を足し、最大速度を制限
    
    % サブゴールに向かう方向ベクトル（SubgoalGeneratorで計算されたもの）
    direction_x = Voronoi_Data.newX(n);
    direction_y = Voronoi_Data.newY(n);

        Parameter.position(n,1) = Parameter.position(n,1) + Voronoi_Data.newX(n) / dynamic_gain
        Parameter.position(n,2) = Parameter.position(n,2) + Voronoi_Data.newY(n) / dynamic_gain
    end

    drawnow

    % --- ここに追加: 特定の時間で画像を保存する処理 ---
    % 浮動小数点の誤差を考慮して差分が非常に小さい時に保存します
    if any(abs(save_target_times - current_disp_time) < 0.0001)
        % ファイル名に時間を付けて保存 (例: Voronoi_t1.6.png)
        filename_png = sprintf('Voronoi_t%g.png', current_disp_time);
        saveas(gcf, filename_png); 
        
        % 必要であればfigファイルも保存
        % filename_fig = sprintf('Voronoi_t%g.fig', current_disp_time);
        % savefig(filename_fig);
        
        disp(['Saved image at t = ' num2str(current_disp_time)]);
    end
    % --------------------------------------------------
    frame(animenumber)  = getframe(1);
    animenumber = animenumber + 1;

    if (sum(vecnorm(Parameter.target' - Parameter.position') > Parameter.threshold) == 0)
        flag = false;
    end

    if(rem(t,0.4)==0)
        %savefig(['step',num2str(t-1),'.fig']);
    end

t = t + Parameter.interval;

% 安全策: t=10を超えたらループを抜ける場合（もしシミュレーションが終わらなければ）
    if t - 1 > 15 
        flag = false; 
    end

end

%動画保存設定
frate = 5;
Video = VideoWriter('C:\Users\Murakami\村上\先輩いじる\プログラム\1.二次元ボロノイ分割');
Video.FrameRate = frate;
open(Video);
writeVideo(Video,frame);
close(Video);

savefig('Voronoi_stepfinal.fig');