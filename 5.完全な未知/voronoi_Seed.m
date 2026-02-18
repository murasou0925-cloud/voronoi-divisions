function [Param, ranges] = voronoi_Seed(Param, ranges, QuadAng, QuadPos, animenum)

% ranges        : センサデータ
% QuadPos       : 機体の現在位置

if ranges ~= 0
    ranges = reshape(ranges, 3, length(ranges)/3);

    % 回転行列
    roll  = QuadAng(1);
    pitch = QuadAng(2);
    yaw   = QuadAng(3);

    RzRyRx = [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll);
              sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll);
              -sin(pitch),         cos(pitch)*sin(roll),                             cos(pitch)*cos(roll)];

    ranges = RzRyRx * ranges;

    % 機体位置を原点とした座標で送られてくるので，機体位置を足す
    x = -ranges(3,:) + QuadPos(1);
    y = ranges(2,:) + QuadPos(2);
    z = ranges(1,:) + QuadPos(3);

    %% 点群のクラスタリング
    ptCloud = pointCloud([x(:), y(:), z(:)]);
    [labels, ~] = pcsegdist(ptCloud, 0.5);
    % 'labels' に各点が属するクラスタのラベルが入っている
    uniqueLabels = unique(labels); % 一意のラベルを取得

    Param.newnearP = [];
    % 障害物ごとに点を間引く
    for i = 1:length(uniqueLabels)    % length(uniqueLabels)=障害物の数
        label = uniqueLabels(i);  % 現在のラベル
        idx = find(labels == label);  % 現在のラベルに対応するインデックスを取得
        x_label = x(idx)';
        y_label = y(idx)';
        z_label = z(idx)';
        Param.nearP = [x_label y_label z_label];   % i番目の障害物データ

        % 障害物を構成する点群の数が300より多ければ間引く
        if length(Param.nearP) > 300
            for j = 1:length(Param.nearP)
                if rem(j,1) == 0   % rem(j,〇)←〇の値を変えることで間引き率を操作
                    Param.newnearP = [Param.newnearP;Param.nearP(j,:)];   % 間引き後の障害物データ
                end
            end
        % 機体と障害物の距離が2m以上で，障害物を構成する点群の数が100より多ければ間引く
        elseif length(Param.nearP) > 100 && norm([mean(x_label), mean(y_label), mean(z_label)] - Param.X) > 2
            for j = 1:length(Param.nearP)
                if rem(j,1) == 0
                    Param.newnearP = [Param.newnearP;Param.nearP(j,:)];
                end
            end
        % 間引く必要がない場合，そのままnewnearPに保存
        else
            Param.newnearP = [Param.newnearP;Param.nearP];
        end
    end

    % newnearP内が空であればmindistにinfを保存
    if ~isempty(Param.newnearP)
        Param.mindist(animenum) = voronoi_GetMinDist(Param);
    else
        Param.mindist(animenum) = inf;
    end

    % nearPに母点と思われる位置座標を保存（自機の位置を含む）
    Param.X = [Param.X; Param.newnearP];
    maxX = Param.X + 3;
    minX = Param.X - 3;
    aveX = (maxX + minX) / 2;
else
    Param.mindist(animenum) = inf;  % 障害物を検知しないとき，mindistにinfを保存

    maxX = Param.X + 3;
    minX = Param.X - 3;
    aveX = (maxX + minX) / 2;
    ranges = [];
end

Param.O_num = length(Param.X);

Param.X = [Param.X; zeros(26, 3)];


%% 対象の点に無限遠が発生しないようにするための処理(点群の外に母点を追加)
Param.X(Param.O_num+1:Param.O_num+6,:) = ...
              [maxX(1), aveX(2), aveX(3);...
               minX(1), aveX(2), aveX(3);...
               aveX(1), maxX(2), aveX(3);...
               aveX(1), minX(2), aveX(3);...
               aveX(1), aveX(2), maxX(3);...
               aveX(1), aveX(2), minX(3)];

Param.X(Param.O_num+7:Param.O_num+14,:) = ...
               [maxX(1), maxX(2), maxX(3);...
                minX(1), maxX(2), maxX(3);...
                maxX(1), minX(2), maxX(3);...
                maxX(1), maxX(2), minX(3);...
                minX(1), minX(2), maxX(3);...
                minX(1), maxX(2), minX(3);...
                maxX(1), minX(2), minX(3);...
                minX(1), minX(2), minX(3)];

Param.X(Param.O_num+15:Param.O_num+26,:) = ...
                [maxX(1), maxX(2), aveX(3);...
                 minX(1), maxX(2), aveX(3);...
                 minX(1), minX(2), aveX(3);...
                 maxX(1), minX(2), aveX(3);...
                 maxX(1), aveX(2), maxX(3);...
                 minX(1), aveX(2), maxX(3);...
                 minX(1), aveX(2), minX(3);...
                 maxX(1), aveX(2), minX(3);...
                 aveX(1), maxX(2), maxX(3);...
                 aveX(1), minX(2), maxX(3);...
                 aveX(1), minX(2), minX(3);...
                 aveX(1), maxX(2), minX(3)];
end