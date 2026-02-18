
function of = ObstacleFree(node1, node2, obstacle)
% 三次元空間における障害物の衝突判定を行う関数
% 障害物に衝突している場合false(0)を，衝突していない場合true(1)を返す
% node1, node2 : 頂点座標
% obstacle : 障害物の情報（配列）
% of : 障害物衝突判定，衝突していない場合(true)と衝突している場合(false)の二値
of = true; % 初期化: 障害物がないと仮定
numPoints = 100; % 線分を分割する点の数

% node1とnode2の間の線分を生成
t = linspace(0, 1, numPoints);
lineSegment = (1 - t') * node1 + t' * node2;

% 障害物の各直方体に対する処理
for oc = 1:size(obstacle, 1)
    % 障害物の各辺の最小値と最大値を取得
    xmin = obstacle(oc, 1);
    xmax = obstacle(oc, 2);
    ymin = obstacle(oc, 3);
    ymax = obstacle(oc, 4);
    zmin = obstacle(oc, 5);
    zmax = obstacle(oc, 6);

    % 線分の各点が直方体内にあるかチェック
    inObstacle = lineSegment(:, 1) >= xmin & lineSegment(:, 1) <= xmax & ...
                 lineSegment(:, 2) >= ymin & lineSegment(:, 2) <= ymax & ...
                 lineSegment(:, 3) >= zmin & lineSegment(:, 3) <= zmax;

    if any(inObstacle)
        of = false; % 障害物に衝突している場合
        return;
    end
end
end