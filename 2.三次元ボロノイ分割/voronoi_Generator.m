function [Param, alshp, secpoint, dist, vec2] = voronoi_Generator(Param)

dt = delaunayTriangulation(double(Param.X));
[Param.V, Param.R] = voronoiDiagram(dt);

Param.vrtxs = Param.V(Param.R{1}, :);  % 現在位置を内包するボロノイ頂点をvrtxsに保存
Param.vrtxs = Param.vrtxs(all(isfinite(Param.vrtxs), 2), :);  % vrtxs内にinfがあれば削除
alshp = alphaShape(Param.vrtxs, 1000);  % ボロノイ頂点を全て含む凸形状をalphaShapeで生成
secpoint = voronoi_SubGoal(Param,1);  % ↑の点をサブゴールとする．

dist = norm(Param.X(1, :) - secpoint);
vec1 = (Param.X(1, :) - secpoint) / dist;  % secpoint→現在位置の単位ベクトル
vec2 = (secpoint - Param.X(1, :)) / dist;  % 現在位置→secpointの単位ベクトル

%% 目標位置の内外判定
% 目標位置がボロノイ領域内にある場合，バッファを設けず，ボロノイ領域外にある場合，バッファを設ける
% ボロノイ領域を楕円に近似し，楕円内外を判定する（この手法は不完全すぎるから改良すべき）

% Param.vrtxs 内の中心 (h, k) を計算（最小値・最大値の平均）
h = (min(Param.vrtxs(:, 1)) + max(Param.vrtxs(:, 1))) / 2;
k = (min(Param.vrtxs(:, 2)) + max(Param.vrtxs(:, 2))) / 2;

% 楕円の半径 a と b（最大・最小の x 値, y 値に基づく）
a = (max(Param.vrtxs(:, 1)) - min(Param.vrtxs(:, 1))) / 2;
b = (max(Param.vrtxs(:, 2)) - min(Param.vrtxs(:, 2))) / 2;

% 判定したい点 (x, y)
point = [Param.goal(1), Param.goal(2)];  % x, y は判定したい点の座標

% 楕円の方程式を使って点が内部か判定
ellipse_eq = ((point(1) - h)^2 / a^2) + ((point(2) - k)^2 / b^2);

% 楕円内にあるかどうかを判定
if ellipse_eq <= 1
    %disp('点は楕円内にあります。');
else
    secpoint = secpoint + vec1 * Param.buffer;
    dist = norm(Param.X(1, :) - secpoint);
end
end