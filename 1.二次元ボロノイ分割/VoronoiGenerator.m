%現在位置からボロノイ境界を生成
function V = VoronoiGenerator(Parameter)

Parameter.P = [Parameter.position ; Parameter.object];
%Parameter.P = [Parameter.position];

%外に点を追加して無限遠を回避
minx = min(Parameter.P(:,1)) - 0.5;
miny = min(Parameter.P(:,2)) - 0.5;
maxx = max(Parameter.P(:,1)) + 0.5;
maxy = max(Parameter.P(:,2)) + 0.5;

rangex = maxx - minx;
rangey = maxy - miny;

range = rangex + rangey;

averagex = (minx + maxx)/2;
averagey = (miny + maxy)/2;

Parameter.newP=[Parameter.P ; minx - range, averagey ; maxx + range, averagey ; ...
    averagex, miny - range ; averagex, maxy + range];   %ドローンの位置の座標に無限遠を回避する4点を追加した行列を保存

[V.X, V.Y] = voronoi(Parameter.newP(:,1), Parameter.newP(:,2));   %各ボロノイ境界のXY座標を取得

[V.Vpoints, V.rpoints] = voronoiDiagram(delaunayTriangulation(Parameter.newP));   %V.Vpointsでボロノイ頂点の座標，V.rpointsで母点に対応するV.Vpointsの行の番号を保存

for a = 1:length(V.Vpoints)
    check = V.Vpoints(a, :) == V.Vpoints;   %r番目の座標と一致しているものを探す("=="を用いて，一致している座標は1，一致していない座標は0にする)
    check = check(:, 1) .* check(:, 2);     %x座標もｙ座標も一致している場合(x座標とy座標がともに一致していると1となる)

    if sum(check) > 1                             % 一致する座標が2つ以上ある場合
            check = find(check==1);             % 一致している座標を見つける
            for k = 1:length(V.r)             % V.r{k}（k番目の点を囲むボロノイ頂点）の中に同じ頂点（check(1)とcheck(2)）を含んでいるときの処理
                % セルが同じ値を含んでいるとき(findで一致する座標を見つけ，k番目の母点を囲むボロノイ頂点にfindで見つけた頂点を含んでいるとき)
                if sum(V.r{k} == check(1)) * sum(V.r{k} == check(2)) > 0    % 同じ頂点を含んでいたら(check(1)はfindで見つけた一つ目の頂点，check(2)は二つ目の頂点)
                    %この場合，check(1)，check(2)の二つしかないため，k番目の母点に重複する頂点が三つ以上ある場合を想定していない
                    tmp = V.r{k};                          % tmpにk番目の点を囲む頂点群を一時保存
                    tmp(tmp==check(2))=[];   % check(2)と一致する頂点を削除する（重複しているので）
                    V.r{k} = tmp;                          % V.r{k} に重複した頂点を削除済みの頂点群を戻す
                end
            end        
    end
end

end
