%ボロノイ境界にマージンをとる
function V = MarginGenerator(V, P)

%マージンありの頂点を保存するための変数を用意
V.MarginPoints = [0 0];   %連結できるように[0 0]を作成
for n = 1:P.dronenum
    V.MarginPoints = [V.MarginPoints ; V.Vpoints(V.rpoints{n},:)];   %V.Vpoints()はn番目の母点を囲むボロノイ頂点の座標の羅列
    V.linenumber(n) = length(V.rpoints{n});
end
V.MarginPoints(1,:) = [];   %1行目の[0 0]を消す

finish = 1 ; finish2 = 1;
for n = 1:P.dronenum   %機体の数だけループ
    mpoints = V.MarginPoints(finish:finish+V.linenumber(n)-1,:);   %必要な頂点をmpointsに保存
    mpoints = polyshape(mpoints);                                  %mpointsをpolyshape形式に変換(これらの頂点を持つ多角形を生成)
    mpoints = polybuffer(mpoints, -P.width);                       %polybufferでマージンをとる(voronoi02.mのP.widthで設定)
    leng = length(mpoints.Vertices);                               %頂点の数を保存   %mpoints.Verticesにpolybufferを形成する座標が保存されている
    V.newMarginPoints(finish2:finish2 + leng - 1,:) = mpoints.Vertices;   %V.newMarginPointsに頂点を一時保存

    %finishとfinish2を更新(polyshapeに変換するとき頂点の数が変わることに伴う処理)
    finish = finish + V.linenumber(n);
    finish2 = finish2 + leng;
    V.linenumber(n) = leng;   %頂点数を修正
end

V.MarginPoints = [];
V.MarginPoints = V.newMarginPoints;   %V.MarginPointsに，V.newMarginPointsに保存している頂点を返す

end
