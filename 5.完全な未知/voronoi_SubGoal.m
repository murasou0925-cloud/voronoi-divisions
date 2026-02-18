function secpoint = voronoi_SubGoal(Param,n)

    alshp = alphaShape(Param.vrtxs,1000);       % ボロノイ頂点を全て含む凸形状をalphaShapeで生成
    spacegrid = -5:0.1:5; % secpointを探索するためのグリッド

    % 現在位置を中心に正方形のグリッドボックスを作成
    [gridx,gridy,gridz] = meshgrid(spacegrid+Param.X(n,1),spacegrid+Param.X(n,2),spacegrid+Param.X(n,3)); 

    secp = inShape(alshp,double(gridx),double(gridy),double(gridz)); % グリッドボックスの点がボロノイ領域の中に入っているか判定
    secnum = find(secp==1);                  % ボロノイ領域内の点を保存
    secp = [gridx(secnum), gridy(secnum), gridz(secnum)];
    %dist = sqrt(sum((repmat([Param.goal(n,1), Param.goal(n,2), Param.goal(n,3)],length(secp),1)-secp).^2,2));
    dist = sqrt(sum((repmat([Param.stepgoal(n,1), Param.stepgoal(n,2), Param.stepgoal(n,3)],length(secp),1)-secp).^2,2));
    minpoint = find(dist==min(dist));        % ゴールから最も近いかつボロノイ領域内にある点を探索

    secpoint = secp(minpoint(1),:);          % ↑の点をサブゴールとする．

end
