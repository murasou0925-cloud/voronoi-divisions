function [val, znearest] = RRT_NearestNode(nodes, zrcoord, nc)
% 各変数の内容
% nodes     : ノード群(struct)
% N         : ノード群(table)
% zrcoord   : ランダムに取られた座標(三次元)
% nc        : 現在のノード番号(znewのノード番号)
% znearest  : zrcoordに最も近いnodes中のノード(struct)

    zr = repmat(zrcoord, nc, 1);
    Distance = (zr-nodes{1:nc,1}).^2;
    Distance = (Distance(:,1) + Distance(:,2) + Distance(:,3)).^(1/2);
    [val, idx] = min(Distance);
    znearest = nodes(idx, :);
end