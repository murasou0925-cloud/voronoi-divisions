function [zmin, cmin] = RRT_ChooseParent(nodes, znear, znearest, zncoord, obstacle)
% "Incremental Sampling-based Algorithms for Optimal Motion Planning" : Algorithm 4 : Line 8-12
% znewの親ノードをコストが小さくなる場合に変更する関数
% 各変数の内容
% zncoord   : znewの座標(3次元)
% znear     : zncoordの周囲のノード群
% znearest  : znewに最も近いノード
% obstacle  : 障害物の座標(6次元:xmin,xmax,ymin,ymax,zmin,zmax)
% zmin      : 経由した際に最もコスト値が小さくなるznewの親となるノードの，nodesにおけるノード番号
% cmin      : zminを経由した際の，nodes側の根からのznewの累積経路長

    zmin = znearest{1,4};
    cmin = RRT_Cost(nodes,zmin) + norm(zncoord-znearest{1,1});
    if ~isempty(znear)
        for i = 1:1:height(znear)
            tentcost = RRT_Cost(nodes,znear{i,4}) + norm(znear{i,1}-zncoord);
            if RRT_ObstacleFree(znear{i,1},zncoord,obstacle) && ... 
               tentcost < cmin
                zmin = znear{i,4};
                cmin = tentcost;
            end
        end
    end
end