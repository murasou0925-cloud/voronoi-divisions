function [State, nodes, nc, tnc, zncoord] = RRT_ExtendStar(nodes, nc, tnc, z, BRC, StateDim, EPS, obstacle)
% 元論文 : "RRT*-Connect : Faster, Asymptotically Optimal Motion Planning" 
%
% nodes     : ノード群
% nc        : ノード群に含まれるノード数
% tnc       : 二つのノード群に含まれるノードの総数
% z         : サンプリングされた座標（[x,y,z]）
% BRC       : Near関数の定数
% EPS       : 辺の最大となる長さ
% StateDim  : 空間の次元数
% obstacle  : 障害物の頂点情報（[xmin,xmax,ymin,ymax,zmin,zmax]）
% State     : 最終的な拡張の状態（REACHED, ADVANCED, TRAPPEDのいずれかの文字）
% zncoord   : znewの座標（[x,y,z]）

[val, znearest] = RRT_NearestNode(nodes, z, nc);
zncoord = RRT_steer(z, znearest{1,1}, val, EPS);
if RRT_ObstacleFree(zncoord, znearest{1,1}, obstacle)
    tnc = tnc + 1;
    nc = nc + 1;
    znear = RRT_NearNode(nodes,zncoord,nc,BRC,StateDim,EPS,tnc);
    [zmin,cmin] = RRT_ChooseParent(nodes,znear,znearest,zncoord,obstacle);
    nodes(nc,:) = {zncoord,zmin,{[]},nc,inf};
    nodes{zmin,3} = {[cell2mat(nodes{zmin,3}), nc]};
    nodes = RRT_Rewire(nodes,nc,cmin,znear,obstacle);
    if isequal(z, zncoord)
        State = "REACHED";
    else
        State = "ADVANCED";
    end
else
    State = "TRAPPED";
end
end