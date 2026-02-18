function [nodes1, nodes2, nc1, nc2, tnc] = RRT_ConnectStar(nodes1, nodes2, nc1, nc2, tnc, z, BRC, StateDim, EPS, obstacle)
% 元論文 : "RRT*-Connect : Faster, Asymptotically Optimal Motion Planning" 
%
% nodes1中のznewに向かってnodes2から衝突またはznewに到達するまで辺を伸ばし続ける関数
% nodes1, nodes2    : それぞれのノード群
% nc1, nc2          : それぞれのノード群に含まれるノード数
% tnc               : 二つのノード群に含まれるノードの総数
% z                 : 伸ばし続ける目標となるサンプリングされた座標（znew, [x,y,z]）
% BRC               : Near関数の定数
% EPS               : 辺の最大となる長さ
% StateDim          : 空間の次元数
% obstacle          : 障害物の頂点情報（[xmin,xmax,ymin,ymax,zmin,zmax]）
% State             : 最終的な拡張の状態（REACHED, ADVANCED, TRAPPEDのいずれかの文字）
% zncoord           : znewの座標（[x,y,z]）

State = "ADVANCED";
while State == "ADVANCED"
    [State, nodes2, nc2, tnc, ~] = RRT_ExtendStar(nodes2, nc2, tnc, z, BRC, StateDim, EPS, obstacle);
end
if State == "REACHED"
    nodes1{nc1,5} = nc2;
    nodes2{nc2,5} = nc1;
end
end
