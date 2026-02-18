function mindist = voronoi_GetMinDist(Param)
% Paramを受け取って最も近い障害物との距離を返す
    dist = Param.X(1,:) - Param.newnearP;
    dist = sqrt(sum(dist.^2,2));

    mindist = min(dist);
end