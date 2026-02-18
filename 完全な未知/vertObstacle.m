function [vert, fac] = vertObstacle(obstacle)
% 障害物（直方体）を三次元座標に描画するために，座標を変換する関数
    vert = [obstacle(1),obstacle(3),obstacle(5);
            obstacle(2),obstacle(3),obstacle(5);
            obstacle(2),obstacle(4),obstacle(5);
            obstacle(1),obstacle(4),obstacle(5);
            obstacle(1),obstacle(3),obstacle(6);
            obstacle(2),obstacle(3),obstacle(6);
            obstacle(2),obstacle(4),obstacle(6);
            obstacle(1),obstacle(4),obstacle(6)];
    fac = [1,2,6,5;2,3,7,6;3,4,8,7;4,1,5,8;1,2,3,4;5,6,7,8];
end