function DrawObs()

% Faces     (四角形にするための頂点の配置)
faces = [1 2 4 3;   % #1
         5 6 8 7;   % #2
         1 2 6 5;   % #3
         2 4 8 6;   % #4
         4 3 7 8;   % #5
         3 1 5 7];  % #6

%% Draw PaneRod1(ｚ軸方向の角柱を描画)

% Side lengths
x = 0.1;   %x方向の角柱の太さ
y = 0.1;   %y方向の角柱の太さ
z = 8;    %z方向の角柱の長さ

% Vertices  (障害物の頂点座標)
vertices = [0 0 0;     %1
            x 0 0;     %2
            0 y 0;     %3
            x y 0;     %4
            0 0 z;     %5
            x 0 z;     %6
            0 y z;     %7
            x y z];    %8

px = -2.05:2:3.95;
py = -2.05:2:1.95;

for X = 1:4
    for Y = 1:3
        V = vertices;
        V(:,1) = V(:,1) + px(X);
        V(:,2) = V(:,2) + py(Y);
        h = patch('Faces', faces, 'Vertices', V, 'FaceColor', 'k','EdgeAlpha',.5);
    end
end

%% Draw PaneRod1(ｘ軸方向の角柱を描画)

% Side lengths
x = 8;
y = 0.1;
z = 0.1;

vertices = [0 0 0;     %1
            x 0 0;     %2
            0 y 0;     %3
            x y 0;     %4
            0 0 z;     %5
            x 0 z;     %6
            0 y z;     %7
            x y z];    %8

px = -3;
py = -2.05:2:1.95;
pz = 1.95:2:5.95;

for y = 1:3
    for z = 1:3
        V = vertices;
        V(:,1) = V(:,1) + px;
        V(:,2) = V(:,2) + py(y);
        V(:,3) = V(:,3) + pz(z);
        h = patch('Faces', faces, 'Vertices', V, 'FaceColor', 'k','EdgeAlpha',.5);
    end
end

%% Draw PaneRod2(ｙ軸方向の角柱を描画)

% Side lengths
x = 0.1;
y = 6;
z = 0.1;

vertices = [0 0 0;     %1
            x 0 0;     %2
            0 y 0;     %3
            x y 0;     %4
            0 0 z;     %5
            x 0 z;     %6
            0 y z;     %7
            x y z];    %8

px = -2.05:2:3.95;
py = -3;
pz = 1.95:2:5.95;

for x = 1:4
    for z = 1:3
        V = vertices;
        V(:,1) = V(:,1) + px(x);
        V(:,2) = V(:,2) + py;
        V(:,3) = V(:,3) + pz(z);
        h = patch('Faces', faces, 'Vertices', V, 'FaceColor', 'k','EdgeAlpha',.5);
    end
end

alpha(.2)   %障害物の透過度

end