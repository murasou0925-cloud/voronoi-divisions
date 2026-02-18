function is_collision = is_in_obstacle(P,O)
%IS_IN_OBSTACLE この関数の概要をここに記述
%   詳細説明をここに記述

margin=0.3;

is_collision = (P(1) >= O(1)-margin) & (P(1) <= O(2)+margin) & ... % X軸方向の判定
                   (P(2) >= O(3)-margin) & (P(2) <= O(4)+margin) & ... % Y軸方向の判定
                   (P(3) >= O(5)-margin) & (P(3) <= O(6)+margin);       % Z軸方向の判定
end