%ボロノイ境界からサブゴールを決定する
function [V, P] = SubgoalGenerator(V, P)

P.Way_Point = repelem(inf, P.dronenum, 2);

finish = 1;
for n = 1:P.dronenum
    
    if(norm(P.fakegoal(n,:) - P.position(n,:)) <= 0.1)
        P.fakegoal(n,1) = P.target(n,1);
        P.fakegoal(n,2) = P.target(n,2);
    end

    finish2 = finish + V.linenumber(n) - 1;
    MarginPoints = V.MarginPoints(finish : finish2, :);
    finish = finish2 + 1;

    %内外判定
    in = inpolygon(P.fakegoal(n,1), P.fakegoal(n,2), MarginPoints(:,1), MarginPoints(:,2));

    %目標点がBVCの内側にある時
    if(in == 1)
        P.Way_Point(n,:) = P.fakegoal(n,:);   %サブゴール=目標点
    %目標点がBVCの外側にある時
    else
        L = circshift(MarginPoints, 1) - MarginPoints;   %ボロノイ境界(バッファあり)のベクトル
        M = P.fakegoal(n,:) - MarginPoints;                %ボロノイ頂点1(バッファあり)から目標点へのベクトル
        N = circshift(M, 1);                             %ボロノイ頂点2(バッファあり)から目標点へのベクトル
        C = dot(L', M')' ./ vecnorm(L')' .^ 2;           
        %目標点と線分Lとの交点の距離比率(目標点から垂線を下ろし，その垂線が線分P上にあるか判定(S<0,1<Sのとき，線分上に目標との最短距離となる垂線はない)
        %(詳しくは本仲先生からSlackに送られた画像を参照する)

       waypoint_dist = vecnorm(M')' .* (C < 0);                   %Cが0以下の時，Mの外側に目標点があるので，その時の目標点からの距離を保存
       waypoint_dist = waypoint_dist + vecnorm(N')' .* (C > 1);   %Cが1以上の時，Nの外側にあるので，その時の目標点からの距離を保存
       
       UnitL = (L' ./ vecnorm(L'))';                             %Lの単位ベクトルを計算
       leng2 = dot(UnitL', M')';                                 %[目標点からおろした垂線との交点]とボロノイ頂点１(バッファあり)との距離
       parp = MarginPoints + UnitL .* leng2;                     %サブゴールの座標('目標点からおろした垂線との交点の座標')
       vecL = parp - MarginPoints;                               %ボロノイ頂点1から'目標点からおろした垂線との交点の座標'までのベクトル
       distwp = sqrt(vecnorm(M') .^ 2 - vecnorm(vecL') .^ 2)';   %'目標点からおろした垂線との交点の座標'と目標点の距離

       waypoint_dist = waypoint_dist + distwp .* ((0 < C)&(C < 1));   %0<S<1のとき，目標点からボロノイ境界におろした垂線との交点をサブゴールとする
       waypoint_dist = waypoint_dist .* (waypoint_dist == min(waypoint_dist));   %waypoint_distが最小になるとき1とする

       cor_dist = find(waypoint_dist > 0);   %waypoint_distが1であるとき

       if(C(cor_dist(1)) < 0)
           P.Way_Point(n,:) = MarginPoints(cor_dist(1),:);
       elseif(C(cor_dist(1)) > 1)
           MarginPoints = circshift(MarginPoints,1);
           P.Way_Point(n,:) = MarginPoints(cor_dist(1),:);
       else
           P.Way_Point(n,:) = parp(cor_dist(1),:);
       end
    end

    %% デッドロック回避アルゴリズムを適用する場合
    % 機体を1台に設定し，静止障害物を回避できないときに使用
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % V.difference(n,1:2) = [sqrt((P.Way_Point(n,1) - P.position(n,1))^2), sqrt((P.Way_Point(n,2) - P.position(n,2))^2)];
    % if (V.difference(n,1) < 0.08 && V.difference(n,2) < 0.08) && (norm(P.fakegoal(n,:) - P.position(n,:)) > 0.1)
    %     %デッドロックが起こったとき，現在位置から目標位置へのベクトルを90度回転させた場所に一時目標点を生成
    %     newvectorX = (P.fakegoal(n,2) - P.position(n,2));% .* P.subgain;    %90度回転させたx成分
    %     newvectorY = (-P.fakegoal(n,1) + P.position(n,1));% .* P.subgain;   %90度回転させたy成分
    %     XY=[newvectorX;newvectorY]/norm([newvectorX;newvectorY]).* P.subgain;
    %     newvectorX=XY(1,1);
    %     newvectorY=XY(2,1);
    %     P.fakegoal(n,1) = P.position(n,1) + newvectorX;                   %現在位置にベクトルの成分を足す
    %     P.fakegoal(n,2) = P.position(n,2) + newvectorY;
    %     if P.fakegoal(n,:) == P.object
    %         P.fakegoal(n,1) = P.fakegoal(n,1) + newvectorX .* 0.2;
    %         P.fakegoal(n,2) = P.fakegoal(n,2) + newvectorY .* 0.2;
    %     end
    % 
    % 
    %     e1 = hypot(P.Way_Point(n,1) - P.position(n,1), P.Way_Point(n,2) - P.position(n,2));    %サブゴールと現在位置の距離
    %     nextX1 = P.position(n,1) + ((P.Way_Point(n,1) - P.position(n,1))/e1) / P.gain;         %次に進む点のX座標
    %     nextY1 = P.position(n,2) + ((P.Way_Point(n,2) - P.position(n,2))/e1) / P.gain;         %次に進む点のY座標
    %     [in,on] = inpolygon(nextX1, nextY1, MarginPoints(:,1), MarginPoints(:,2));             %次に進む点がマージンの内側にあるか判定
    %     if in == 1 || on == 1
    %         V.newX(n) = (P.Way_Point(n,1) - P.position(n,1)) / e1;   %現在位置からサブゴールまでの単位ベクトル
    %         V.newY(n) = (P.Way_Point(n,2) - P.position(n,2)) / e1;
    %     else   %外側にある時(このまま速度のまま進むと，マージンの外側に移動してしまうから，速度を落としてサブゴールの外側に行かないように調節する)
    %         V.newX(n) = (P.Way_Point(n,1) - P.position(n,1)) / P.gain;   %現在位置からサブゴールまでの，長さが(サブゴール-現在位置)/P.gainのベクトル
    %         V.newY(n) = (P.Way_Point(n,2) - P.position(n,1)) / P.gain;
    %     end
    % 
    % %デッドロックでないとき
    % else 
    %     e2 = hypot(P.Way_Point(n,1) - P.position(n,1), P.Way_Point(n,2) - P.position(n,2));    %サブゴールと現在位置の距離
    %     nextX2 = P.position(n,1) + ((P.Way_Point(n,1) - P.position(n,1))/e2) / P.gain;         %次に進む点のX座標
    %     nextY2 = P.position(n,2) + ((P.Way_Point(n,2) - P.position(n,2))/e2) / P.gain;         %次に進む点のY座標
    %     [in,on] = inpolygon(nextX2, nextY2, MarginPoints(:,1), MarginPoints(:,2));             %次に進む点がマージンの内側にあるか判定
    %     if in == 1 || on == 1   %内側にある時
    %         V.newX(n) = (P.Way_Point(n,1) - P.position(n,1)) / e2;   %現在位置からサブゴールまでの単位ベクトル
    %         V.newY(n) = (P.Way_Point(n,2) - P.position(n,2)) / e2;
    %     else   %外側にある時(このまま速度のまま進むと，マージンの外側に移動してしまうから，速度を落としてサブゴールの外側に行かないように調節する)
    %         V.newX(n) = (P.Way_Point(n,1) - P.position(n,1)) / P.gain;   %現在位置からサブゴールまでの，長さが(サブゴール-現在位置)/P.gainのベクトル
    %         V.newY(n) = (P.Way_Point(n,2) - P.position(n,1)) / P.gain;
    %     end
    % end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %% デッドロック回避を適用しないとき
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    e2 = hypot(P.Way_Point(n,1) - P.position(n,1), P.Way_Point(n,2) - P.position(n,2));    %サブゴールと現在位置の距離
    nextX2 = P.position(n,1) + ((P.Way_Point(n,1) - P.position(n,1))/e2) / P.gain;         %次に進む点のX座標
    nextY2 = P.position(n,2) + ((P.Way_Point(n,2) - P.position(n,2))/e2) / P.gain;         %次に進む点のY座標
    [in,on] = inpolygon(nextX2, nextY2, MarginPoints(:,1), MarginPoints(:,2));             %次に進む点がマージンの内側にあるか判定
    if in == 1 || on == 1   %内側にある時
        V.newX(n) = (P.Way_Point(n,1) - P.position(n,1)) / e2;   %現在位置からサブゴールまでの単位ベクトル
        V.newY(n) = (P.Way_Point(n,2) - P.position(n,2)) / e2;
    else   %外側にある時(このまま速度のまま進むと，マージンの外側に移動してしまうから，速度を落としてサブゴールの外側に行かないように調節する)
        V.newX(n) = (P.Way_Point(n,1) - P.position(n,1)) / P.gain;   %現在位置からサブゴールまでの，長さが(サブゴール-現在位置)/P.gainのベクトル
        V.newY(n) = (P.Way_Point(n,2) - P.position(n,1)) / P.gain;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end
