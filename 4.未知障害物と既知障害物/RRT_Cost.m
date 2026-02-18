function c = RRT_Cost(nodes,nodenum)
% 対象のノードの根からの累積経路長を返す関数
% 各変数の内容
% nodes     : ノード群
% nodenum   : 累積経路長を導出したいノード番号
% c         : 根からの累積経路長
    c = 0;
    child = nodes(nodenum,:);
    while true
        if child{1,4} == 1
            break;
        else
            parent = nodes(child{1,2},:);
            c = c + norm(parent{1,1}-child{1,1});
            child = parent;
        end
    end
end