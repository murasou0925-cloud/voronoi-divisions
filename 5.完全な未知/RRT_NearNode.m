function znear = RRT_NearNode(nodes, zncoord, nc, BRC, StateDim, EPS, i)
% 各変数の内容
% nodes     : ノード群(struct)
% N         : ノード群(table)
% zncoord   : 付近のノードを探索する際の，中心になるノードの座標(double)
% znear     : zncoordを中心とした球内に存在するノード群(struct)
% nc        : 現在のノード番号(znewのノード番号)
% r         : znearを導出する際の探索半径
% BRC       : 探索半径の定数
% StateDim  : 状態空間の次元数
% EPS       : 辺の最大長
% i         : ノード数

    r = BRC * power(log(i) / i, 1 / StateDim);
    r = min(r, EPS);
    znew = repmat(zncoord, nc-1, 1);
    Distance = (znew-nodes{1:nc-1,1}).^2;
    Distance = (Distance(:,1) + Distance(:,2) + Distance(:,3)).^(1/2);
    nearid = find(Distance <= r);
    if ~isempty(nearid)
        index = ismember(nodes{:,4}, nearid);
        znear = nodes(index,:);
    else
        znear = [];
    end
end