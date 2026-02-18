function nodes = RRT_Rewire(nodes, nc, znew_cost, znear, obstacle)
        for i = 1:1:height(znear)
                % znearの現在の累積経路長を導出
                znear_cost = RRT_Cost(nodes,znear{i,4});
                % znewを経由した際の経路長との比較
                if znear{i,4} ~= nodes{nc,2} && ...
                   RRT_ObstacleFree(nodes{nc, 1}, znear{i, 1}, obstacle) && ...
                   znear_cost > znew_cost + norm(nodes{nc,1}-nodes{znear{i,4},1})
                    % znearの現在の親のノードに格納されている子の情報からznearを削除
                    child = cell2mat(nodes{znear{i,2},3});
                    child(child==znear{i,4}) = [];
                    nodes{znear{i,2},3} = {child};
                    % znearの親のidをznewに更新
                    nodes{znear{i,4},2} = nc;
                    % znewの子にznearを追加
                    nodes{nc,3} = {[cell2mat(nodes{nc,3}), znear{i,4}]};
                end
        end
end