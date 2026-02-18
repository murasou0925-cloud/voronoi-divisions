function zncoord = RRT_steer(zrcoord, znearestcoord, val, eps)
% z_randとz_nearestの距離が，あらかじめ定められた距離epsよりも大きい場合において
% z_nearestからz_randの方向にeps伸ばした位置にz_newを取る関数
% z_randとz_nearestの距離がepsより小さい場合はz_randをz_newとする
    if val >= eps
       znx = znearestcoord(1) + ((zrcoord(1)-znearestcoord(1))*eps)/norm(zrcoord-znearestcoord);
       zny = znearestcoord(2) + ((zrcoord(2)-znearestcoord(2))*eps)/norm(zrcoord-znearestcoord);
       znz = znearestcoord(3) + ((zrcoord(3)-znearestcoord(3))*eps)/norm(zrcoord-znearestcoord);
    else
       znx = zrcoord(1);
       zny = zrcoord(2);
       znz = zrcoord(3);
    end
    
    zncoord = [znx, zny, znz];
end