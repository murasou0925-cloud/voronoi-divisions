%初期位置設定
function P = Init(x)

switch x
    case 5   %5台
        P.position = [1 2 4 4 5 ; 1 4 1 5 3]';   % 初期位置
        P.target   = [4 5 2 1 1 ; 4 5 1 2 3]';   % 最終目標位置
        P.object   = [20 ; 20]';                 % 障害物の位置
        P.dronenum = 5;                          % 機体の数
    case 4   %4台 
        P.position = [1 2 7 4 ; 1 3 1 5]';
        P.target   = [5 7 2 1 ; 3 5 4 2]';
        P.object   = [20 ; 20]';
        P.dronenum = 4;
    case 3   %3台
        P.position = [1 4 5 ; 2 1 4]';
        P.target   = [4 3 2 ; 2 4 1]';
        P.object   = [2 3 ; 3 2]';
        P.dronenum = 3;
    case 2
        P.position = [1 5 ; 1 2.7]';
        P.target   = [5 1.6 ; 3.2 1]';
        P.object   = [20 ; 20]';
        P.dronenum = 2;
    case 1
        P.position = [3 ; 0.5]';
        P.target   = [3 ; 4]';
        P.object   = [2.8 3.2 ; 3 3]';
        P.dronenum = 1;
    case 6   %ランダム
        P.dronenum = 10;   %ドローンの数
        P.position = randn(P.dronenum,2)*5;   %初期位置
        P.target   = randn(P.dronenum,2)*5;   %最終目標位置
end

%各パラメータの設定
P.limit = 100;   %最終終了時刻
P.interval = 0.2;   %更新時間間隔の設定
P.speed = 0.1;   %機体の速度の設定
P.width = 0.2; %マージンの幅の設定
P.width_base=0.5; %基本マージン
P.threshold = 0.1;   %終了判定の閾値
P.gain = 10;   %ゲイン
P.subgain = 1;

%描画用パラメータの設定
P.animecounter = 1;   %アニメーション作成用のカウンタ 
P.textboxdimention = [.15 .84 .20 .06];   %時刻描画ボックスの位置と設定

end
