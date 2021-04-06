function [MI] = PV(FixedImg, MovingImg, TransPar, GrayScale)
%{ 
    1、函数功能：PV插值，用于计算互信息
    2、函数输入：
               FixedImg        参考图像
               MovingImg       浮动图像
               TransPar        变换参数
               GrayScale       灰度级
    3、函数输出：
               MI              计算得到的互信息             
    4、函数版本：
               Vision-1.0      V1.0-CYZ-2021-01-19
%}
%{
    TransPar变换参数说明：
    1、二维图像：
               刚性变换：x,y平移, x,y旋转（4个参数）
               仿射变换：x,y平移, x,y旋转, x,y尺度（6个参数）
    2、三维图像：
               刚性变换：x,y,z平移, x,y,z旋转（6个参数）
               仿射变换：x,y,z平移, x,y,z旋转, x,y,z尺度（9个参数）
%}

% 参数初始化
if size(FixedImg) ~= size(MovingImg)
    errordlg('输入图像错误','固定图像和浮动图像的分辨率不一致！请重新输入！')
end
global TransGPoints;
FixedImg = double(FixedImg);    
MovingImg = double(MovingImg);   
% 对两幅图像进行最大最小归一化处理,之后进行灰度级操作
if max(FixedImg(:))~=min(FixedImg(:)) && max(MovingImg(:))~=min(MovingImg(:))
%     FixedImg = (FixedImg-min(FixedImg(:))) / (max(FixedImg(:))-min(FixedImg(:)));
%     MovingImg = (MovingImg-min(MovingImg(:))) / (max(MovingImg(:))-min(MovingImg(:)));
    FixedImg = double(int16(FixedImg*(GrayScale-1)))+1;
    MovingImg = double(int16(MovingImg*(GrayScale-1)))+1;
end
% 


% 构建变换矩阵
if size(FixedImg, 3)==1
    if size(TransPar)==4         % 二维刚性变换
        disp('功能尚未开发，敬请期待');return;
    elseif size(TransPar)==6     % 二维仿射变换
        disp('功能尚未开发，敬请期待');return;
    end
else
    % 坐标变换，计算浮动图像中像素点变换后在固定图像中的位置
    % 生成输出图像的网格点
    [GY, GX, GZ] = meshgrid(1:size(FixedImg,1), 1:size(FixedImg,2), 1:size(FixedImg,3));
    % reshape网格点矩阵，构建[x,y,z,1]'形式
    PixelsNum = size(FixedImg,1)*size(FixedImg,2)*size(FixedImg,3);
    GX = reshape(GX,PixelsNum,1);
    GY = reshape(GY,PixelsNum,1);
    GZ = reshape(GZ,PixelsNum,1);
    G = ones(size(GZ));
    GPoints = [GX,GY,GZ,G]'; clear GX GY GZ G;
    RotationX = [1,     0,                      0,                      0;
                 0,     cos(TransPar(4)),       sin(TransPar(4)),       0;
                 0,     -sin(TransPar(4)),      cos(TransPar(4)),       0;
                 0,     0,                      0,                      1];

    RotationY = [cos(TransPar(5)),     0,       -sin(TransPar(5)),      0;
                 0,                    1,       0,                      0;
                 sin(TransPar(5)),     0,       cos(TransPar(5)),       0;
                 0,                    0,       0,                      1];

    RotationZ = [cos(TransPar(6)),     sin(TransPar(6)),       0,       0;
                 -sin(TransPar(6)),    cos(TransPar(6)),       0,       0;
                 0,                    0,                      1,       0;
                 0,                    0,                      0,       1];

    SetOrigin = [1,     0,      0,      -double(size(FixedImg,1)/2);
                 0,     1,      0,      -double(size(FixedImg,2)/2);
                 0,     0,      1,      -double(size(FixedImg,3)/2);
                 0,     0,      0,                               1];

    Translation = [1,     0,      0,      TransPar(1);
                   0,     1,      0,      TransPar(2);
                   0,     0,      1,      TransPar(3);
                   0,     0,      0,                1];     

    BackOrigin = [1,     0,      0,      double(size(FixedImg,1)/2);
                  0,     1,      0,      double(size(FixedImg,2)/2);
                  0,     0,      1,      double(size(FixedImg,3)/2);
                  0,     0,      0,                              1];
    
    if size(TransPar,2)==6     % 三维刚性变换
        TransMartix = BackOrigin * Translation * RotationZ * RotationY * RotationX * SetOrigin;
        TransGPoints = TransMartix * GPoints;   
        TransGPoints(4,:) = [];
    elseif size(TransPar,2)==9     % 三维仿射变换
        Scale = [TransPar(7),     0,                0,                0;
                 0,               TransPar(8),      0,                0;
                 0,               0,                TransPar(9),      0;
                 0,               0,                0,                1];
        TransMartix = BackOrigin * Translation * RotationZ * RotationY * RotationX * Scale * SetOrigin;   
        TransGPoints = TransMartix * GPoints;
        TransGPoints(4,:) = [];     
    end
    
    % 进行PV插值
    % 处理出界点
    % 获取InputPoints中界内点的索引
    [~, Col1] = find(TransGPoints(1,:)<0 | TransGPoints(1,:)>size(FixedImg,1));
    Col1 = unique(Col1);%删除重复的索引
    [~, Col2] = find(TransGPoints(2,:)<0 | TransGPoints(2,:)>size(FixedImg,2));
    Col2 = unique(Col2);
    [~, Col3] = find(TransGPoints(3,:)<0 | TransGPoints(3,:)>size(FixedImg,3));
    Col3 = unique(Col3);
    Col = intersect(Col1,Col2); %求得交集
    Col = intersect(Col3,Col);
    % 得到出界点参考图像灰度值
    OutSidePointValue = FixedImg(sub2ind(size(FixedImg),GPoints(1,Col),GPoints(2,Col),GPoints(3,Col)));
    % 浮动图像和固定图像的归一化联合直方图
    HistMF = double(zeros(GrayScale, GrayScale+1));  
    OutSidePointIndex = sub2ind(size(HistMF),OutSidePointValue,double(ones(size(OutSidePointValue)))*(GrayScale+1));
    HistMF(OutSidePointIndex) = HistMF(OutSidePointIndex) + 1;
    
    % 界内点处理
    TransGPoints(:,Col) = []; GPoints(:,Col) = []; clear Col1 Col2 Col3 Col;
    AGPoints1 = floor(TransGPoints);
    O = ones(1,size(TransGPoints,2));
    Z = zeros(1,size(TransGPoints,2));
    AGPoints2 = AGPoints1 + [Z;O;Z];%010
    AGPoints3 = AGPoints1 + [O;O;Z];%110
    AGPoints4 = AGPoints1 + [O;Z;Z];%100
    AGPoints5 = AGPoints1 + [Z;Z;O];%001
    AGPoints6 = AGPoints1 + [Z;O;O];%011
    AGPoints7 = AGPoints1 + [O;O;O];%111
    AGPoints8 = AGPoints1 + [O;Z;O];%101  
    GrayF = FixedImg(sub2ind(size(FixedImg),GPoints(1,:),GPoints(2,:),GPoints(3,:)));  clear O Z;
    D = TransGPoints - AGPoints1;
    DX = D(1,:)';
    DY = D(2,:)';
    DZ = D(3,:)'; clear D;
    GrayM1 = GetPixelValue(AGPoints1,MovingImg); clear AGPoints1;
    HistMF = accumarray([GrayM1 GrayF'], (1-DX).*(1-DY).*(1-DZ)); clear GrayM1;
    GrayM2 = GetPixelValue(AGPoints2,MovingImg); clear AGPoints2;
    HistMF = HistMF + accumarray([GrayM2 GrayF'], DX.*(1-DY).*(1-DZ)); clear GrayM2;
    GrayM3 = GetPixelValue(AGPoints3,MovingImg); clear AGPoints3;
    HistMF = HistMF + accumarray([GrayM3 GrayF'], DX.*DY.*(1-DZ)); clear GrayM3;
    GrayM4 = GetPixelValue(AGPoints4,MovingImg); clear AGPoints4;
    HistMF = HistMF + accumarray([GrayM4 GrayF'], (1-DX).*DY.*(1-DZ)); clear GrayM4;
    GrayM5 = GetPixelValue(AGPoints5,MovingImg); clear AGPoints5;
    HistMF = HistMF + accumarray([GrayM5 GrayF'], (1-DX).*(1-DY).*DZ); clear GrayM5;
    GrayM6 = GetPixelValue(AGPoints6,MovingImg); clear AGPoints6;
    HistMF = HistMF + accumarray([GrayM6 GrayF'], DX.*(1-DY).*DZ); clear GrayM6;
    GrayM7 = GetPixelValue(AGPoints7,MovingImg); clear AGPoints7;
    HistMF = HistMF + accumarray([GrayM7 GrayF'], DX.*DY.*DZ); clear GrayM7;
    GrayM8 = GetPixelValue(AGPoints8,MovingImg); clear AGPoints8;
    HistMF = HistMF + accumarray([GrayM8 GrayF'], (1-DX).*DY.*DZ); clear GrayM8 DX DY DZ;
    HistMFSum = sum(HistMF(:));
    index = find(HistMF~=0);
    ProbMF = HistMF/HistMFSum;
    EntropyMF = -ProbMF(index).*log2(ProbMF(index));
    EntropyMF = sum(EntropyMF(:));
    ProbM = sum(ProbMF,2);
    index = find(ProbM~=0);
    EntropyM = -ProbM(index).*log2(ProbM(index));
    EntropyM = sum(EntropyM(:));
    ProbF = sum(ProbMF,1);
    index = find(ProbF~=0);
    EntropyF = -ProbF(index).*log2(ProbF(index));
    EntropyF = sum(EntropyF(:));
    MI = -(EntropyM + EntropyF - EntropyMF);
end
    
% 获取Img中InputPoints位置处的像素值，出界点像素值为0
function OutPoints = GetPixelValue(InputPoints,Img)
     % 获取InputPoints中界内点的索引
    [~, Col1] = find(InputPoints(1,:)>0 & InputPoints(1,:)<=size(Img,1));
    Col1 = unique(Col1);%删除重复的索引
    [~, Col2] = find(InputPoints(2,:)>0 & InputPoints(2,:)<=size(Img,2));
    Col2 = unique(Col2);
    [~, Col3] = find(InputPoints(3,:)>0 &InputPoints(3,:)<=size(Img,3));
    Col3 = unique(Col3);
    Col = intersect(Col1,Col2); %求得交集
    Col = intersect(Col3,Col);
    % 将出界点置0
    OutPoints = ones(size(InputPoints,2),1);
    OutPoints(Col,1) = Img(sub2ind(size(Img),InputPoints(1,Col),InputPoints(2,Col),InputPoints(3,Col)));
