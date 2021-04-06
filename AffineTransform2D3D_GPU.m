function OutputImg = AffineTransform2D3D_GPU(InputImg, TransPar)
%{ 
    1、函数功能：2D3D图像仿射变换，包含刚性变换
    2、函数输入：
               InputImg        输入图像
               TransPar        变换参数
    3、函数输出：
               OutputImg       变换后的输出图像             
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

if size(InputImg, 3)==1
    if size(TransPar)==4         % 二维刚性变换
        disp('功能尚未开发，敬请期待');return;
    elseif size(TransPar)==6     % 二维仿射变换
        disp('功能尚未开发，敬请期待');return;
    end
else
    % 坐标变换，计算浮动图像中像素点变换后在固定图像中的位置
    % 生成输出图像的网格点
    InputImg = gpuArray(double(InputImg));
    [GY, GX, GZ] = meshgrid(1:size(InputImg,1), 1:size(InputImg,2), 1:size(InputImg,3));
    % reshape网格点矩阵，构建[x,y,z,1]'形式
    PixelsNum = size(InputImg,1)*size(InputImg,2)*size(InputImg,3);
    GX = reshape(GX,PixelsNum,1);
    GY = reshape(GY,PixelsNum,1);
    GZ = reshape(GZ,PixelsNum,1);
    G = ones(size(GZ));
    GPoints = [GX,GY,GZ,G]'; clear GX GY GZ G;
    GPoints = gpuArray(GPoints);
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

    SetOrigin = [1,     0,      0,      -double(size(InputImg,1)/2);
                 0,     1,      0,      -double(size(InputImg,2)/2);
                 0,     0,      1,      -double(size(InputImg,3)/2);
                 0,     0,      0,                               1];

    Translation = [1,     0,      0,      TransPar(1);
                   0,     1,      0,      TransPar(2);
                   0,     0,      1,      TransPar(3);
                   0,     0,      0,                1];     

    BackOrigin = [1,     0,      0,      double(size(InputImg,1)/2);
                  0,     1,      0,      double(size(InputImg,2)/2);
                  0,     0,      1,      double(size(InputImg,3)/2);
                  0,     0,      0,                              1];
    
    if size(TransPar,2)==6     % 三维刚性变换
        TransMartix = BackOrigin * Translation * RotationZ * RotationY * RotationX * SetOrigin;
        TransMartix = gpuArray(TransMartix);
        TransGPoints = TransMartix * GPoints;   
        TransGPoints(4,:) = [];
    elseif size(TransPar,2)==9     % 三维仿射变换
        Scale = [TransPar(7),     0,                0,                0;
                 0,               TransPar(8),      0,                0;
                 0,               0,                TransPar(9),      0;
                 0,               0,                0,                1];
        TransMartix = BackOrigin * Translation * RotationZ * RotationY * RotationX * Scale * SetOrigin;   
        TransMartix = gpuArray(TransMartix);
        TransGPoints = TransMartix * GPoints;
        TransGPoints(4,:) = [];     
    end
    %% 三线性插值
    %计算变换后坐标的领域立方体坐标（8个）
    AGPoints1 = floor(TransGPoints);
    O = ones(1,size(TransGPoints,2),'gpuArray');
    Z = zeros(1,size(TransGPoints,2),'gpuArray');
    AGPoints2 = AGPoints1 + [Z;O;Z];%010
    AGPoints3 = AGPoints1 + [O;O;Z];%110
    AGPoints4 = AGPoints1 + [O;Z;Z];%100
    AGPoints5 = AGPoints1 + [Z;Z;O];%001
    AGPoints6 = AGPoints1 + [Z;O;O];%011
    AGPoints7 = AGPoints1 + [O;O;O];%111
    AGPoints8 = AGPoints1 + [O;Z;O];%101
    D = TransGPoints - AGPoints1; clear O Z;
    DX = D(1,:)';
    DY = D(2,:)';
    DZ = D(3,:)';
    %得到立方体顶点坐标在输入图像中对应位置像素值
    AGPoints1 =  GetPixelValue(AGPoints1,InputImg);
    AGPoints2 =  GetPixelValue(AGPoints2,InputImg);
    AGPoints3 =  GetPixelValue(AGPoints3,InputImg);
    AGPoints4 =  GetPixelValue(AGPoints4,InputImg);
    AGPoints5 =  GetPixelValue(AGPoints5,InputImg);
    AGPoints6 =  GetPixelValue(AGPoints6,InputImg);
    AGPoints7 =  GetPixelValue(AGPoints7,InputImg);
    AGPoints8 =  GetPixelValue(AGPoints8,InputImg);

    % 插值
    AGPointsA = Interp_1(AGPoints1,AGPoints2,DY);
    AGPointsB = Interp_1(AGPoints5,AGPoints6,DY);
    AGPointsC = Interp_1(AGPoints4,AGPoints3,DY);
    AGPointsD = Interp_1(AGPoints8,AGPoints7,DY);
    AGPointsE = Interp_1(AGPointsA,AGPointsC,DX);
    AGPointsF = Interp_1(AGPointsB,AGPointsD,DX);
    AffineImg = Interp_1(AGPointsE,AGPointsF,DZ);
    
    AffineImg = gather(AffineImg);
    OutputImg = reshape(AffineImg,size(InputImg));
end
end


%% 辅助函数
% 一维线性插值函数，D为待插值点与InputPoints1之间的距离，默认InputPoints1和InputPoints2之间的距离为1
% 其中InputPoints1比InputPoints2距离原点更近
function OutPoints = Interp_1(InputPoints1,InputPoints2,D)
    OutPoints = InputPoints1 + (InputPoints2 - InputPoints1).*D;
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
    OutPoints = zeros(size(InputPoints,2),1,'gpuArray');
    OutPoints(Col,1) = Img(sub2ind(size(Img),InputPoints(1,Col),InputPoints(2,Col),InputPoints(3,Col)));
end