function OutputImg = AffineTransform_cyz(InputImg, AffineMatrix, TranslationMatrix)
%% 函数功能：对图像进行仿射变换（反向映射）
% InputImg：输入的待配准图像
% AffineMatrix：变换矩阵，形式如下：
%                   [a1,      a2,      a3,       0;
%                    a4,      a5,      a6,       0;
%                    a7,      a8,      a9,       0;
%                    0,       0,       0,        1]
% TranslationMatrix：仿射变换中的平移参数，形式如下：
%                   [t1,      t2,      t3,       0]

%% 仿射变换流程
% 该函数为仿射变换的反向映射，故假设浮动图像坐标为MP(MovingPoints)，仿射变换后的图像的坐标为AP(AffinePoints)
% 由于反向映射，故将仿射变换后的图像的坐标AP视为网格点，计算AP在浮动图像中的对应的坐标点MP，由于MP不一定为整数，所以需要插值
% 有AP得到MP的变换公式为   MP = A * ( AP - TC ) + TC + T
% 即先将仿射后的图像的中心移至原点（0，0），再进行旋转和尺度操作，然后将图像中心移至原位，最后进行平移操作
% 其中A为AffineMatrix，TC（TranslationToCenter）为图像中心点坐标，T为TranslationMatrix。

%% 函数实现
% 生成输出图像的网格点
[GY, GX, GZ] = meshgrid(1:size(InputImg,1), 1:size(InputImg,2), 1:size(InputImg,3));
% reshape网格点矩阵，构建[x,y,z,1]'形式
PixelsNum = size(InputImg,1)*size(InputImg,2)*size(InputImg,3);
GX = reshape(GX,PixelsNum,1);
GY = reshape(GY,PixelsNum,1);
GZ = reshape(GZ,PixelsNum,1);
G = ones(size(GZ ));
GPoints = [GX,GY,GZ,G]';
% 仿射变换矩阵
AffineMatrix = double(AffineMatrix);
% 平移矩阵
TranslationMatrix = double(TranslationMatrix);
TranslationMatrix = repmat(TranslationMatrix, 1, size(GPoints,2));
% 将图像中心移至原点，再进行旋转
T_SetOrigin = double([(size(InputImg,1)+1)/2,(size(InputImg,2)+1)/2,(size(InputImg,3)+1)/2,1]');
T_SetOrigin = repmat(T_SetOrigin, 1, size(GPoints,2));
% 计算输出图像的网格点坐标经过仿射变换后的坐标
AffineGPoints =  double(AffineMatrix * (GPoints - T_SetOrigin ) + TranslationMatrix + T_SetOrigin);   
AffineGPoints(4,:) = [];

%% 三线性插值
%计算变换后坐标的领域立方体坐标（8个）
AGPoints1 = floor(AffineGPoints);
O = ones(1,size(AffineGPoints,2));
Z = zeros(1,size(AffineGPoints,2));
AGPoints2 = AGPoints1 + [Z;O;Z];%010
AGPoints3 = AGPoints1 + [O;O;Z];%110
AGPoints4 = AGPoints1 + [O;Z;Z];%100
AGPoints5 = AGPoints1 + [Z;Z;O];%001
AGPoints6 = AGPoints1 + [Z;O;O];%011
AGPoints7 = AGPoints1 + [O;O;O];%111
AGPoints8 = AGPoints1 + [O;Z;O];%101
D = AffineGPoints - AGPoints1;
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

OutputImg = reshape(AffineImg,size(InputImg));

end

%% 辅助函数
% 一维线性插值函数，D为代插值点与InputPoints1之间的距离，默认InputPoints1和InputPoints2之间的距离为1
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
    OutPoints = zeros(size(InputPoints,2),1);
    OutPoints(Col,1) = Img(sub2ind(size(Img),InputPoints(1,Col),InputPoints(2,Col),InputPoints(3,Col)));
end