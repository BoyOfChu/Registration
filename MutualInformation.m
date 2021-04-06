function MI = MutualInformation(FixedImg, MovingImg, GrayScale)
%{ 
    1、函数功能：计算互信息
    2、函数输入：
               FixedImg        参考图像
               MovingImg       浮动图像
               GrayScale       灰度级
    3、函数输出：
               MI              计算得到的互信息             
    4、函数版本：
               Vision-1.0      V1.0-CYZ-2021-01-20
%}
% 参数初始化
if size(FixedImg) ~= size(MovingImg)
    errordlg('输入图像错误','固定图像和浮动图像的分辨率不一致！请重新输入！')
end
FixedImg = int16(FixedImg*(GrayScale-1))+1;
MovingImg = int16(MovingImg*(GrayScale-1))+1;
FixedImg = FixedImg(:);
MovingImg = MovingImg(:);
% 计算联合熵
jointHistogram = accumarray([FixedImg MovingImg], 1);
jointProb = jointHistogram / numel(FixedImg);
indNoZero = find(jointHistogram ~= 0);
jointProb1DNoZero = jointProb(indNoZero);
jointEntropy = -sum(sum(jointProb1DNoZero.*log2(jointProb1DNoZero)));
% 计算图像熵
histogramImage1 = sum(jointHistogram, 1);
histogramImage2 = sum(jointHistogram, 2);
indNoZero = find(histogramImage1 ~= 0);
prob1NoZero = histogramImage1 / numel(FixedImg);
entropy1 = -sum(prob1NoZero(indNoZero).*log2(prob1NoZero(indNoZero)));
indNoZero = find(histogramImage2 ~= 0);
prob2NoZero = histogramImage2 / numel(FixedImg);
entropy2 = -sum(prob2NoZero(indNoZero).*log2(prob2NoZero(indNoZero)));
MI = - (entropy1 + entropy2 - jointEntropy);