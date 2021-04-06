function OutputValue = ParabolicInterpolationOptimizer(Func, LeftValue, MidValue, RightValue, Error)
%% 函数介绍
% 函数功能：抛物线插值法，又称二次插值法，是一维线性搜索方法。
% 函数输入：
%           Func            待优化函数
%           LeftValue       初始左值
%           MidValue        初始中值
%           RightValue      初始右值
%           Error           允许误差
% 函数输出：
%           OutputValue        输出结果
% 函数版本：
%           Vision-1.0      V1.0-CYZ-2021-01-18

%% 函数实现
%初始化参数，放在这里是为了减少重复计算
LeftValue = double(LeftValue); RightValue = double(RightValue);
MidValue = double(MidValue); Error = double(Error);
LeftFuncValue = Func(LeftValue);
MidFuncValue = Func(MidValue);
RightFuncValue = Func(RightValue);
if ~(LeftValue < MidValue && MidValue < RightValue)
    errordlg('初始点必须满足LeftValue < MidValue < RightValue！','初始点错误');
end
if ~(LeftFuncValue > MidFuncValue && MidFuncValue < RightFuncValue)
    errordlg('初始点必须满足Func(LeftValue) > Func(MidValue) < Func(RightValue)！','初始点错误');
end
while (RightValue - LeftValue) > Error    
    B1 = (MidValue^2 - RightValue^2) * LeftFuncValue;
    B2 = (RightValue^2 - LeftValue^2) * MidFuncValue;
    B3 = (LeftValue^2 - MidValue^2) * RightFuncValue;
    C1 = (MidValue - RightValue) * LeftFuncValue;
    C2 = (RightValue - LeftValue) * MidFuncValue;
    C3 = (LeftValue - MidValue) * RightFuncValue;
    MinmumValue = (B1 + B2 + B3)/(2 * (C1 + C2 + C3));
    MinmumFuncValue = Func(MinmumValue);
    if MinmumFuncValue >= MidFuncValue
        if MidValue >= MinmumValue
            LeftValue = MinmumValue;LeftFuncValue = MinmumFuncValue;            
        else
            RightValue = MinmumValue;RightFuncValue = MinmumFuncValue; 
        end
    else
        if MidValue >= MinmumValue  
            RightValue = MidValue;RightFuncValue = MidFuncValue; 
            MidValue = MinmumValue; MidFuncValue = MinmumFuncValue;
        else
            LeftValue = MidValue;LeftFuncValue = MidFuncValue; 
            MidValue = MinmumValue; MidFuncValue = MinmumFuncValue;
        end
    end
end
OutputValue = MidValue;
