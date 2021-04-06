function OutputValue = GoldenSectionOptimizer(Func, LeftValude, RightValue, Error)
%% 函数介绍
% 函数功能：黄金分割法，即一维线性搜索方法。
% 函数输入：
%           Func            待优化函数
%           LeftValude      初始区间左值
%           RightValue      初始区间右值
%           Error           允许误差
% 函数输出：
%           OutputValue     输出结果
% 函数版本：
%           Vision-1.0      V1.0-CYZ-2021-01-18

%% 函数实现
if LeftValude > RightValue
    errordlg('区间左值必须小于或等于区间右值！','初始区间赋值错误');
end

LeftValude = double(LeftValude);RightValue = double(RightValue);Error = double(Error);
Lamada = 0.382 * (RightValue - LeftValude) + LeftValude;
Mu = 0.618 * (RightValue - LeftValude) + LeftValude;
while (RightValue - LeftValude) > Error
    if Func(Lamada) > Func(Mu)
        LeftValude = Lamada;
        Lamada = Mu;
        Mu = 0.618 * (RightValue - LeftValude) + LeftValude;
    else 
        RightValue = Mu;
        Mu = Lamada;
        Lamada = 0.382 * (RightValue - LeftValude) + LeftValude;
    end
end
OutputValue = (RightValue + LeftValude)/2;