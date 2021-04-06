function [OutPoint, OutValue, OutLamada] = OneDimensionSearch(Func, StartPoint, StepLength, Dir, Error, FixedImg, MovingImg)
%{ 
    1、函数功能：一维线性搜索算法（进退法加黄金分割法），用于Powell算法
    2、函数输入：
               Func            相似性测度函数
               StartPoint      初始搜索点(一般为原点)
               StepLength      进退法搜索步长
               Dir             搜索方向
               Error           允许误差
               FixedImg        参考图像
               MovingImg       浮动图像
    3、函数输出：
               OutPoint        输出当前搜索方向上的极小点
               OutValue        该极小点的函数值
               OutLamada       该极小点在当前搜索方向上的步长
    4、函数版本：
               Vision-1.0      V1.0-CYZ-2021-01-19
%}

%% 首先利用进退法获得包含极小点的区间
% 参数初始化
StartLamada = 0;
StartLamada = double(StartLamada);StepLength = double(StepLength);
NextLamada = StartLamada + StepLength;
NextFuncValue = Func(StartPoint + NextLamada*Dir, FixedImg, MovingImg);
StartFuncValue = Func(StartPoint + StartLamada*Dir, FixedImg, MovingImg);

if NextFuncValue < StartFuncValue
    LastLamada = StartLamada;
    StartLamada = NextLamada;
    StartFuncValue = NextFuncValue;
    StepLength = StepLength * 2.0;
else 
    LastLamada = NextLamada;
    StepLength = -1.0 * StepLength;
end

while 1
    NextLamada = StartLamada + StepLength;
    NextFuncValue = Func(StartPoint + NextLamada*Dir, FixedImg, MovingImg);
    if NextFuncValue < StartFuncValue
        LastLamada = StartLamada;
        StartLamada = NextLamada;
        StartFuncValue = NextFuncValue;
        StepLength = StepLength * 2.0;
    else
        LeftValue = min(LastLamada, NextLamada);
        RightValue = max(LastLamada, NextLamada);
        break;
    end
end
%%

%% 然后利用黄金分割法搜索极小点
LeftValue = double(LeftValue);RightValue = double(RightValue);Error = double(Error);
Lamada = 0.382 * (RightValue - LeftValue) + LeftValue;
Mu = 0.618 * (RightValue - LeftValue) + LeftValue;
while (RightValue - LeftValue) > Error
    if Func(StartPoint + Lamada*Dir, FixedImg, MovingImg) > Func(StartPoint + Mu*Dir, FixedImg, MovingImg)
        LeftValue = Lamada;
        Lamada = Mu;
        Mu = 0.618 * (RightValue - LeftValue) + LeftValue;
    else 
        RightValue = Mu;
        Mu = Lamada;
        Lamada = 0.382 * (RightValue - LeftValue) + LeftValue;
    end
end
OutLamada = (RightValue + LeftValue)/2;
OutPoint = StartPoint + OutLamada*Dir;
OutValue = Func(OutPoint, FixedImg, MovingImg);
%%