function [LeftValue, RightValue]= AdvanceAndRetreatMethod(Func, StartPoint, StepLength)
%{
    1、函数功能：进退法，用来确定包含极小点的区间。
    2、函数输入：
              Func            待优化函数
              StartPoint      初始点
              StepLength      步长
    3、函数输出：
              LeftValue       包含极小点的区间的左值
              RightValue      包含极小点的区间的右值    
    4、函数版本：
              Vision-1.0      V1.0-CYZ-2021-01-19
%}
StartPoint = double(StartPoint);StepLength = double(StepLength);
NextPoint = StartPoint + StepLength;
NextFuncValue = Func(NextPoint);
StartFuncValue = Func(StartPoint);
if NextFuncValue < StartFuncValue
    LastPoint = StartPoint;
    StartPoint = NextPoint;
    StartFuncValue = NextFuncValue;
    StepLength = StepLength * 2.0;
else 
    LastPoint = NextPoint;
    StepLength = -1.0 * StepLength;
end

while 1
    NextPoint = StartPoint + StepLength;
    NextFuncValue = Func(NextPoint);
    if NextFuncValue < StartFuncValue
        LastPoint = StartPoint;
        StartPoint = NextPoint;
        StartFuncValue = NextFuncValue;
        StepLength = StepLength * 2.0;
    else
        LeftValue = min(LastPoint, NextPoint);
        RightValue = max(LastPoint, NextPoint);
        break;
    end
end