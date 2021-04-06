function OutputValue = PowellMethod(Func, StartPoint, DimeOfVar, StepLength, Error, FixedImg, MovingImg)
%{ 
    1、函数功能：Powell算法，用于配准
    2、函数输入：
               Func            相似性测度函数
               StartPoint      初始搜索点(一般为原点)
               DimeOfVar       搜索空间的维度
               StepLength      一维搜索步长
               Error           允许误差
               FixedImg        参考图像
               MovingImg       浮动图像
    3、函数输出：
               OutputValue     输出结果
    4、函数版本：
               Vision-1.0      V1.0-CYZ-2021-01-19
%}

%% 参数初始化
    StartPoint = double(StartPoint); 
    Num = DimeOfVar; %搜索空间的维度
    % 初始化搜索方向，保证其线性无关
    Diraction = double(zeros(Num,Num));
    for i = 1:Num
        Diraction(i,i) = 1;
    end
%%

%% 预分配内存
    % CurrentDir更新后的方向矩阵
    CurrentDir = double(zeros(Num+1,Num));
    % 搜索得到的点
    X = double(zeros(Num+1,Num));
    % 搜索得到的点的函数值
    FuncX = double(zeros(Num+1));
    % 各点函数值之间的差值
    Diff = double(zeros(Num));
%%

%% 迭代优化
iterationNum = 0;
while 1
    iterationNum = iterationNum + 1;disp(iterationNum);
    %---------------------------------------Step1-------------------------------------
    % 第一次迭代，第一个方向
    [X(1,:), FuncX(1), ~] = OneDimensionSearch(Func, StartPoint, StepLength, Diraction(1,:), Error, FixedImg, MovingImg);
    %沿着第2到第N方向进行一维搜索
    for k = 2:Num
        CurrentDir(k,:) = Diraction(k,:);
        [X(k,:), FuncX(k), ~] = OneDimensionSearch(Func, X(k-1,:), StepLength, CurrentDir(k,:), Error, FixedImg, MovingImg);
    end
    %构造新的搜索方向向量 并以X0为起点，沿此方向进行新一轮搜索，得到相应的极值
    CurrentDir(Num+1,:) = X(Num,:)-StartPoint;
    [X(Num+1,:), FuncX(Num+1), Lamda] = OneDimensionSearch(Func, StartPoint, StepLength, CurrentDir(Num+1,:), Error, FixedImg, MovingImg);
    %---------------------------------------Step2-------------------------------------
    %判断迭代终止条件
    Delta = X(Num+1,:)-StartPoint;%收敛判断
    MoldOfDelta = Delta.*Delta; %计算Delta的模长
    MoldOfDelta = sqrt(sum(MoldOfDelta(:)));
    if( MoldOfDelta <= Error )% 判断精度是否达到要求
        OutputValue = X(Num+1,:); %输出最优解和最优参数
        break;%跳过后续代码
    else
    %---------------------------------------Step3-------------------------------------        
        StartPointFuncValue = Func(StartPoint, FixedImg, MovingImg);
        %计算各个方向上的下降量
        Diff(1) = StartPointFuncValue-FuncX(1);%此时我们在寻找极小值
        for k = 2:Num
            Diff(k) =  FuncX(k-1)-FuncX(k);
        end
        %记录最下降量 以及 其出现的位置 求最大下降量
        MaxDiff = max(Diff(:));
        index = find(Diff == MaxDiff);
        m = index(1);
    %---------------------------------------Step4-------------------------------------
        %判断Powell条件
        if abs(Lamda) > sqrt( (StartPointFuncValue - FuncX(Num+1))/MaxDiff )
            %如果满足,即为非线性相关 %设置下一轮搜索的初始点
            StartPoint = X(Num+1,:);
            %替换m位置的搜索方向
            Diraction(1:m-1,:) = CurrentDir(1:m-1,:);
            Diraction(m:end,:) = CurrentDir(m+1:end,:);
        else %如果不满足，即为线性相关，%保持原来的搜索方向，设置下一轮搜索始点
            StartPoint = X(Num+1,:);
        end
    end
end
%%