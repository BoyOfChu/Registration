clc;clear;
% F = @Func;
% OutputValue = ParabolicInterpolationOptimizer(F, 0, 0.5, 1, 0.0001);
% 
% function Y = Func(X)
% Y = X^2 - sin(X);
% end

FixedImg = load_nii('D:\ImageRegistration\UltrasoundImageMosaicing\Data_NII_Processed\patient_1\1.nii');
FixedImg = FixedImg.img;FixedImg = FixedImg(:,:,26:44);
MovingImg = load_nii('D:\ImageRegistration\UltrasoundImageMosaicing\Data_NII_Processed\patient_1\2.nii');
MovingImg = MovingImg.img;MovingImg = MovingImg(:,:,29:47);
nii_Img = make_nii(FixedImg);
save_nii(nii_Img,'D:\ImageRegistration\UltrasoundImageMosaicing\RoIData\patient_1\1_temp.nii');
nii_Img = make_nii(MovingImg);
save_nii(nii_Img,'D:\ImageRegistration\UltrasoundImageMosaicing\RoIData\patient_1\2_temp.nii');
TransPar = [0, 0, 0, 0, 0, 0]; 
% F = FixedImg(:,:,35:38); M = MovingImg(:,:,30:40);
% F = double(int16(F*(100-1)));

GrayScale = 256;
% tic;pv_MI = PV(FixedImg, MovingImg, TransPar, GrayScale);toc;
% tic; OutputImg = AffineTransform2D3D_GPU(FixedImg, TransPar);toc;
% tic;MI = MutualInformation(FixedImg, MovingImg, GrayScale);toc;

Func = @RigidFunc; 
StartPoint = [0, 0, 0, 0, 0, 0, 0, 0, 0];
OutputValue = PowellMethod(Func, StartPoint, 9, 0.5, 0.1, FixedImg, MovingImg);
% TransPar = [-29.0524477412745,-32.2149646490508,-0.0220296923773858,-0.00134477358987511,0.00990888249876404,-0.433080128547611];
OutputImg = AffineTransform2D3D_GPU(MovingImg, OutputValue);
nii_Img = make_nii(OutputImg);
save_nii(nii_Img,'D:\ImageRegistration\UltrasoundImageMosaicing\RoIData\patient_1\2_trans_affine.nii');

function MI = RigidFunc(TransPar, FixedImg, MovingImg)
    MovingImg = AffineTransform2D3D_GPU(MovingImg, TransPar);
    MI = MutualInformation(FixedImg, MovingImg, 256);
end