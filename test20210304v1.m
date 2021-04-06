clc;clear;
FixedImg = load_nii('E:\CBCT_CT_Data\TrainingData\CBCTnew0\1.nii');
FixedImg = FixedImg.img; FixedImg = medfilt3(FixedImg,[3 3 3]);
[sx, sy, sz] = size(FixedImg);
width = (11-1)/2; height = (7-1)/2;
TempImg = zeros(sx+width*2, sy+width*2, sz+height*2);
TempImg(1+width:width+sx, 1+width:width+sy, 1+height:height+sz) = FixedImg;
ProImg = zeros(size(FixedImg));

Mask = double(imbinarize(FixedImg, 0));
SE = strel('disk',3);
Mask = imclose(Mask,SE);
for i = 1:sz
    SE = strel('disk',3);
    Mask(:,:,i) = imopen(Mask(:,:,i),SE);
    Mask(:,:,i) = imfill(Mask(:,:,i),'holes');
end
nii_Img = make_nii(Mask);
save_nii(nii_Img,'D:\ImageRegistration\RegistrationMatlab\Data\Temp\1_Mask.nii');
for i = 1 + width : sx+width
     for j = 1 + width : sy+width
         for k = 1 + height : sz + height
             ROI = TempImg(i-width:i+width, j-width:j+width, k-height:k+height);
             [count, x] = imhist(ROI,256) ;
%              stem(x,count);
             g = round(ROI(width+1,width+1,height+1)*255);
             ProImg(i-width, j-width, k-height) = count(g+1)/sum(count(1:end));             
         end
     end
end
nii_Img = make_nii((ProImg).*Mask);
save_nii(nii_Img,'D:\ImageRegistration\RegistrationMatlab\Data\Temp\CBCT_1.nii');

FixedImg = load_nii('E:\CBCT_CT_Data\TrainingData\LCTnew0\1.nii');
FixedImg = FixedImg.img; FixedImg = medfilt3(FixedImg,[3 3 3]);
[sx, sy, sz] = size(FixedImg);
width = (11-1)/2; height = (7-1)/2;
TempImg = zeros(sx+width*2, sy+width*2, sz+height*2);
TempImg(1+width:width+sx, 1+width:width+sy, 1+height:height+sz) = FixedImg;
ProImg = zeros(size(FixedImg));

Mask = double(imbinarize(FixedImg, 0));
SE = strel('disk',3);
Mask = imclose(Mask,SE);
for i = 1:sz
    SE = strel('disk',3);
    Mask(:,:,i) = imopen(Mask(:,:,i),SE);
    Mask(:,:,i) = imfill(Mask(:,:,i),'holes');
end
nii_Img = make_nii(Mask);
save_nii(nii_Img,'D:\ImageRegistration\RegistrationMatlab\Data\Temp\1_Mask.nii');
for i = 1 + width : sx+width
     for j = 1 + width : sy+width
         for k = 1 + height : sz + height
             ROI = TempImg(i-width:i+width, j-width:j+width, k-height:k+height);
             [count, x] = imhist(ROI,256) ;
%              stem(x,count);
             g = round(ROI(width+1,width+1,height+1)*255);
             ProImg(i-width, j-width, k-height) = count(g+1)/sum(count(1:end));             
         end
     end
end
nii_Img = make_nii((ProImg).*Mask);
save_nii(nii_Img,'D:\ImageRegistration\RegistrationMatlab\Data\Temp\CT_1.nii');
