 function [corner,corner_count]= Harris_point_detection(im,mask_type,window_size)
%Harris point detection
%input: im image matrix; mask_type: Gradient, Prewitt or Sobel
%Analysis Window size:  Gaussian

 I=double(im);
Prewitt_mask = [-1 0 1; -1 0 1; -1 0 1]; % The Prewitt Mask 
Gradient_mask=[-1,0,1];
Sobel_mask=[-1 0 1;-2 0 2;-1 0 1];
row=size(I,1);
col=size(I,2);
if strcmpi(mask_type,'Prewitt')
    x_mask=Prewitt_mask;
    y_mask=x_mask';
end
if strcmpi(mask_type,'Gradient')
    x_mask=Gradient_mask;
    y_mask=x_mask';
end
if strcmpi(mask_type,'Sobel')
    x_mask=Sobel_mask;
    y_mask=x_mask';
end
% im_expand= padarray(im,window_size,window_size,both);
sigma=2;
Ix=imfilter(I,x_mask);
Ix2=Ix.^2;
Iy=imfilter(I,y_mask);
Iy2=Iy.^2;
Ixy=Ix.*Iy;
w = fspecial('gaussian',max(1,fix(window_size*sigma)), sigma);
A=imfilter(Ix2,w); 
B=imfilter(Iy2,w);
C=imfilter(Ixy,w); 
k=0.04;
RMax=0;
R=zeros(row,col);
 for h=1:row
     for w=1:col 
         M=[A(h,w) C(h,w);C(h,w) B(h,w)]; 
         R(h,w)=det(M) - k*(trace(M))^2; 
     end
 end
R_max=max(max(R));
 %Use the value Q*R_max as threshold, if the R value of a point if far smaller than
 %the R_max, we ignore it
Q=0.01;
R=(R>=Q*R_max).*R;
%Find the local maximum in eight neighbor area
corner=zeros(row,col);
corner_count=0;
for i=2:row-1
    for j=2:col-1
        if ((R(i,j)>=max(max(R(i-1:i+1,j-1:j+1))))&(R(i,j)>0))
            corner(i,j)=1;
            corner_count=corner_count+1;
        end
    end
end
newline;
fprintf('%d coners are detected', corner_count);
%  [x,y]=find(corner==1);
% imshow(im)
% hold on
% plot(y,x,'go')
% plot(corner)
%plot(corner,'o')

 



