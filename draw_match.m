function [img_match] = draw_match(im1,im2,match1,match2)
%UNTITLED8 Summary of this function goes here
%   Detailed explanation goes here
[x,y] = size(im1);
img_match = zeros(2*x,y);
img_match(1:x,:) = im1;
img_match((x+1):2*x,:) = im2;
figure;
imshow(uint8(img_match));
for i=1:length(match1)
    hold on;
%      if inliersIndex(i)>0
    plot(match1(i,1),match1(i,2),'r+');
    plot(match2(i,1),x+match2(i,2),'r+');
    S=[match1(i,1),match2(i,1)]; 
    T=[match1(i,2),x+match2(i,2)];
    line(S,T);
%      end
end

end

