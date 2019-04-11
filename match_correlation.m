function [match1,match2,matchIndex] = match_correlation(im1,im2,cor1,cor2,cnt1,cnt2)
win_size=5; %check the different between a window of size (11*11)around a corner point
im1_ex = padarray(double(im1),[win_size win_size],0,'both');%expand the image
im2_ex = padarray(double(im2),[win_size win_size],0,'both');
[r1,c1]=find(cor1 == 1);
[r2,c2]=find(cor2 == 1);
cp1=[r1 c1]; % save the x y coordinates of the corner points 
cp2=[r2 c2];
temp_cp2 = zeros(size(cp1));
SSD=zeros(cnt1,cnt2);
n=1;
for i=1:cnt1
    x1 = cp1(i,1) + win_size;  % x coordinate of corner point in the expand im1
    y1 = cp1(i,2) + win_size; % y coordinate of corner point in the expand im1
    Win1 = im1_ex(x1-win_size:x1+win_size, y1-win_size:y1+win_size); % Build the window around the corner point in im1
    for j=1:cnt2
        x2 = cp2(j,1) + win_size; % x coordinate of corner point in the expand im2
        y2 = cp2(j,2) + win_size; % y coordinate of corner point in the expand im2
        Win2 = im2_ex(x2-win_size:x2+win_size,y2-win_size:y2+win_size);
        SSD(i,j)= sum(sum((Win1-Win2).^2));
    end
    MIN(i) = min(SSD(i,:));
    match_idx(i) = find(SSD(i,:)==MIN(i));
    if min(SSD(i,:))>100000
        temp_cp2(i,:) = 0; 
    else
        temp_cp2(i,:) = cp2(match_idx(i),:);
        matchIndex(n,1)=i;
        matchIndex(n,2)=match_idx(i);
         n=n+1;
    end
cnt=numel(temp_cp2,temp_cp2~=0)/2;
match1=zeros(cnt,2);
match2=zeros(cnt,2);
j=1;
for i=1:cnt1 
    if((temp_cp2(i,1)~=0)&&(cp1(i,1)~=0)) 
        match2(j,:)=temp_cp2(i,:);
        match1(j,:)=cp1(i,:);
        j=j+1;
    end
end


    


end

