function [point_track] = pointTracker(imageID,match1,match2)
for i = 1:size(match1,1)
    point_track(i) = pointTrack(imageID,[match1(i,:);match2(i,:)]);
end

end

