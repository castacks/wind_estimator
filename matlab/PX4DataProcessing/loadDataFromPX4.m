function [data] = loadDataFromPX4(px4Time, px4Data, timeReferencePoint, time)
%loadDataFromPX4

%% Extract data
%% Normalize time
px4Time = (px4Time - timeReferencePoint)/1e6;

dataComponents = size(px4Data,2);
data = zeros(size(time,1),dataComponents);

for i = 1:dataComponents
   data(:,i) = interp1(px4Time, px4Data(:,i), time);
end

end

