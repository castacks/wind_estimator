function [ headings ] = getHeadingsFromVector( north, east )
%plots the headings

%% Calculate the headings in degree from vector components
headings(:,1) = rad2deg(atan2(east(:,1),north(:,1)));

headings(:,1) = headings(:,1) - 180;

%% Eliminate negative values
for i = 1:size(north,1)
   if headings(i)<0
      headings(i) = headings(i)+360; 
   end
end

end

