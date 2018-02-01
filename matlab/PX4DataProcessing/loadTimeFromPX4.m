function [timeReferencePoint,time,timeMax] = loadTimeFromPX4(datalog, timeMin, frequency)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

%% Extract groundspeed
px4Time = datalog.GPS(:,2);
% Set the time reference
timeReferencePoint = px4Time(1);
normTime = (px4Time - timeReferencePoint)/1e6;
timeMax = normTime(end);
time = transpose(timeMin : 1/frequency : timeMax);
end

