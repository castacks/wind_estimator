function [time cekfStates swkfStates mafStates] = loadWindEstimatorData(filename)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

data = readtable(filename);

%% Data comes in ENU, we want NEU
cekfStates = [data.cekf_wind_y data.cekf_wind_x data.cekf_wind_z data.cekf_eta data.cekf_airspeed_x data.cekf_airspeed_y data.cekf_airspeed_z];
swkfStates = [data.swkf_wind_y data.swkf_wind_x data.swkf_wind_z];
mafStates = [data.moving_average_wind_y data.moving_average_wind_x data.moving_average_wind_z];

time = [data.time];
end

