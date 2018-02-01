function [CEKF_states, CEKF_var, localPosition] = CEKFWindFromPX4Log(filename, timeMin, frequency)
%CEKFWindFromPX4Log 
%   Runs CEKF wind estimation with PX4 dataflash log and returns results in
%   ENU
datalog = load(filename);

%% Extract and normalize time --> timeReferencePoint t=0
[timeReferencePoint, time] = loadTimeFromPX4(datalog,timeMin,frequency);
maxIndex = size(time,1);

%% interpolate data


groundspeed =   loadDataFromPX4(datalog.GPS(:,2), datalog.GPS(:,11), timeReferencePoint, time);
airspeed =      loadDataFromPX4(datalog.ARSP(:,2), datalog.ARSP(:,3), timeReferencePoint, time);
diffPressure =  loadDataFromPX4(datalog.ARSP(:,2), datalog.ARSP(:,4), timeReferencePoint, time);
attitude_q =    loadDataFromPX4(datalog.AHR2(:,2), datalog.AHR2(:,9:12), timeReferencePoint, time);
position =      loadDataFromPX4(datalog.GPS(:,2), datalog.GPS(:,8:10), timeReferencePoint, time);

localPositionOrigin = position(1,:);
localPosition(:,1:2) = LatLonToMetric(position(:,1:2), localPositionOrigin(1,1:2)); %North,East,Up
% Swap North and East
temp = localPosition(:,2);
localPosition(:,2) = localPosition(:,1);
localPosition(:,1) = temp;
% Set altitute origin
localPosition(:,3) = position(:,3) - localPositionOrigin(1,3);

%plot(raw_groundspeed(:,1),raw_groundspeed(:,2), groundspeed(:,1), groundspeed(:,2));

Vg_g = zeros(maxIndex,3);
Va_g = zeros(maxIndex,3);
for i = 1:size(groundspeed(:,1))
    R_f2g = transpose(getR_g2f(attitude_q(i,1:4)));
    Vg_g(i,:) = transpose(R_f2g*[groundspeed(i); 0; 0]);
    Va_g(i,:) = transpose(R_f2g*[airspeed(i,1); 0; 0]);
end

%================================================================================
%================================================================================
%% run wind estimation

% Goyal Filter:
[CEKF_states,CEKF_var] = CEKF(time, Vg_g, diffPressure(:,1), attitude_q(:,1:4));

% Swap east and north component for ENU
temp = CEKF_states(:,1);
CEKF_states(:,1) = CEKF_states(:,2);
CEKF_states(:,2) = temp;

temp = CEKF_var(:,1);
CEKF_var(:,1) = CEKF_var(:,2);
CEKF_var(:,2) = temp;


end

