clear();
delete(findall(0,'Type','figure'));

%% Load datalog
%% 2017-11-22 Flight 1
%datalog = load('2017-11-22/dflogs/flight1-tuning/2017-11-22 09-59-57.mat');
%timeMin = 1820;
%% 2017-11-22 Flight 2
%datalog = load('2017-11-22/dflogs/flight2-trochoid_ws08/2017-11-22 11-33-56.mat');
%timeMin = 130;
%% 2017-11-22 Flight 3
%datalog = load('2017-11-22/dflogs/flight3-dubins_ws08_incomplete/2017-11-22 12-46-51.mat');
%timeMin = 80;
%% 2017-11-22 Flight 4
%datalog = load('2017-11-22/dflogs/flight4-trochoid_ws06_dubins_ws08_incomplete/2017-11-22 12-58-53.mat');
%timeMin = 110;
%% 2017-11-22 Flight 5
%datalog = load('2017-11-22/dflogs/flight5-trochoid_ws05_dubins_ws04_incomplete/2017-11-22 14-49-46.mat');
%timeMin = 220;
%% 2017-11-22 Flight 6
%datalog = load('2017-11-22/dflogs/flight6-dubins-lost-mavctrl-log/2017-11-22 16-08-09.mat');
%timeMin = 150;
%% 2017-11-22 Flight 7
% datalog = load('2017-11-22/dflogs/flight7-dubins_trochoid_ws05/2017-11-22 16-19-41.mat');
% timeMin = 400;
%% 2018-01-25 Flight 2
datalog = load('2018-01-25/Flight2/dflogs/2018-01-25 12-20-28.mat');
timeMin = 41;

frequency = 10;

%% Extract and normalize time --> timeReferencePoint t=0
[timeReferencePoint, time] = loadTimeFromPX4(datalog,timeMin,frequency);
maxIndex = size(time,1);
timeMax = time(end);

%% interpolate data
groundspeed =   loadDataFromPX4(datalog.GPS(:,2), datalog.GPS(:,11), timeReferencePoint, time);
airspeed =      loadDataFromPX4(datalog.ARSP(:,2), datalog.ARSP(:,3), timeReferencePoint, time);
diffPressure =  loadDataFromPX4(datalog.ARSP(:,2), datalog.ARSP(:,4), timeReferencePoint, time);
attitude_q =    loadDataFromPX4(datalog.AHR2(:,2), datalog.AHR2(:,9:12), timeReferencePoint, time);
position =      loadDataFromPX4(datalog.GPS(:,2), datalog.GPS(:,8:10), timeReferencePoint, time);

localPositionOrigin = position(1,:);
localPosition(:,1:2) = LatLonToMetric(position(:,1:2), localPositionOrigin(1,1:2));
localPosition(:,3) = position(:,3) - localPositionOrigin(1,3);

%plot(raw_groundspeed(:,1),raw_groundspeed(:,2), groundspeed(:,1), groundspeed(:,2));

Vg_g = zeros(maxIndex,3);
Va_g = zeros(maxIndex,3);
for i = 1:size(groundspeed(:,1))
    R_f2g = transpose(getR_g2f(attitude_q(i,1:4)));
    Vg_g(i,:) = transpose(R_f2g*[groundspeed(i,1); 0; 0]);
    Va_g(i,:) = transpose(R_f2g*[airspeed(i,1); 0; 0]);
end

%================================================================================
%================================================================================
%% run filter
% Easy approach:
Vw_g = Vg_g-Va_g;
%Vw_g(1,:) = [ -3 4 0];

% Calibrating Extended Kalman Filter:
[cekfStates,cekfVariances] = CEKF(time, Vg_g, diffPressure(:,1), attitude_q(:,1:4));

% Simple Wind Kalman Filter:
[swkfStates, swkfVariances] = SimpleWindKF(time, Vg_g, Va_g);

% Moving Average Filter over 3min
N = frequency*60*3;
movAvStates = [MovingAverageFilter(Vw_g(:,1),N) MovingAverageFilter(Vw_g(:,2),N) MovingAverageFilter(Vw_g(:,3),N)];


%==========================================================================
%===========================================================
%% Plots
%==========================================================================
%===========================================================
%Colors
clRed = [255, 121, 45]/255;
clBlue = [44, 105, 179]/255;
clGreen = [32, 179, 138]/255;
clYellow = [255, 173, 45]/255;

cekfDisplayName = 'CEKF';
swkfDisplayName = 'SWKF';
mafDisplayName = 'MAF';
umDisplayName = 'UM';
%% Plot Velocities
figure(1);
w = -sqrt(12.5);
subplot(3,1,1);
plot( time, groundspeed(:,1), time, cekfStates(:,5), time, airspeed(:,1));
xlabel('time[s]');
xlim([timeMin timeMax]);
ylabel('velocity[m/s]'); 
title('Aircraft velocities','FontSize',12);
legend('Groundspeed','est. Airspeed', 'onboard Airspeed');
grid on;

%% Plot wind
subplot(3,1,2);
plotWith3SigmaBorders(time, cekfStates(:,1), sqrt(cekfVariances(:,1)), 'b', 'Wind in North direction');
plotWith3SigmaBorders(time, cekfStates(:,2), sqrt(cekfVariances(:,2)), 'r', 'Wind in East direction');
plotWith3SigmaBorders(time, cekfStates(:,3), sqrt(cekfVariances(:,3)), 'g', 'Vertical Wind');
xlabel('time[s]');
xlim([timeMin timeMax]);
ylabel('velocity[m/s]'); 
legend('show');
title('Wind velocities','FontSize',12);
grid on;

%% Plot eta
subplot(3,1,3);
%figure();
plotWith3SigmaBorders(time, cekfStates(:,4), sqrt(cekfVariances(:,4)), 'r', 'estimated eta');
ylabel('eta[kg/m^3]')
xlabel('time[s]');
xlim([timeMin timeMax]);
title('Airspeed calibration factor')
legend('show');
grid on;

%% Plot eta in separate figure
    fig = figure(2);
    fig.Resize = 'off';
    %vectorFig.InvertHardcopy = 'on';
    fig.PaperPositionMode = 'manual';
    fig.PaperUnits = 'inches';
    fig.Units = 'inches';
    width = 7.5;
    height = 7/3;
    fig.PaperPosition = [0, 0, width, height];
    fig.PaperSize = [width, height];
    fig.Position = [0, 0, width, height];
    
    onboardEta = diffPressure(:,1) ./ (airspeed(:,1).*airspeed(:,1));
    
    hold on;
    plot(time, onboardEta, 'Color', clRed, 'DisplayName', 'on-board');
    plotWith3SigmaBorders(time, cekfStates(:,4), sqrt(cekfVariances(:,4)), clBlue, 'CEKF');
    ylabel('eta [kg/$m^3$]')
    xlabel('time[s]');
    xlim([timeMin timeMax]);
    ylim([0.49 0.75]);
    lgd = legend('show');
    lgd.Box = 'off';
    lgd.Interpreter = 'LaTeX';
    title('Airspeed calibration factor')
    grid on;
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
    
    %% Plot airspeed in separate figure
    fig = figure(3);
    fig.Resize = 'off';
    %vectorFig.InvertHardcopy = 'on';
    fig.PaperPositionMode = 'manual';
    fig.PaperUnits = 'inches';
    fig.Units = 'inches';
    width = 7.5;
    height = 7/2;
    fig.PaperPosition = [0, 0, width, height];
    fig.PaperSize = [width, height];
    fig.Position = [0, 0, width, height];
    
    plot(time, cekfStates(:,5), 'LineWidth',1.5, 'Color', clRed);hold on;
    plot(time, airspeed(:,1),'LineWidth',0.5, 'Color', clBlue);
    
    ylabel('Velocity [m/s]');
    ylim([10 20]);
    xlabel('time[s]');
    xlim([timeMin timeMax]);
    title('Airspeeds')
    lgd = legend('CEFK','On-board');
    lgd.Box = 'off';
    lgd.Interpreter = 'LaTeX';
    grid on;
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
%%=============================================================================================
%% Wind Overview figure
%%=============================================================================================
    % Plot Wind magnitude
    fig = figure(4);
    fig.Resize = 'off';
    %vectorFig.InvertHardcopy = 'on';
    fig.PaperPositionMode = 'manual';
    fig.PaperUnits = 'inches';
    fig.Units = 'inches';
    width = 7.5;
    height = 7;
    fig.PaperPosition = [0, 0, width, height];
    fig.PaperSize = [width, height];
    fig.Position = [0, 0, width, height];
    
    subplot(2,1,1);
    cekfMagnitude = zeros(maxIndex,1);
    swkfMagnitude = zeros(maxIndex,1);
    movAvMagnitude = zeros(maxIndex,1);
    unfilteredMagnitude = zeros(maxIndex,1);
    for i = 1:maxIndex
        cekfMagnitude(i) = norm(transpose(cekfStates(i,1:3)));
        swkfMagnitude(i) = norm(transpose(swkfStates(i,4:6)));
        movAvMagnitude(i) = norm(transpose(movAvStates(i,1:3)));
        unfilteredMagnitude(i) = norm(transpose(Vw_g(i,1:3)));
    end
    %plot(time, unfilteredMagnitude, ':'); hold on;
    plot(time, movAvMagnitude, 'LineWidth', 1.5, 'Color', clGreen); hold on;
    plot(time, swkfMagnitude, 'LineWidth', 1.5, 'Color', clBlue);
    plot(time, cekfMagnitude, 'LineWidth', 1.5, 'Color', clRed);
    ylabel('Velocity[m/s]');
    xlabel('time[s]');
    xlim([timeMin timeMax]);
    ylim([2 4.5]);
    title('Wind magnitude');
    lgd = legend( mafDisplayName, swkfDisplayName, cekfDisplayName);
    lgd.Box = 'off';
    lgd.Interpreter = 'LaTeX';
    grid on;

    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
    
    % Plot Wind Heading
    subplot(2,1,2);
    cekfHeadings = getHeadingsFromVector(cekfStates(:,1),cekfStates(:,2));
    swkfHeadings = getHeadingsFromVector(swkfStates(:,4),swkfStates(:,5));
    movAvHeadings = getHeadingsFromVector(movAvStates(:,1),movAvStates(:,2));
    unfilteredHeadings = getHeadingsFromVector(Vw_g(:,1),Vw_g(:,2));

    %plot( time, unfilteredHeadings, ':'); hold on;
    plot(time, movAvHeadings,'-','LineWidth',1.5,'Color',clGreen); hold on;
    plot(time, swkfHeadings,'-','LineWidth',1.5,'Color',clBlue); hold on;
    plot(time, cekfHeadings,'-','LineWidth',1.5,'Color',clRed); 
    title('Wind direction FROM');
    ylabel('direction[deg]');
    xlabel('time[s]');
    xlim([timeMin timeMax]);
    ylim([290 330]);
    %legend( mafDisplayName, swkfDisplayName, goyalDisplayName);
    grid on;
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
%%=============================================================================================
%% Wind vector component figure
%%=============================================================================================
    %Setup figure
    vectorFig = figure(5);
    vectorFig.Resize = 'off';
    %vectorFig.InvertHardcopy = 'on';
    vectorFig.PaperPositionMode = 'manual';
    vectorFig.PaperUnits = 'inches';
    vectorFig.Units = 'inches';
    width = 7.5;
    height = 7;
    vectorFig.PaperPosition = [0, 0, width, height];
    vectorFig.PaperSize = [width, height];
    vectorFig.Position = [0, 0, width, height];
    
    %Plot North component
    subplot(3,1,1);
    
    %plot(time, Vw_g(:,1), ':', 'DisplayName', umDisplayName, 'Color', clBlue);hold on;
    plot(time, movAvStates(:,1), ':', 'DisplayName', mafDisplayName, 'LineWidth', 2, 'Color', clRed); hold on;
    %plotWith3SigmaBorders(time, swkfStates(:,4), sqrt(swkfVariances(:,4)), clBlue, swkfDisplayName);
    plotWith3SigmaBorders(time, cekfStates(:,1), sqrt(cekfVariances(:,1)), clBlue, cekfDisplayName);
    
    grid on;
    title('North wind');
    lgd = legend('show');
    lgd.Box = 'off';
    lgd.Interpreter = 'LaTeX';
    xlabel('time [s]');
    xlim([timeMin timeMax]);
    %ylim([-9 3]);
    ylim([-4 -1]);
    ylabel('Velocity [m/s]');
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
    
    %Plot East component
    subplot(3,1,2);
    
    %plot(time, Vw_g(:,2), ':', 'DisplayName', umDisplayName, 'Color', clBlue); hold on;
    plot(time, movAvStates(:,2), ':', 'DisplayName', mafDisplayName, 'LineWidth', 2, 'Color', clRed); hold on;
    %plotWith3SigmaBorders(time, swkfStates(:,5), sqrt(swkfVariances(:,5)), clBlue, swkfDisplayName);
    plotWith3SigmaBorders(time, cekfStates(:,2), sqrt(cekfVariances(:,2)), clBlue, cekfDisplayName);
    
    grid on;
    title('East wind');
    xlabel('time [s]');
    xlim([timeMin timeMax]);
    %ylim([-2 7]);
    ylim([1 4]);
    ylabel('Velocity [m/s]');
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
    
    %Plot vertical component
    subplot(3,1,3);
    
    %plot(time, Vw_g(:,3), ':','DisplayName', umDisplayName, 'Color', clBlue); hold on;
    plot(time, movAvStates(:,3), ':', 'DisplayName', mafDisplayName, 'LineWidth', 2, 'Color', clRed); hold on;
    %plotWith3SigmaBorders(time, swkfStates(:,6), sqrt(swkfVariances(:,6)), clBlue, swkfDisplayName);
    plotWith3SigmaBorders(time, cekfStates(:,3), sqrt(cekfVariances(:,3)), clBlue, cekfDisplayName);
    
    grid on;
    title('Vertical wind');
    xlabel('time [s]');
    xlim([timeMin timeMax]);
    %ylim([-2.0 2]);
    ylim([-1.5 1.5]);
    ylabel('Velocity [m/s]');
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
%%=============================================================================================
%% Plot flight path information
%%=============================================================================================
    %Setup figure
    vectorFig = figure(6);
    vectorFig.Resize = 'off';
    %vectorFig.InvertHardcopy = 'on';
    vectorFig.PaperPositionMode = 'manual';
    vectorFig.PaperUnits = 'inches';
    vectorFig.Units = 'inches';
    width = 7.5;
    height = 7*2/3;
    vectorFig.PaperPosition = [0, 0, width, height];
    vectorFig.PaperSize = [width, height];
    vectorFig.Position = [0, 0, width, height];
    
    %Plot altitude
    subplot(2,1,1);
    plot(time, position(:,3), 'Color', clBlue);
    grid on;
    title('Altitude ASL');
    xlabel('time [s]');
    xlim([timeMin timeMax]);
    ylim([330 420]);
    ylabel('altitude ASL [m]');
    
    ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
    
    %Plot velocities
    subplot(2,1,2);
    plot(time, groundspeed(:,1), 'Color', clBlue);hold on;
    plot(time, airspeed(:,1), 'Color', clRed);
    grid on;
    title('Aircraft velocities');
    xlabel('time [s]');
    xlim([timeMin timeMax]);
    ylim([0 25]);
    ylabel('velocity [m/s]');
    lgd = legend('Groundspeed','Airspeed');
    lgd.Box = 'off';
    lgd.Interpreter = 'LaTeX';
    
        ax = gca;
    ax.FontName = 'LaTeX';
    ax.Title.Interpreter = 'LaTeX';
    ax.XLabel.Interpreter = 'LaTeX';
    ax.YLabel.Interpreter = 'LaTeX';
    ax.Box = 'off';
    ax.FontSize = 11;
    
%==========================================================================
%===========================================================
%% Export data
%==========================================================================
%===========================================================
% pixhawkTime = time*1e6 + timeReferencePoint;
% outputMatrix = [pixhawkTime time position cekfStates cekfMagnitude cekfHeadings swkfStates swkfMagnitude swkfHeadings movAvStates movAvMagnitude movAvHeadings Vw_g];
% csvwrite("flight7_winddata.csv",outputMatrix);

%==========================================================================
%===========================================================
%% Import wind_estimator data
%==========================================================================
%===========================================================
[estimator_time estimator_cekf] = loadWindEstimatorData('2018-01-25/Flight2/wind_estimator_2018_01_25_12_22_03.dat');

estimator_time = estimator_time + 94.8;

estimator_cekf_Headings = getHeadingsFromVector(estimator_cekf(:,1),estimator_cekf(:,2));
estimator_cekf_Magnitude = zeros(size(estimator_time,1),1);

for i = 1:size(estimator_time,1)
    estimator_cekf_Magnitude(i) = norm(transpose(estimator_cekf(i,1:3)));
end

%%
figure();
subplot(2,1,1);
plot(estimator_time, estimator_cekf_Magnitude, time, cekfMagnitude);
legend('Real-time', 'Offline');

subplot(2,1,2);
plot(estimator_time, estimator_cekf_Headings, time, cekfHeadings);
legend('Real-time', 'Offline');
