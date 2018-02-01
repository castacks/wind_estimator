clear();
%delete(findall(0,'Type','figure'));

%% Load datalog
[CEKF_states, CEKF_var, localPosition] = CEKFWindFromPX4Log('2017-11-22/dflogs/flight7-dubins_trochoid_ws05/2017-11-22 16-19-41.mat',400,1);

localPosition = localPosition(100:end,:);
CEKF_states = CEKF_states(100:end,:);
CEKF_var = CEKF_var(100:end,:);

%% Set parameters
l = 66.0342;
sigma_f = 1.6508;

l_E = 180;
sigma_f_E = 1.85;
sigma_n_E = 0.1;

l_N = l_E;
sigma_f_N = sigma_f_E;
sigma_n_N = sigma_n_E;

l_U = 2.137956813081712e+02;
sigma_f_U = 0.353143217690198;
sigma_n_U = 0.137248827677595;

sigma_n = 0.1;
sigma_n_E2 = diag(CEKF_var(:,1)); sigma_n_E2 = sigma_n_E*eye(size(CEKF_states,1));
sigma_n_N2 = diag(CEKF_var(:,2)); sigma_n_N2 = sigma_n_N*eye(size(CEKF_states,1));
sigma_n_U2 = diag(CEKF_var(:,3)); %sigma_n_U2 = sigma_n_U*eye(size(CEKF_states,1));

%l = 50;
%sigma_f = 0.1;

%% Plot Kernel function
r = 0:0.1:200;
r = r';
for i = 1:1:size(r,1)
    k(i,1) = SquaredExponentialKernel([0,0,0], [0,0,r(i)], l, sigma_f);
end
figure();
plot(r,k);
xlabel('distance r in m');
ylabel('Covariance k(r)');


%% Setup sample grid
[x,y,z] = meshgrid(-100:25:150, -100:25:150, 40:4:60);

%% Compute wind field
% disp('1/3 Calculating x-component of wind field');
% u = WindFieldGPRegressionMeshgrid(localPosition, goyalStates(:,1), x,y,z, l,sigma_f,sigma_n);
% disp('2/3 Calculating y-component of wind field');
% v = WindFieldGPRegressionMeshgrid(localPosition, goyalStates(:,2), x,y,z, l,sigma_f,sigma_n);
% disp('3/3 Calculating z-component of wind field');
% w = WindFieldGPRegressionMeshgrid(localPosition, goyalStates(:,3), x,y,z, l,sigma_f,sigma_n);

q = meshgrid2vec(x,y,z)';
disp('1/3 Calculating x-component of wind field');
y_u = WindFieldGPRegression(localPosition, CEKF_states(:,1), q, l_E, sigma_f_E, sigma_n_E2)';
disp('2/3 Calculating y-component of wind field');
y_v = WindFieldGPRegression(localPosition, CEKF_states(:,2), q, l_N, sigma_f_N, sigma_n_N2)';
disp('3/3 Calculating z-component of wind field');
y_w = WindFieldGPRegression(localPosition, CEKF_states(:,3), q, l_U, sigma_f_U, sigma_n_U2)';

u = scalar2meshgrid(y_u, x);
v = scalar2meshgrid(y_v, x);
w = scalar2meshgrid(y_w, x);

%% Plot vector field
figure();
%plot3(localPosition(:,1), localPosition(:,2), localPosition(:,3));
hold on;
% for i = 1:1:size(goyalStates,1)
%     quiver3(localPosition(i,1),localPosition(i,2),localPosition(i,3), goyalStates(i,1),goyalStates(i,2),goyalStates(i,3), 'r');
% end
[cx, cy, cz] = meshgrid(linspace(-100, 150, 7), linspace(-100,150,7), linspace(40, 60, 3));

subplot(2,2,1)
quiver3(x,y,z, u,v,w, 'r');
xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

subplot(2,2,2);
cones = coneplot(x,y,z, u,v,w, cx,cy,cz, 1);
set(cones,'FaceColor','red','EdgeColor','none');
camlight right; lighting phong;
xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

subplot(2,2,3);
streamslice(x,y,z, u,v,w, 0,0,45);
xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

subplot(2,2,4);
[s1x,s1y,s1z] = meshgrid(-100:50:150, 150, 40:5:60);
[s2x,s2y,s2z] = meshgrid(-100, -100:50:100, 40:5:60);
plot3(s1x(:), s1y(:), s1z(:), 'bo', 'MarkerFaceColor', 'b');
hold on;
plot3(s2x(:), s2y(:), s2z(:), 'bo', 'MarkerFaceColor', 'b');
streamline(x,y,z, u,v,w, s1x,s1y,s1z);
streamline(x,y,z, u,v,w, s2x,s2y,s2z);

xlabel('East [m]');
ylabel('North [m]');
zlabel('Altitude [m]');

grid on; box on;