function plotWith3SigmaBorders( time, mean, sigma, color, name)
%plotWith3SigmaBorders
%   

plot(time,mean, 'Color', color, 'DisplayName', name, 'LineWidth', 1.5);
hold on;
plot(time, mean-3*sigma, '--', 'Color', color, 'DisplayName', strcat(name,' 3sig'));
hold on;
plot(time, mean+3*sigma, '--', 'Color', color, 'DisplayName', strcat(name,' 3sig'));

end

