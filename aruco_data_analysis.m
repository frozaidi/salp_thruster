filename = ['processed_csv/ClockwiseRotation.csv'];
data = readtable(filename); % Replace with your filename

t = data{:,1};
x = data{:,2};
x_rem = rmoutliers(x);
y = data{:,3};
y_rem = rmoutliers(y);
z = data{:,4};

yaw = data{:,7};
yaw_rem = movmedian(yaw,3);

x_loess = smooth(x, 0.2, 'loess'); % 0.1 = span fraction
y_loess = smooth(y, 0.2, 'loess');
z_loess = smooth(z, 0.2, 'loess');

x_diff = diff(x_loess).*60;
y_diff = diff(y_loess).*60;

N = 100; % adjust for how often arrows appear
idx = 1:N:length(x_loess)-1;

vel = sqrt(x_diff.^2+y_diff.^2);

time_data = (t-t(1))./60;

% plot(time_data(2:end),vel)

plot(x, y,'.'); hold on;
plot(x_loess,y_loess)
quiver(x_loess(idx), y_loess(idx),x_diff(idx)/10,y_diff(idx)/5, 0, 'r', 'MaxHeadSize', 3)


% Filter parameters
% alpha = 0.04;   % smoothing factor
% 
% % Initialize
% x_filt = zeros(size(x));
% y_filt = zeros(size(y));
% x_filt(1) = x(1);
% y_filt(1) = y(1);
% 
% % Apply EWMA to both
% for k = 2:length(t)
%     x_filt(k) = alpha*x(k) + (1-alpha)*x_filt(k-1);
%     y_filt(k) = alpha*y(k) + (1-alpha)*y_filt(k-1);
% end
% 
% % Plot comparison
% figure;
% plot(x, y, 'b.', 'DisplayName','Raw trajectory'); hold on;
% plot(x_filt, y_filt, 'r-', 'LineWidth',2, 'DisplayName','EWMA filtered');
% legend;
% axis equal;