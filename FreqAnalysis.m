close all;
% data = readtable('CSVFiles/LoadCellTest500Hz.csv'); % Replace with your filename
data = readtable('Forward_170_styro.csv'); % Replace with your filename



time = data{:,1};
time_seconds = time./1000;
times = time./1000-27.4;
thrust = data{:,2};

timestep = time_seconds(2)-time_seconds(1);

newtons = thrust./14500.*9.81.*1; % in mN

peak_max = max(newtons);
peak_threshold = 0.85;

[peakVals, peakLocs] = findpeaks(newtons, time_seconds, ...
    'MinPeakProminence', peak_max*peak_threshold, ...
    'MinPeakDistance', 0.5);

peakIntervals = diff(peakLocs); % Time between peaks
meanPeriod = mean(peakIntervals); % Average period
frequency = 1 / meanPeriod; % Hz

period_itr = floor(meanPeriod/timestep);
init_peak_idx = find(time_seconds==peakLocs(1));
init_start_idx = init_peak_idx - floor(period_itr/4);
init_end_idx = init_start_idx + period_itr*numel(peakLocs)-1;

times_adj = times(init_start_idx:init_end_idx);
thrust_adj = newtons(init_start_idx:init_end_idx);
times_period = linspace(0,meanPeriod,period_itr);

thrust_reshape = reshape(thrust_adj,[],numel(peakLocs));
% 
% times_adj = times(5978:15602);
% % thrust_adj = thrust(5978:15602);
% 
% thrust_adj = newtons(5978:15602);
% 
% times_period = linspace(0,1,385);
% 
% thrust_reshape = reshape(thrust_adj,[],25);
thr_avg = mean(thrust_reshape,2);
thr_std = std(thrust_reshape,0,2);

thr_upp = (thr_avg + thr_std);
thr_low = (thr_avg - thr_std);

times_period = linspace(0,meanPeriod,period_itr)';

coord_upp = [times_period,thr_upp];
coord_low = [times_period,thr_low];

coord_combine = [coord_upp;flipud(coord_low)];

plot(time_seconds, newtons); hold on;
plot(peakLocs, peakVals, 'ro'); % Highlight peaks
title(['Estimated Frequency: ', num2str(frequency, '%.2f'), ' Hz']);
xlabel('Time (s)'); ylabel('Thrust');

f3 = figure(3);
hax3 = axes;
f3.Position = [2000 100 900 700];
hold on;
% fill([times_period, flipud(times_period)], [thr_upp, flipud(thr_low)], [0.8 0.8 1], ...
%     'EdgeColor', 'none', 'FaceAlpha', 0.4);  % shaded region

fill(hax3,coord_combine(:,1),coord_combine(:,2),[0.8,0.8,0.8],'EdgeColor','none')
plot(hax3,times_period, thr_avg, 'Color',"#D73F09", 'LineWidth', 2);           % average line
y0 = yline(0,'LineWidth',3,'Color','k',Layer='bottom');
% plot(times_period, thr_low, 'b', 'LineWidth', 2);
xlabel('Time(s)');
ylabel('Force (N)');
% title('Average with Shaded Standard Deviation');
grid on;
%     ylim([0,0.00025])
set(hax3,'FontWeight','bold','FontSize',15,'FontName','OpenSans')
set(f3,'Color','white')
hold off

[f4,ax4] = makeFig('Time(s)','Standard Deviation (N)',15);
plot(ax4,times_period,thr_std,'Color',"#D73F09", 'LineWidth', 2)
grid(ax4,"on")
ylim([0,2])
hold(ax4,"off")