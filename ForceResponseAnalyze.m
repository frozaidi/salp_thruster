function [frequency, max_thrust, impulse_per_cycle, ...
    coord_combine, times_period, thr_avg] = ForceResponseAnalyze(filename)
data = readtable(filename); % Replace with your filename
% data = readtable('Forward_170_styro_2.csv'); % Replace with your filename
freq = 100*.229/30;

cam_num = regexp(filename,'\d+cam','match');
phase_offset = regexp(filename,'\d*+pi+\d*','match');
rot_num = regexp(filename,'_(\d{2,3})','tokens');
rot_num = str2double(rot_num{1});

time = data{:,1};
time_seconds = time./1000;
times = time./1000;
thrust = data{:,2};

timestep = time_seconds(2)-time_seconds(1);

newtons = thrust./14500.*9.81.*1; % in mN

peak_max = max(newtons);
peak_threshold = 0.85;
% rot_num = 100;
delay_num = 1/(rot_num*0.229/60*2)*.51;

[~, peakIdx] = findpeaks(newtons, ...
    'MinPeakProminence', peak_max*peak_threshold, ...
    'MinPeakDistance',delay_num*500);

% --- Step 2: Extract individual cycles ---
nCycles = length(peakIdx) - 1;
nInterpPoints = 500;
allCycles = zeros(nCycles, nInterpPoints);

for i = 1:1:nCycles
    idxStart = peakIdx(i);
    idxEnd = peakIdx(i+1);
    
    cycleThrust = newtons(idxStart:idxEnd);
    cycleTime = time(idxStart:idxEnd);
    
    % Normalize time to 0–1 range
    tNorm = linspace(0, 1, length(cycleTime));
    tInterp = linspace(0, 1, nInterpPoints);
    
    % Interpolate thrust
    allCycles(i, :) = interp1(tNorm, cycleThrust, tInterp, 'linear');
end

thrust_reshape = allCycles';

[peakVals, peakLocs] = findpeaks(newtons, time_seconds, ...
    'MinPeakProminence', peak_max*peak_threshold, ...
    'MinPeakDistance', delay_num);

peakIntervals = diff(peakLocs); % Time between peaks
meanPeriod = mean(peakIntervals); % Average period
frequency = 1 / meanPeriod; % Hz

period_itr = floor(meanPeriod/timestep);
thr_avg = mean(thrust_reshape,2);
thr_std = std(thrust_reshape,0,2);
shiftAmount = round(0.25 * nInterpPoints);
thr_avg = circshift(thr_avg, shiftAmount);
thr_std = circshift(thr_std, shiftAmount);

thr_upp = (thr_avg + thr_std);
thr_low = (thr_avg - thr_std);

times_period = linspace(0,meanPeriod,500)';

coord_upp = [times_period,thr_upp];
coord_low = [times_period,thr_low];

coord_combine = [coord_upp;flipud(coord_low)];

impulse_per_cycle = trapz(times_period,thr_avg);
max_thrust = max(peakVals);

end