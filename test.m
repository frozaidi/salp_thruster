% Load CSV data
data = readtable(['CSVFiles/ThrustTestsFinal/3cam_170_pi8.csv']);  % Replace with actual filename
time = data{:,1}./1000;       % Convert ms to seconds
thrust = data{:,2};

% --- Step 1: Find peaks to define cycles ---
newtons = thrust./14500.*9.81.*1; % in mN

peak_max = max(newtons);
peak_threshold = 0.95;

[~, peakIdx] = findpeaks(newtons, ...
    'MinPeakProminence', peak_max*peak_threshold, ...
    'MinPeakDistance',0.5);

% --- Step 2: Extract individual cycles ---
nCycles = length(peakIdx) - 1;
nInterpPoints = 500;
allCycles = zeros(nCycles, nInterpPoints);

for i = 1:1:nCycles
    idxStart = peakIdx(i);
    idxEnd = peakIdx(i+1);
    
    cycleThrust = thrust(idxStart:idxEnd);
    cycleTime = time(idxStart:idxEnd);
    
    % Normalize time to 0–1 range
    tNorm = linspace(0, 1, length(cycleTime));
    tInterp = linspace(0, 1, nInterpPoints);
    
    % Interpolate thrust
    allCycles(i, :) = interp1(tNorm, cycleThrust, tInterp, 'linear');
end

% --- Step 3: Average all interpolated cycles ---
meanCycle = mean(allCycles, 1);

% Shift the mean cycle by +0.25 phase
shiftAmount = round(0.25 * nInterpPoints);
meanCycleShifted = circshift(meanCycle, [0, shiftAmount]);
allCyclesShifted = circshift(allCycles, [0, shiftAmount]);


% --- Step 4: Plot ---
figure;
hold on;
plot(linspace(0, 1, nInterpPoints), allCyclesShifted', 'Color', [0.85 0.85 0.85]);
plot(linspace(0, 1, nInterpPoints), meanCycleShifted, 'r', 'LineWidth', 2);
xlabel('Shifted Phase (0–1)');
ylabel('Thrust (ADC Value)');
title('Mean Thrust Cycle Shifted by +0.25 Phase');
legend('Shifted Cycles', 'Shifted Mean Cycle');
grid on;

