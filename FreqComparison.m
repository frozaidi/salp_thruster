% clear;
filename_check = 'CSVFiles/ThrustTestsFinal/3cam_170_pi8.csv';
filename_nocheck = 'CSVFiles/ThrustTestsFinal/3cam_170_0pi.csv';

[frequency_check, max_thrust_check, impulse_per_cycle_check, ...
    coord_combine_check, times_period_check, thr_avg_check] = ForceResponseAnalyze(filename_check);

[frequency_nocheck, max_thrust_nocheck, impulse_per_cycle_nocheck, ...
    coord_combine_nocheck, times_period_nocheck, thr_avg_nocheck] = ForceResponseAnalyze(filename_nocheck);

f3 = figure(3);
hax3 = axes;
f3.Position = [2000 100 900 700];
hold on;
ylim([-1.2 1.4])
plot(hax3,times_period_check, thr_avg_check, 'Color',"#D73F09", 'LineWidth', 4);           % average line
plot(hax3,times_period_nocheck, thr_avg_nocheck, 'Color',"#000000", 'LineWidth', 4);           % average line
% plot(hax3,sim_time, sim_thrust, 'Color',"#000000", 'LineWidth', 4);           % average line


y0 = yline(0,'LineWidth',3,'Color','k',Layer='bottom');
legend(hax3,{'\pi/8 Phase Offset', '0 Phase Offset',''})
xlabel('Time(s)');
ylabel('Force (N)');
grid on;
set(hax3,'FontWeight','bold','FontSize',30,'FontName','Times')
set(f3,'Color','white')
hold off