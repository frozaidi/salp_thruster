frequency = [0.68
0.76
0.84
0.92
0.99
1.07
1.14
1.22
1.29];

peak_thrust = [0.04
0.19
0.44
0.36
0.35
0.45
0.75
0.97
1.02
];

impulse_per_cycle = [0.021
-0.019
0.059
0.069
0.079
0.076
0.086
0.086
0.070
];

[f1,ax1] = makeFig('Frequency (Hz)','Peak Thrust (N)',15);
colororder(["#D73F09","#000000"])
yyaxis(ax1,"left")
plot(ax1,frequency,peak_thrust,'Color',"#D73F09", 'LineWidth', 2)
grid(ax1,"on")
xlim(ax1,[min(frequency),max(frequency)])
ylim([-0.2,1.2])
yyaxis(ax1,"right")
ylim([-0.02,0.12])
plot(ax1,frequency,impulse_per_cycle,'Color',"k", 'LineWidth', 2)
ylabel("Impulse Per Cycle (Ns)")
hold(ax1,"off")