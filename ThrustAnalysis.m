T = readtable("LoadCellTest500Hz.csv");

times = T.(1);
times = times./1000-27.4;
data = T.(2);

newtons = data./14500.*9.81.*1; % in mN

st = 5500;

timestep = 0.051;
i_st = 5978;
i_en = 15863;

int_time = 0:timestep:times(end);

int_force = interp1(times, data, int_time, 'linear');

int_imp = zeros(size(int_force));

for i = 1:i_en-i_st
    int_imp(i) = 0.002*newtons(i+i_st)/1;

    if isnan(int_imp(i))
        int_imp(i) = 0;
    end
end

net = sum(int_imp); % in N.s
imp_cyc = net/26; % in N.s / cyc
net_pos = sum(int_imp(int_imp>0));
net_neg = sum(int_imp(int_imp<0));

% myVideo = VideoWriter('ThrustPlotAnimation','MPEG-4'); %open video file
%     myVideo.FrameRate = 60;  %can adjust this, 5 - 10 works well for me
%     myVideo.Quality = 100;
%     open(myVideo)

f = figure(1);
f.Position = [100 100 1500 800];
% f.Position = [100 100 900 700];


% plot(int_time,int_imp.*1000,'LineWidth',4, 'Color',"#D73F09");
% xlim([int_time(1) int_time(end)])
xlim([0 21])
xlabel("time (s)")
ylabel("force (mN)")
set(gca,'FontWeight','bold','FontSize',25,'FontName','OpenSans')
set(gcf,'Color','white')
% hold on
l = line(times(st:end),newtons(st:end), 'Color',"#D73F09",'LineWidth',2);

% for i = 1:numel(int_time)
%     set(l,'Xdata',int_time(1:i),'Ydata',int_imp(1:i).*1000)
%     drawnow
% %     frame = getframe(gcf);
% %     writeVideo(myVideo,frame);
%     % pause(0.1)
% end

% saveas(gcf,"ThrustTest",'png')
hold off
% close(myVideo)

