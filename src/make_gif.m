clear
close all
clc

load ('trajectories/TestTrack.mat');
load ('trajectories/part1_track.mat');
load ('trajectories/part2_track.mat');

gif_filename = 'data/output.gif';
original_gif_delay = 0.1;
speedup_factor = 1.25;
gif_delay = original_gif_delay / speedup_factor;

for i=1:35:size(Y_mpc,1)-50
    plot(TestTrack.bl(1,:)',TestTrack.bl(2,:)','k','LineWidth',1.5);
    hold on
    plot(TestTrack.br(1,:)',TestTrack.br(2,:)','k','LineWidth',1.5);
    for j=1:length(Xobs)
        fill([Xobs{1,j}(:,1);Xobs{1,j}(1,1)],[Xobs{1,j}(:,2);Xobs{1,j}(1,2)],'r');
    end
    try
        h1 = plot(Y(1:i,1),Y(1:i,3),'g','LineWidth',1.5);
        plot(Y(i,1),Y(i,3),'go','MarkerFaceColor','g','LineWidth',1.5);
    catch
        h1 = plot(Y(1:end,1),Y(1:end,3),'g','LineWidth',1.5);
        plot(Y(end,1),Y(end,3),'go','MarkerFaceColor','g','LineWidth',1.5);
    end
    h2 = plot(Y_mpc(1:i,1),Y_mpc(1:i,3),'b','LineWidth',1.5);
    plot(Y_mpc(i,1),Y_mpc(i,3),'bo','MarkerFaceColor','b','LineWidth',1.5);

    hold off
    xlim([Y_mpc(i,1)-50,Y_mpc(i,1)+50]);
    ylim([Y_mpc(i,3)-50,Y_mpc(i,3)+50]);
    legend([h1 h2],'Reference Trajectory','MPC Trajectory','Position',[0.75,0.2,0.124,0.037],'FontSize',15);
    set(gcf,'Position',[0 0 1500 1000]);
    set(gca,'FontSize',15);
    set(gca,'TickDir','out'); 
    box off
    xlabel('Meters (m)');
    ylabel('Meters (m)');
    title('Obstacle Avoidance using Model Predictive Control');

    frame = getframe(gcf);

    im = frame2im(frame);
    [imind,cm] = rgb2ind(im,256);
    if i == 1
        imwrite(imind,cm,gif_filename,'gif','Loopcount',inf,'DelayTime',gif_delay);
    else
        imwrite(imind,cm,gif_filename,'gif','WriteMode','append','DelayTime',gif_delay);
    end
end
