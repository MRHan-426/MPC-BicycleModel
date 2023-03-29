% Creates a plot of the simulation outputs (track, reference trajectory, simulated trajectory, obstacles)
% Additionally creates an animation video of the simulation if video = 1
% Inputs - None - requires files saved from simulation.m
% Outputs - None - produces simulation plot and optional animation video.  

clear
close all
clc
load ('trajectories/sim_result2.mat');
load ('trajectories/TestTrack.mat');
load ('trajectories/RefTraj.mat');

video=1; %set to 1 for making video else 0

if video
v = VideoWriter('trajectories/car_sim.avi','Motion JPEG AVI');
v.Quality = 95;
open(v);
end

for i=1:35:size(Y_sim,1)-50
plot(TestTrack.bl(1,:)',TestTrack.bl(2,:)','k','LineWidth',1.5);
hold on
plot(TestTrack.br(1,:)',TestTrack.br(2,:)','k','LineWidth',1.5);
for j=1:length(Xobs)
    %plot([Xobs{1,j}(:,1);Xobs{1,j}(1,1)],[Xobs{1,j}(:,2);Xobs{1,j}(1,2)],'r','LineWidth',1.5)
    fill([Xobs{1,j}(:,1);Xobs{1,j}(1,1)],[Xobs{1,j}(:,2);Xobs{1,j}(1,2)],'r');
end
try
    h1 = plot(Y(1:i,1),Y(1:i,3),'g','LineWidth',1.5);
    plot(Y(i,1),Y(i,3),'go','MarkerFaceColor','g','LineWidth',1.5);
catch
    h1 = plot(Y(1:end,1),Y(1:end,3),'g','LineWidth',1.5);
    plot(Y(end,1),Y(end,3),'go','MarkerFaceColor','g','LineWidth',1.5);
end
h2 = plot(Y_sim(1:i,1),Y_sim(1:i,3),'b','LineWidth',1.5);
plot(Y_sim(i,1),Y_sim(i,3),'bo','MarkerFaceColor','b','LineWidth',1.5);

hold off
xlim([Y_sim(i,1)-50,Y_sim(i,1)+50]);
ylim([Y_sim(i,3)-50,Y_sim(i,3)+50]);
legend([h1 h2],'Reference Trajectory','MPC Trajectory','Position',[0.75,0.2,0.124,0.037],'FontSize',15);
set(gcf,'Position',[0 0 1500 1000]);
set(gca,'FontSize',15);
set(gca,'TickDir','out'); 
box off
xlabel('Meters (m)');
ylabel('Meters (m)');
title('Car Obstacle Avoidance using Model Predictive Control');
if video
frame = getframe(gcf);
writeVideo(v,frame);
end

end
if video
close(v);
end