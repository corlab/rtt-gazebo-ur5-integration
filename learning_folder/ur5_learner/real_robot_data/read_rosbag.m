clear rosbag_wrapper;
clear ros.Bag;
clear all

addpath('/homes/abalayn/Downloads/matlab_rosbag-0.5.0-linux64/');

%bag = ros.Bag.load('/homes/abalayn/Downloads/2016-06-17-16-46-51.bag');
%bag = ros.Bag.load('/homes/abalayn/Downloads/rosbag_joint_states.bag'); % Old data
%bag = ros.Bag.load('/homes/abalayn/Downloads/2016-06-22-09-29-56.bag'); % Small test
bag = ros.Bag.load('/homes/abalayn/Downloads/2016-06-22-14-07-06.bag');%With 2.5 end effector

bag.info()


%% See definitions of messages contained within the bag
raw = true;
twist_definition = bag.definition('/joint_states' , raw)


%% Read all messages on a few topics
topic1 = '/joint_states';


[msgs, meta] = bag.readAll(topic1);
fprintf('Got %i messages, first one at time %f\n', ...
    length(msgs), meta{1}.time.time);



%% Convert position and torque messages to a matrix to plot linear speed
accessor = @(twist) twist.position;
[xyz] = ros.msgs2mat(msgs, accessor); % Convert struct to 6-by-N matrix of position
accessor2 = @(eff) eff.effort;
[effort] = ros.msgs2mat(msgs , accessor2);

times = cellfun(@(x) x.time.time, meta) - meta{1}.time.time; % Get timestamps

% !!!!! Inverse line 1 and line 3 because rosbag inverse the joints!
xyz0 = xyz(3,:);
xyz2 = xyz(1,:);
xyz(1,:) = xyz0;
xyz(3,:) = xyz2;

effort0 = effort(3,:);
effort2 = effort(1,:);
effort(1,:) = effort0;
effort(3,:) = effort2;

for n=1:6
    figure(n)
    % Plot position and torque over time
    plot(times, effort(n, :));
    hold on;
    plot(times, xyz(n, :));
    str_title=sprintf('Torque and position - joint %d - as a function of time', (n-1));
    title(str_title);
    xlabel('time in s')
    ylabel('joint torque and joint position');
    legend('torque in N' , 'position in rad');
end


%% Computation of mean position and mean torque on 20 measures.
new_time = [];
time_step = 0;

temp_trq = [];
temp_pos = [];

mean_trq = [];
mean_pos = [];

mean_trq_temp = [0 ; 0 ; 0 ; 0 ; 0 ; 0];
mean_pos_temp = [0 ; 0 ; 0 ; 0 ; 0 ; 0];

nb_temp = 1;

for vec = 1:size(times ,2)
    nb_temp = nb_temp + 1;
    if (nb_temp >= 51)
        for n=1:6
            mean_trq_temp(n) = mean(temp_trq( n , :));
            mean_pos_temp(n) = mean(temp_pos(n , :));
        end  
        mean_trq = [mean_trq mean_trq_temp];
        mean_pos = [mean_pos mean_pos_temp];
        new_time = [new_time time_step];
        nb_temp = 1;
        temp_trq = [];
        temp_pos = [];
        mean_trq_temp = [0 ; 0 ; 0 ; 0 ; 0 ; 0];
        mean_pos_temp = [0 ; 0 ; 0 ; 0 ; 0 ; 0];
    end     
      
        
      temp_trq = [temp_trq effort(: , vec) ];
      temp_pos = [temp_pos xyz(: , vec)];
      
      time_step = time_step + (meta{end}.time.time - meta{1}.time.time)/size(msgs,2);
    
end


figure();
for n=1:6
    subplot(2,3 , n);
    % Plot position and torque over time
    plot(new_time, mean_trq(n, :));
    hold on;
    plot(new_time, mean_pos(n, :));
    line([41.65 41.65] , [-4 2] , 'Color',[.8 .8 .8]);
    line([52.51 52.51] , [-4 2] , 'Color',[.8 .8 .8]);
    line([71.94 71.94] , [-4 2] , 'Color',[.8 .8 .8]);
    line([89.07 89.07] , [-4 2] , 'Color',[.8 .8 .8]);
    line([127.1 127.1] , [-4 2] , 'Color',[.8 .8 .8]);
    line([139.1 139.1] , [-4 2] , 'Color',[.8 .8 .8]);
    line([155.9 155.9] , [-4 2] , 'Color',[.8 .8 .8]);
    xlim([0 ; time_step]);
    str_title=sprintf('Mean Torque and position - joint %d - as a function of time', (n-1));
    title(str_title);
    xlabel('time in s')
    ylabel('mean joint torque and joint position');
    legend('torque in N' , 'position in rad');
end



%% Read positions which seem stable. 


%{
%Observation: Decided based on torque and positions which seems the more stable.
trgt_pnt = [4.437 15.14 24.52 29.96 36.87 57.45 62.4 67.83 90.22 94.01 97.96 101.9 111.8 117.1 120.7 130.4 143.6 151.3 158.9 175.5 177.8];
%Found by comparing the target time with array data.
trgt_col = [27 92 149 182 224 349 379 412 548 571 595 619 679 711 733 792 872 919 965 1066 1080];
%Real time, position and torque for these data.
pos = [];
trq = [];
time_point = [];
for k=trgt_col
    pos = [pos mean_pos(:,k)];
    trq = [trq mean_trq(: , k)];
    time_point = [time_point new_time(k)];
end    
pos_deg = pos*180/3.14;

figure()
for n=1:6
    subplot(2,3,n);
    plot(time_point , pos_deg(n,:))
    hold on
    plot(time_point , trq(n,:))
    legend('pos' , 'trq');
end    
%}
