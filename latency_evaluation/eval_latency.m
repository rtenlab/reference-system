%%
clc; clear all; close all;
e2eData = readtable("default_e2e_latency.txt");
e2eData_picas = readtable("e2e_latency.txt");

e2eData = sortrows(e2eData, 4);
e2eData_picas = sortrows(e2eData_picas, 4);

e2e_lat = table2array(e2eData(:,2)) - table2array(e2eData(:,5));
e2e_lat_picas = table2array(e2eData_picas(:,2)) - table2array(e2eData_picas(:,5));

figure(1); hold on; grid on;
plot(e2e_lat, 'b', 'LineWidth', 2);
plot(e2e_lat_picas, 'r', 'LineWidth', 2);
title('e2e latency');
ylabel('e2e latency [nsec]');
xlabel('Sample #');

set(gca, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 15);
set(gca, 'Position', [0.04 0.15 0.93 0.7]);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 0.65]);

legend('Singlethreaded(Default)', 'PiCAS');

%%
clc; clear all; close all;

e2eData = readtable("buf_1_default_latency.txt");
e2eData_picas = readtable("latency.txt");

e2eData = sortrows(e2eData, 4);
e2eData_picas = sortrows(e2eData_picas, 4);

behavior_1 = e2eData(e2eData.Var1 == "BehaviorPlanner_subscription_0",:);
behavior_2 = e2eData(e2eData.Var1 == "BehaviorPlanner_subscription_1",:);
behavior_3 = e2eData(e2eData.Var1 == "BehaviorPlanner_subscription_2",:);
behavior_4 = e2eData(e2eData.Var1 == "BehaviorPlanner_subscription_3",:);
behavior_5 = e2eData(e2eData.Var1 == "BehaviorPlanner_subscription_4",:);
behavior_6 = e2eData(e2eData.Var1 == "BehaviorPlanner_subscription_5",:);
behav_lat_1 = table2array(behavior_1(:,2)) - table2array(behavior_1(:,5));
behav_lat_2 = table2array(behavior_2(:,2)) - table2array(behavior_2(:,5));
behav_lat_3 = table2array(behavior_3(:,2)) - table2array(behavior_3(:,5));
behav_lat_4 = table2array(behavior_4(:,2)) - table2array(behavior_4(:,5));
behav_lat_5 = table2array(behavior_5(:,2)) - table2array(behavior_5(:,5));
behav_lat_6 = table2array(behavior_6(:,2)) - table2array(behavior_6(:,5));

behavior_picas_1 = e2eData_picas(e2eData_picas.Var1 == "BehaviorPlanner_subscription_0",:);
behavior_picas_2 = e2eData_picas(e2eData_picas.Var1 == "BehaviorPlanner_subscription_1",:);
behavior_picas_3 = e2eData_picas(e2eData_picas.Var1 == "BehaviorPlanner_subscription_2",:);
behavior_picas_4 = e2eData_picas(e2eData_picas.Var1 == "BehaviorPlanner_subscription_3",:);
behavior_picas_5 = e2eData_picas(e2eData_picas.Var1 == "BehaviorPlanner_subscription_4",:);
behavior_picas_6 = e2eData_picas(e2eData_picas.Var1 == "BehaviorPlanner_subscription_5",:);
behav_lat_picas_1 = table2array(behavior_picas_1(:,2)) - table2array(behavior_picas_1(:,5));
behav_lat_picas_2 = table2array(behavior_picas_2(:,2)) - table2array(behavior_picas_2(:,5));
behav_lat_picas_3 = table2array(behavior_picas_3(:,2)) - table2array(behavior_picas_3(:,5));
behav_lat_picas_4 = table2array(behavior_picas_4(:,2)) - table2array(behavior_picas_4(:,5));
behav_lat_picas_5 = table2array(behavior_picas_5(:,2)) - table2array(behavior_picas_5(:,5));
behav_lat_picas_6 = table2array(behavior_picas_6(:,2)) - table2array(behavior_picas_6(:,5));

collision = e2eData(e2eData.Var1 == "ObjectCollisionEstimator",:);
coll_lat = table2array(collision(:,2)) - table2array(collision(:,5));

collision_picas = e2eData_picas(e2eData_picas.Var1 == "ObjectCollisionEstimator",:);
coll_lat_picas = table2array(collision_picas(:,2)) - table2array(collision_picas(:,5));

figure(2); hold on; grid on;
plot(behav_lat_1, 'LineWidth', 2);
plot(behav_lat_2, 'LineWidth', 2);
plot(behav_lat_3, 'LineWidth', 2);
plot(behav_lat_4, 'LineWidth', 2);
plot(behav_lat_5, 'LineWidth', 2);
plot(behav_lat_6, 'LineWidth', 2);
title('Sensor to BehaviorPlanner latency (Default)');
ylabel('e2e latency [nsec]');
xlabel('Sample #');

set(gca, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 15);
set(gca, 'Position', [0.04 0.15 0.93 0.7]);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 0.65]);
legend('Singlethreaded(Default)', 'PiCAS');

figure(3); hold on; grid on;
plot(behav_lat_picas_1, 'LineWidth', 2);
plot(behav_lat_picas_2, 'LineWidth', 2);
plot(behav_lat_picas_3, 'LineWidth', 2);
plot(behav_lat_picas_4, 'LineWidth', 2);
plot(behav_lat_picas_5, 'LineWidth', 2);
plot(behav_lat_picas_6, 'LineWidth', 2);
title('Sensor to BehaviorPlanner latency, fusion buffer size: 1');
ylabel('e2e latency [nsec]');
xlabel('Sample #');

set(gca, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 15);
set(gca, 'Position', [0.04 0.15 0.93 0.7]);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 0.65]);
legend('Singlethreaded(Default)', 'PiCAS');

figure(4); hold on; grid on;
plot(coll_lat, 'b', 'LineWidth', 2);
plot(coll_lat_picas, 'r', 'LineWidth', 2);
title('Sensor to CollisionEstimator latency, fusion buffer size: 1');
ylabel('e2e latency [nsec]');
xlabel('Sample #');

set(gca, 'FontName', 'Times New Roman');
set(gca, 'FontSize', 15);
set(gca, 'Position', [0.04 0.15 0.93 0.7]);
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0, 1, 0.65]);
legend('Singlethreaded(Default)', 'PiCAS');




