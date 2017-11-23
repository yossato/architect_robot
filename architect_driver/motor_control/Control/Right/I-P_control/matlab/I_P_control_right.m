% ------ 初期設定 ------
close all;
% パラメータ設定
sample_time = 1;

% ------ 同定した変数の読み取り ------
load('I_P_control_right.mat');

% ------ コントローラのゲイン ------
kp = 0.1775;
ki = 0.038;

% ------ ステップ入力ゲイン ------
r = 1;

sim('I_P_control_model_right.slx');

% ------ 結果のプロット ------
fig1 = figure(1);
plt1 = plot(t, controled_y, 'k');
plt1.LineWidth = 0.5;
plt1.DisplayName = 'Simulation Output';
lab1 = legend('-DynamicLegend'); 
lab1.Location = 'best';
hold on;

plt1 = plot(t, target, 'k:');
plt1.LineWidth = 0.5;
plt1.DisplayName = 'Target';
lab1 = legend('-DynamicLegend'); 
lab1.Location = 'best';
hold on;

plt1 = plot(t, output_result, 'ko');
plt1.MarkerSize = 2;
plt1.DisplayName = 'Result Output';
lab1 = legend('-DynamicLegend'); 
lab1.Location = 'best';
hold on;

title('I-P Control Output Result', 'FontSize', 15, 'Interpreter', 'Latex');
xlim([0 200]);
xlabel('step$$[steps]$$', 'FontSize', 15, 'Interpreter', 'Latex');
ylabel('Rotation $$[rad/s]$$', 'FontSize', 15, 'Interpreter', 'Latex');

fig2 = figure(2);
plt2 = plot(t, control_input, 'k--');
plt2.LineWidth = 0.5;
plt2.DisplayName = 'Simulation Input';
lab2 = legend('-DynamicLegend'); 
lab2.Location = 'best';
hold on;

plt2 = plot(t, input_result, 'ko');
plt2.MarkerSize = 2;
plt2.DisplayName = 'Result Input';
lab2 = legend('-DynamicLegend'); 
lab2.Location = 'best';
hold on;

title('I-P Control Input Result', 'FontSize', 15, 'Interpreter', 'Latex');
xlim([0 200]);
xlabel('step$$[steps]$$', 'FontSize', 15, 'Interpreter', 'Latex');
ylabel('Rotation $$[rad/s]$$', 'FontSize', 15, 'Interpreter', 'Latex');

fig3 = figure(3);
plt1 = plot(t, controled_y, 'k');
plt1.LineWidth = 0.5;
plt1.DisplayName = 'Simulation Output';
lab1 = legend('-DynamicLegend'); 
lab1.Location = 'best';
hold on;

plt1 = plot(t, target, 'k:');
plt1.LineWidth = 0.5;
plt1.DisplayName = 'Target';
lab1 = legend('-DynamicLegend'); 
lab1.Location = 'best';
hold on;

plt1 = plot(t, output_result, 'ko');
plt1.MarkerSize = 2;
plt1.DisplayName = 'Result Output';
lab1 = legend('-DynamicLegend'); 
lab1.Location = 'best';
hold on;

plt2 = plot(t, control_input, 'k--');
plt2.LineWidth = 0.5;
plt2.DisplayName = 'Simulation Input';
lab2 = legend('-DynamicLegend'); 
lab2.Location = 'best';
hold on;

plt2 = plot(t, input_result, 'ko');
plt2.MarkerSize = 2;
plt2.DisplayName = 'Result Input';
lab2 = legend('-DynamicLegend'); 
lab2.Location = 'best';
hold on;

title('I-P Control Result', 'FontSize', 15, 'Interpreter', 'Latex');
xlim([0 200]);
xlabel('step$$[steps]$$', 'FontSize', 15, 'Interpreter', 'Latex');
ylabel('Rotation $$[rad/s]$$', 'FontSize', 15, 'Interpreter', 'Latex');
% % ------ 結果のプロット終了 ------