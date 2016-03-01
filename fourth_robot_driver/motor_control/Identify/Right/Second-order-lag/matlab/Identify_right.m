% ------ 初期設定 ------
close all;

% パラメータ設定
sample_time = 1;
% ------ 初期設定 終了 ------

load('Identify_right.mat');

% ------ ステップ同定 ------
% 実験時の目標値
wheel_diameter = 0.38725;
r = wheel_diameter*pi*(100/60);
% データから同定パラメータの読み取り
k_max = 19.0;
y_max = 2.46696;
y0 = 2.25;
% ------ 変数の算出 ------
a1 = -(1/k_max)*2*log((y_max/y0)-1);
a0 = (pi^2/k_max^2) + (a1^2/4);
b0 = a0*y0/r;
% ------ ステップ同定終了 ------

sim('Identify_model_right.slx');

% ------ 結果のプロット ------
fig = figure(1);
plt = plot(t, y, 'k');
plt.LineWidth = 0.5;
plt.DisplayName = 'Identified';
lab = legend('-DynamicLegend'); 
lab.Location = 'best';
hold on;

plt = plot(t, data, 'ko');
plt.MarkerSize = 2.1;
plt.DisplayName = 'Raw Data';
lab = legend('-DynamicLegend'); 
lab.Location = 'best';
hold on;

plt = plot(t, target, 'k:');
plt.LineWidth = 0.5;
plt.DisplayName = 'Target';
lab = legend('-DynamicLegend'); 
lab.Location = 'best';
hold on;

title('Identify Result', 'FontSize', 15, 'Interpreter', 'Latex');
xlim([0, 140]);
xlabel('step [steps]', 'FontSize', 15, 'Interpreter', 'Latex');
ylabel('output [rad/s]', 'FontSize', 15, 'Interpreter', 'Latex');
% % ------ 結果のプロット終了 ------