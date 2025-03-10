L = [0.00, 0.13, 0.45, 0.58, 0.67, 0.72, 0.75, 0.77, 0.79, 0.81, 0.81, 0.81, 0.73, 0.81, 0.76, 0.84, 0.76, 0.85, 0.77, 0.86, 0.79, 0.86, 0.80, 0.88, 0.79, 0.86, 0.79, 0.85, 0.88, 0.85, 0.79, 0.88, 0.80, 0.89, 0.80, 0.88, 0.80, 0.88, 0.81, 0.89, 0.80, 0.88, 0.88, 0.88, 0.80, 0.88, 0.81];
R = [0.00, 0.14, 0.45, 0.59, 0.66, 0.71, 0.73, 0.76, 0.76, 0.76, 0.76, 0.76, 0.70, 0.76, 0.70, 0.75, 0.70, 0.76, 0.70, 0.79, 0.70, 0.77, 0.71, 0.77, 0.71, 0.77, 0.72, 0.80, 0.80, 0.77, 0.71, 0.80, 0.72, 0.81, 0.72, 0.80, 0.73, 0.80, 0.72, 0.81, 0.72, 0.80, 0.80, 0.80, 0.72, 0.80, 0.72];

t = linspace(0, 3, 47);

figure();
plot(t, L, '.');
hold on;
plot(t, R, '.');
legend('Left Motor', 'Right Motor');
xlabel('Time (s)');
ylabel('Speed (cm/s)');
title('Motor Calibration Test Curves');
hold off;

%% Fit: 'Left Motor Curve Fit'.
[xData, yData] = prepareCurveData( t, L );

% Set up fittype and options.
ft = fittype( 'a*(1-exp(-b*x))', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.774910464711502 0.817303220653433];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'Left Motor Curve Fit' );
h = plot( fitresult, xData, yData );
legend( h, 'L vs. t', 'Left Motor Curve Fit', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 't', 'Interpreter', 'none' );
ylabel( 'L', 'Interpreter', 'none' );
grid on


%% Fit: 'Right Motor Curve Fit'.
[xData, yData] = prepareCurveData( t, R );

% Set up fittype and options.
ft = fittype( 'a*(1-exp(-b*x))', 'independent', 'x', 'dependent', 'y' );
opts = fitoptions( 'Method', 'NonlinearLeastSquares' );
opts.Display = 'Off';
opts.StartPoint = [0.549860201836332 0.144954798223727];

% Fit model to data.
[fitresult, gof] = fit( xData, yData, ft, opts );

% Plot fit with data.
figure( 'Name', 'Right Motor Curve Fit' );
h = plot( fitresult, xData, yData );
legend( h, 'R vs. t', 'Right Motor Curve Fit', 'Location', 'NorthEast', 'Interpreter', 'none' );
% Label axes
xlabel( 't', 'Interpreter', 'none' );
ylabel( 'R', 'Interpreter', 'none' );
grid on

%% Finding Natural Frequency

data = [-3.06, ...
    1.66, 1.70, 1.78, 1.87, 2.01, 2.14, 2.31, 2.48, 2.61, 2.74, 2.85, 2.91, 2.94, ...
    -1.66, -1.69, -1.75, -1.86, -1.96, -2.10, -2.22, -2.37, -2.52, -2.63, -2.74, -2.82, -2.86, ...
    1.66, 1.67, 1.72, 1.80, 1.89, 2.02, 2.17, 2.29, 2.44, 2.57, 2.66, 2.75, 2.80, 2.81, ...
    -1.67, -1.71, -1.77, -1.87, -1.96, -2.10, -2.23, -2.34, -2.47, -2.59, -2.66, -2.72, -2.76, ...
    1.66, 1.68, 1.73, 1.81, 1.92, 2.02, 2.15, 2.29, 2.40, 2.52, 2.61, 2.67, 2.71, ...
    -1.66, -1.67, -1.72, -1.78, -1.87, -1.99, -2.09, -2.22, -2.35, -2.44, -2.54, -2.61, -2.66, -2.68, ...
    1.66, 1.69, 1.76, 1.83, 1.93, 2.05, 2.16, 2.28, 2.40, 2.48, 2.56, 2.61, 2.63, ...
    -1.66, -1.68, -1.74, -1.81, -1.89, -2.00, -2.12, -2.22, -2.33, -2.43, -2.49, -2.55, -2.58, ...
    1.66, 1.67, 1.71, 1.78, 1.87, 1.95, 2.06, 2.18, 2.27, 2.37, 2.45, 2.50, 2.54, ...
    -1.66, -1.67, -1.71, -1.76, -1.84, -1.93, -2.02, -2.13, -2.23, -2.31, -2.40, -2.45, -2.49, -2.51, ...
    1.66, 1.69, 1.75, 1.81, 1.90, 2.00, 2.08, 2.18, 2.28, 2.34, 2.40, 2.44, 2.46, ...
    -1.66, -1.68, -1.73, -1.80, -1.86, -1.95, -2.05, -2.13, -2.22, -2.28, -2.35, -2.39, -2.41, ...
    1.66, 1.68, 1.71, 1.78, 1.85, 1.92, 2.02, 2.11, 2.18, 2.25, 2.30, 2.35, 2.37, ...
    -1.66, -1.67, -1.71, -1.76, -1.83, -1.91, -1.98, -2.06, -2.13, -2.20, -2.26, -2.29, -2.32, ...
    1.66, 1.67, 1.70, 1.76, 1.81, 1.88, 1.96, 2.03, 2.10, 2.16, 2.21, 2.25, 2.27, ...
    -1.66, -1.67, -1.69, -1.74, -1.80, -1.85, -1.92, -1.98, -2.05, -2.11, -2.15, -2.19, -2.21, ...
    1.66, 1.67, 1.69, 1.73, 1.78, 1.84, 1.90, 1.97, 2.02, 2.08, 2.12, 2.15, 2.17, ...
    -1.66, -1.66, -1.69, -1.73, -1.77, -1.82, -1.87, -1.93, -1.99, -2.03, -2.07, -2.11, -2.12, ...
    1.66, 1.66, 1.68, 1.72, 1.76, 1.81, 1.86, 1.91, 1.97, 2.01, 2.05, 2.07, 2.09, ...
    -1.66, -1.67, -1.69, -1.71, -1.75, -1.79, -1.84, -1.88, -1.92, -1.95, -1.98, -2.00, -2.01, ...
    1.66, 1.66, 1.68, 1.71, 1.73, 1.77, 1.80, 1.84, 1.88, 1.90, 1.93, 1.95, 1.96, ...
    -1.66, -1.66, -1.67, -1.70, -1.72, -1.75, -1.79, -1.82, -1.85, -1.88, -1.90, -1.91, ...
    1.66, 1.66, 1.66, 1.68, 1.69, 1.72, 1.74, 1.78, 1.81, 1.83, 1.86, 1.88, 1.89, ...
    -1.66, -1.66, -1.66, -1.67, -1.69, -1.71, -1.73, -1.75, -1.76, -1.78, -1.79, -1.80, -1.80
];

t = linspace(0, length(data)*0.053, length(data));

figure();
plot(t, data);
hold on;
xlabel('Time (s)');
ylabel('Swing Position');
title('Natural Frequency Calibration');

[peaks, idx] = findpeaks(data);

plot(t(idx), peaks, '.');
legend('Swing Data', 'Peaks');
ylim([-4 4]);
hold off;

freq_hz = (length(peaks)-1) / (t(idx(end)) - t(idx(1)));
wn = freq_hz*2*pi;

l_eff = 9.81 / wn^2;

%% Transfer functions

syms s Kp Ki Jp Ji Ci

g = 9.81;  
l = 0.4815; 
b = 0.00265;  
tau = 0.161;  
a = 1/tau;

K = (Kp*s + Ki)/s;
J = (Jp*s + Ji)/s;
C = Ci/s^2;
M = (a*b)/(s+a);
P = (-s/l) / (s^2 - (g/l));

M_nested_v = M/(1+J*M);
M_nested_x = M_nested_v/(1+C*M_nested_v);


sys = collect(K*M*P);
sys_nested = collect(K * M_nested_x * P);


pretty(sys_nested)