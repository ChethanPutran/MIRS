%######################################### MOTION PARAMS ###############################################%

% Declare goal and motion params
qg = 100*10^-3;         %(in m)
qs = 0;             %(in m)
tg = 6;                  %(in sec)
const_acceleration = 20*10^-3;  %(in m/s^2)

%%%% Sampling Params %%%%

%Sampling Time
delT = 0.05;                     % (in sec)

%Sampling Frequency
frequency = (1/delT);            % (in Hz)

% !!! Keep step_size <= delT
step_size = delT;

%%%% Compute via points %%%%
qm = 0.5*(qg + qs);                                                                     % Midposition
tm = tg / 2;                                                                            % Mid-time
tb = 0.5 * tg - (0.5*sqrt(((const_acceleration*tg^2)-4*(qg-qs))/(const_acceleration))); % Blend time
tp = tg - tb;                                                                           % Time upto last blend (tg-tb)
qb = qm - (const_acceleration * tb * (tm - tb));                                        % Blend position


%============= Square signal generator function %=============%

%Function to create single square wave signal (consisiting of five points)
function sig = signal(time,amplitude,step_size)
    sig = [
            time                 ,     0;
            time                 , amplitude;
            time + (step_size/2) , amplitude;
            time + (step_size/2) ,     0;
            time +  step_size    ,     0
        ];
end

%Function to generate square wave for a given curve
function [X,Y] = generateSignal(amplitudes,time_stamps,step_size)
    %Get total number of sample points in the reference signal
    n = length(amplitudes);

    % Initialize the points matrix with zeros
    % points = [
    %            [x1,y1],
    %            [x2,y2]
    %            ...
    %           ]

    points = zeros(5*n,2);

    %Iterate over entire reference signal to generate square signal
    for i = 1 : n
        wave = signal(time_stamps(i),amplitudes(i),step_size);
        points(5*(i-1)+1:5*i,:)= wave;
    end

    %Return x and y co-ordinates of the signal
    X = points(:,1);
    Y = points(:,2);
end



%############################################### Trajectory Computation ###############################################%

%================ Time =================%

tx = 0:delT:tb;             % 0 <= t <= tb          -> 1st segment position 0 to tb
ty = tb+delT:delT:tp;            % tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
tz = tp+delT:delT:tg;            % (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
t = [tx,ty,tz];             % Combine segment times

%=========== Displacements =============%

d1 = qs + 0.5*const_acceleration*(tx.^2);                                       % 0 <= t <= tb          -> 1st segment position 0 to tb
d2 = qs + (0.5*const_acceleration*(tb.^2)) + (const_acceleration*tb*(ty-tb));   % tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
d3 = qg - (0.5*const_acceleration*((tg-tz).^2));                                % (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
d = [d1,d2,d3];                                                                 % Combine segment displacements

% Position vs Time plot
%s1 = subplot(4,1,1);
plot(t,d);
ylabel('Position (m)');
xlabel('Time (s)');

%Debug
%fprintf('Program paused. Press enter to continue.\n');
%pause;


%============ Velocities =============%

v1 = const_acceleration*tx;                         % 0 <= t <= tb          -> 1st segment position 0 to tb
v2 = const_acceleration*tb*ones(size(ty));          % tb <= t <= (tg - tb)  -> 2nd segment position tb to tg-tb
v3 = const_acceleration*(tg-tz);                    % (tg - tb) <= t <= tg  -> 3rd segment position tg-tb to tg
v = [v1,v2,v3];                                     % Combine segment velocities


% Plot of Velocity vs Time
%s2 = subplot(4,1,2);
plot(t,v);
ylabel('Velocity (m/s)');
xlabel('Time (s)');
title("Plot of Velocity vs Time");

%Debug
%fprintf('Program paused. Press enter to continue.\n');
%pause;

%Fit polynomial function and get polynomial coefficients
p = polyfit(t,v,3);
%xlswrite('data.xlsx',p);


%========== Accelerations =============%

a1 = const_acceleration*ones(size(tx));              % 0 <= t <= tb         -> 1st segment acceleration 0 to tb
a2 = 0*ty;                                           % tb <= t <= (tg - tb) -> 2nd segment acceleration tb to tg-tb
a3 = -const_acceleration*ones(size(tz));             % (tg - tb) <= t <= tg -> 3rd segment acceleration tg-tb to tg
a = [a1,a2,a3];                                      % Combine the segment accelerations

% Plot of Acceleration vs Time
%s3 = subplot(4,1,3);
plot(t,a);
ylabel('Acceleration (m/s^2)');
xlabel('Time (s)');
title("Plot of Acceleration vs Time");

hold off;
% Generate square signal
[X,Y]= generateSignal(v,t,step_size);

% Plot of step-size vs time
%s3 = subplot(4,1,4);

% Plot reference curve
plot(t,v);
hold on; % Keep previous plot visible

% Denote reference points
scatter(t,v, 5, "filled");
hold on; % keep previous plot visible

% Plot square signal
plot(X,Y);

xlabel("Time (in s)");
ylabel("Step size (in m)");
title("Plot of Step size vs Time");
hold off;

P = [X,Y];
%xlswrite('steps.xlsx',X,Y);

%{
f1 = ((v1)/(step_size));  %1st segment frequency   0  to tb
y1 = 0.5*step_size+0.5*step_size*square(2*pi*0.5*f1.*tx);

f2 = ((v2)/(step_size));                  %2nd segment frequency   tb to tg-tb
y2 = 0.5*step_size+0.5*step_size*square(2*pi*f2.*ty);

y3 = fliplr(y1);
y = [y1,y2,y3];

yin = timeseries(y,t);
s4 = subplot(4,1,4); plot(yin);
ylabel('step_size (m)');
xlabel('Time (s)');

figure(2)
plot(yin)

hold on

z1=0.5*step_size+0.5*step_size*square(0.5*pi*0.5*f1.*tx);
f2=((v2)/(step_size));                  %2nd segment frequency   tb to tg-tb
z2=0.5*step_size+0.5*step_size*square(0.5*pi*f2.*ty);
z3= fliplr(z1);
z=[z1,z2,z3];
zin=timeseries(z,t); plot(zin)

hold off

%}
