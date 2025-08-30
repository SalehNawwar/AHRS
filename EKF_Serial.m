% Start of script

addpath('quaternion_library');      % include quaternion library
addpath('GyroscopeIntegration');    % include Gyroscope integration library
addpath('AccelerometerMagnetometer');    % include Accelerometer and Magnetometer Integration library
addpath('EulerKF');                 % Linear Kalman filter with Euler
addpath('EulerEKF');                % Extended Kalman filter with Euler
addpath('EulerUKF');                % Unscented Kalman filter with Euler
addpath('EKF');                     % Extended Kalman filter
addpath('UKF');                     % Unscented Kalman filter
close all;                          % close all figures
clear all;                          % clear all variables
clc;                                % clear the command terminal

%  Import and plot sensor data
port = "COM5";         
baudRate = 115200;      
s = serialport(port, baudRate,'ByteOrder','little-endian');
s.Timeout = 30;
flush(s);
disp("waiting mark");
Accelerometer = [];
Magnetometer = [];
Gyroscope = [];
time = double([]);
while 1
   mark = read(s,1,'uint16');
   if mark==200
       break
   end
end
disp("started receiving");
k = 1;
try
    disp("start initial alignment")
    while 1
        
        time(k) = double(read(s,1,'uint32'));
        Accelerometer(k,:) = double(read(s,3,'single'));
        Gyroscope(k,:) = deg2rad(read(s,3,'single'));
        Magnetometer(k,:) = double(read(s,3,'single'));
        if k<1000
            continue
        elseif k==1000
        bg1=Gyroscope(1:1000,1);
        bg2=Gyroscope(1:1000,2);
        bg3=Gyroscope(1:1000,3);
        disp("end initial alignment")

        disp("start estimating")
        else
        
    
    
        


%load('data.mat');
Accelerometer(:,1) = Accelerometer(:,1) + 0.0193;
Accelerometer(:,2) = Accelerometer(:,2) - 0.02439;
Accelerometer(:,3) = Accelerometer(:,3) + 0.014847;
Accelerometer(:,1) = 0.99984*Accelerometer(:,1) + 0.0003*Accelerometer(:,2) + 0.0008*Accelerometer(:,3) ;
Accelerometer(:,2) = -0.0002*Accelerometer(:,1) + 0.99609*Accelerometer(:,2) + 0.0027*Accelerometer(:,3);
Accelerometer(:,3) = 0.0015*Accelerometer(:,1) -0.0004*Accelerometer(:,2) + 0.99658*Accelerometer(:,3);
b = deg2rad([-0.2385758 , -1.4034547 , 0.6379800]) + [-0.0018 , -0.0017 , 0.0144];
Gyroscope(:,1) = Gyroscope(:,1) - bg1;
Gyroscope(:,2) = Gyroscope(:,2) - bg2;
Gyroscope(:,3) = Gyroscope(:,3) + bg3;
Magnetometer(:,1) = Magnetometer(:,1) + 0.02/100;
Magnetometer(:,2) = Magnetometer(:,2) - 32.83/100;
Magnetometer(:,3) = Magnetometer(:,3) + 23.39/100;
Magnetometer(:,1) = 0.997*Magnetometer(:,1) + 0.028*Magnetometer(:,2) - 0.041*Magnetometer(:,3) ;
Magnetometer(:,2) = +0.028*Magnetometer(:,1) + 1.011*Magnetometer(:,2) - 0.041*Magnetometer(:,3);
Magnetometer(:,3) = -0.041*Magnetometer(:,1) - 0.041*Magnetometer(:,2) + 0.996*Magnetometer(:,3);

figure('Name', 'Sensor Data');
axis(1) = subplot(3,1,1);
hold on;
plot(time, Gyroscope(:,1), 'r');
plot(time, Gyroscope(:,2), 'g');
plot(time, Gyroscope(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Angular rate (deg/s)');
title('Gyroscope');
hold off;
axis(2) = subplot(3,1,2);
hold on;
plot(time, Accelerometer(:,1), 'r');
plot(time, Accelerometer(:,2), 'g');
plot(time, Accelerometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Acceleration (g)');
title('Accelerometer');
hold off;
axis(3) = subplot(3,1,3);
hold on;
plot(time, Magnetometer(:,1), 'r');
plot(time, Magnetometer(:,2), 'g');
plot(time, Magnetometer(:,3), 'b');
legend('X', 'Y', 'Z');
xlabel('Time (s)');
ylabel('Flux (G)');
title('Magnetometer');
hold off;
linkaxes(axis, 'x');

 Process sensor data through Extended Kalman filter with Quternions PX4 autopilot
clearvars -except time Gyroscope Accelerometer Magnetometer
EulerSaved = zeros(length(time), 3);

AHRSEKF = EKFfilter();
for t = 1:length(time)
    if t > 1
        dt = (time(t)-time(t-1));
    else
        dt = (time(2)-time(1));
    end
              
    AHRSEKF.Update(dt, Gyroscope(t,:) * (pi/180), Accelerometer(t,:), Magnetometer(t,:));
    EulerSaved(t, :) = AHRSEKF.eulerAngles;    

end

PhiSaved   = mod(EulerSaved(:, 1) * (180/pi),360)-180;
ThetaSaved = EulerSaved(:, 2) * (180/pi);
PsiSaved   = mod((EulerSaved(:, 3)-EulerSaved(1, 3)) * (180/pi)+180,360)-180;

figure('Name', 'Extended Kalman Filter');
hold on;
plot(time, PhiSaved, 'r');
plot(time, ThetaSaved, 'g');
plot(time, PsiSaved, 'b');
title('Extended Kalman Filter');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('\phi Roll', '\theta Pitch', '\psi Yaw');
grid on;
hold off;
        end
        k=k+1
    end
catch ME
    disp(ME.message);
    clear s;
end
