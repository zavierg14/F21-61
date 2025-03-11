close all
clc
clear

pot1data = readtable('Pot120250311101002.csv');
pot1data(1,:) = [];
raw1 = pot1data.RawValue;
timepot = pot1data.Time;
offset = timepot(1);
timepot = timepot - offset;
dt1 = compute_dt(timepot);

pot2data = readtable('Pot220250311101002.csv');
pot2data(1,:) = [];
raw2 = pot2data.RawValue;
timepot2 = pot2data.Time;
timepot2 = timepot2 - offset;

imudata = readtable('IMU20250311101002.csv');
imutime = imudata.Time;
imutime = imutime - offset;
imudt = compute_dt(imutime);
accx = imudata.AccX;
accy = imudata.AccY;
accz = imudata.AccZ;
anglex = imudata.AngleX;
angley = imudata.AngleY;
anglez = imudata.AngleZ;

gpsdata = readtable('GPS20250311101002.csv');
timegps = gpsdata.Time;
timegps = timegps - offset;
lat = gpsdata.Lat;
long = gpsdata.Long;
alt = gpsdata.Alt;
speed = gpsdata.Speed;
sats = gpsdata.Sats;

hall1data = readtable('Hall120250311101002.csv');
timehall1 = hall1data.Time;
timehall1 = timehall1 - offset;
pulsecounts = hall1data.PulseCounts;

figure;
plot(timepot,raw1, "LineWidth", 1)
hold on
plot(timepot2, raw2, "LineWidth", 1)
title("Rear Potentiometers")
xlabel("Time (s)")
ylabel("Raw Value")
legend("Pot1", "Pot2")
grid on
%xlim([15 25])
hold off

figure 
hold on
yyaxis left
plot(timepot(1:end-1), dt1, "LineWidth", 1)
grid on
max(dt1)
1/550
1/max(dt1)

title("Sampling Interval Pots")
xlabel("Time (s)")
ylabel("Time Since Last Sample (s)")
yyaxis right
plot(timepot(1:end-1), (dt1/(1/550)*100-100), "-.", "Linewidth", 1)
ylabel("% off 550Hz")
ylim([0 2200])
grid on
hold off

figure 
plot(imutime, accx, "LineWidth", 1)
hold on
plot(imutime, accy, "LineWidth", 1)
plot(imutime, accz, "LineWidth", 1)
grid on
title("Acceleration Data")
xlabel("Time (s)")
ylabel("Acceleration (g)")
legend("X", "Y", "Z")
hold off

figure
hold on
plot(imutime, anglex, "LineWidth", 1)
plot(imutime, angley, "LineWidth", 1)
plot(imutime, anglez, "LineWidth", 1)
title("Atitude Data")
xlabel("Time (s)")
grid on
ylabel("Angle (deg)")
legend("X", "Y", "Z")
hold off

figure 
hold on
yyaxis left
plot(imutime(1:end-1), imudt, "LineWidth", 1)
yline(.1)
ylabel("Time Since Last Sample (s)")
yyaxis right
plot(imutime(1:end-1), (imudt/(1/10)*100-100))
grid on
title("Sampling Interval IMU")
xlabel("Time (s)")
ylabel("% off 10Hz")
hold on

figure
plot(timegps, speed, "LineWidth", 1)
hold on
title("Speed")
xlabel("Time (s)")
grid on
ylabel("Speed (km/hr)")
hold off

figure
hold on
plot(timegps, sats, "LineWidth", 1)
title("Satellites Connected")
xlabel("Time (s)")
grid on
ylabel("Sat Count")
hold off

figure
plot(timehall1, pulsecounts, "LineWidth", 2)
hold on
title("Pulse Count")
grid on
xlabel("Time (s)")
ylabel("Total Pulses")
xlim([min(timepot) max(timepot)])
hold off

function dt = compute_dt(time_array)
    % compute_dt: Computes the time differences between consecutive elements.
    % 
    % INPUT:
    %   time_array - A numeric vector of time values (e.g., timestamps)
    %
    % OUTPUT:
    %   dt - A vector containing the time differences between consecutive elements
    %
    % USAGE:
    %   dt = compute_dt(time_array);

    % Ensure the input is a column vector
    time_array = time_array(:); 

    % Compute time differences
    dt = diff(time_array);
end