clc;
close all;

%open bag file
bag = rosbag('/home/marley/catkin_ws/src/lab4_driver/bag_files/data_driving.bag');

%rosbag info 'data_going_in_circles.bag';


bsel = select(bag,'Topic','/imu');
msgStructs1 = readMessages(bsel,'DataFormat','struct');
bsel2 = select(bag,'Topic','/gps');
msgStructs2 = readMessages(bsel2,'DataFormat','struct');

mag_x = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStructs1);
mag_y = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStructs1);
mag_z = cellfun(@(m) double(m.MagField.MagneticField_.Z),msgStructs1);

omega_x = cellfun(@(m) double(m.IMU.AngularVelocity.X),msgStructs1);
omega_y = cellfun(@(m) double(m.IMU.AngularVelocity.Y),msgStructs1);
omega_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z),msgStructs1);

acc_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X),msgStructs1);
acc_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),msgStructs1);
acc_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),msgStructs1);

orientation_x = cellfun(@(m) double(m.IMU.Orientation.X),msgStructs1);
orientation_y = cellfun(@(m) double(m.IMU.Orientation.Y),msgStructs1);
orientation_z = cellfun(@(m) double(m.IMU.Orientation.Z),msgStructs1);
orientation_w = cellfun(@(m) double(m.IMU.Orientation.W),msgStructs1);

UTM_easting = cellfun(@(m) double(m.UTMEasting),msgStructs2);
UTM_northing = cellfun(@(m) double(m.UTMNorthing),msgStructs2);

gps_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs2);
gps_time_nano_sec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs2);
imu_time_points = double(gps_time_sec + ( gps_time_nano_sec * 10^(-9)));
gps_time = imu_time_points - imu_time_points(1);

imu_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs1);
imu_time_nano_sec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs1);
imu_time_points = double(imu_time_sec + ( imu_time_nano_sec * 10^(-9)));
imu_time = imu_time_points - imu_time_points(1);

%quat to euler
quat = [orientation_w orientation_x orientation_y orientation_z];
eulZYX_rad = quat2eul(quat);
yaw = eulZYX_rad (:,1);
pitch = eulZYX_rad (:,2);
roll = eulZYX_rad (:,3);

%integrate acceleration to velocity
vel_imu = cumtrapz(imu_time,acc_x);
vel_gps = zeros(length(UTM_easting),1);
utm_distance = zeros(length(UTM_easting),1);
gps_time_d =  zeros(length(UTM_easting),1);
for i = 2 : length(UTM_easting)

    utm_distance(i-1) = sqrt((UTM_easting(i)-UTM_easting(i-1))^2+(UTM_northing(i)-UTM_northing(i-1))^2);
    vel_gps(1) =  sqrt((UTM_easting(2)-UTM_easting(1))^2+(UTM_northing(2)-UTM_northing(1))^2);   
    gps_time_d(i-1) = abs(gps_time(i) - gps_time(i-1));
    if gps_time_d(i-1) ==0
        vel_gps(i) = vel_gps(i-1);
    else
        vel_gps(i) = (utm_distance(i-1)/gps_time_d(i-1));  
    end
end
gps_time = sort(gps_time);

figure;
plot(imu_time, vel_imu, "DisplayName","Forward velocity (accelerometer)",'LineWidth',2.0);
hold on;
plot(gps_time, vel_gps, "DisplayName"," velocity from GPS ",'LineWidth',2.0);
xlabel('time (s)')
ylabel('vel (m/s)')
title('Forward velocity (accelerometer) vs velocity from GPS')
legend;


% moving average filter - accelerometer

a = 0.992;
accel_filt = zeros(length(acc_x),1);
accel_filt(1) = acc_x(1);
for i = 2 : length(acc_x)    
    accel_filt(i) = (1-a)*acc_x(i) + a * accel_filt(i-1);
end

%dynamically change our acceleration values
%measure start - end values for bias removal
%dynamically change it based on gps velocity. If gps velocity is zero - we didnt move - our imu
%is supposed to say the same. It wont, so, thats our bias
accel_chg = diff(accel_filt);
len = 0;
start_end_time = [];
for i = 2 : length(accel_chg)
    if abs(accel_chg(i)) < 0.001
        len = len + 1;
    else
        if len > 1*40
            start_c = int16(imu_time(i-1-len));
            end_c = int16(imu_time(i-1));

            a = find(gps_time>start_c,1,'first');
            b = find(gps_time>end_c,1,'first')-1;

            if vel_gps(a:b,1) < 0.7
                start_end_time = [start_end_time,i-1-len];
                start_end_time = [start_end_time,i-1];
            end
        end
        len = 0;
    end
end

%dynamic bias removal
start_end = length(start_end_time);
for i = 2 : (start_end)
    bias = mean(accel_filt(start_end_time(i-1):start_end_time(i)));
    if i==2
        start = 1;
    else
        start = start_end_time(i-1);
    end   
    if i < start_end -1
        end_ = start_end_time(i);
    else
        end_ = length(accel_filt);
    end
    for j = start : end_
        accel_correct(j) = accel_filt(j)- bias;
    end
end


fwd_correct_vel = cumtrapz(imu_time,accel_correct);
fwd_correct_vel(end) = 0;
vel_gps(length(vel_gps)) =0;

figure;
plot(imu_time,acc_x,"DisplayName"," Accel - before adj", 'LineWidth',2.0);
hold on;
plot(imu_time,accel_filt,"DisplayName","Accel - after smoothening", 'LineWidth',2.0);
hold on;
plot(imu_time,accel_correct,"DisplayName","Accel - after adj", 'LineWidth',2.0);
% hold on;
xlabel('time (s)')
ylabel('accel (m/s^2)')
title('Acceleration before and after dynamic removal of bias')
legend


figure;
plot(gps_time,vel_gps,"DisplayName"," Velocity estimate - GPS ", 'LineWidth',2.0);
hold on;
plot(imu_time,fwd_correct_vel,"DisplayName","Vel (accel) - after adj", 'LineWidth',2.0);
xlabel('time (s)')
ylabel('vel (m/s)')
title('plot - adjusted velocity with the GPS velocity')
legend












