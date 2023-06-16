clc;
close all;

%open bag file
bag = rosbag('/home/marley/catkin_ws/src/lab4_driver/bag_files/data_driving.bag');

%rosbag info 'data_going_in_circles.bag';

% imu_data
bsel = select(bag,'Topic','/imu');
msgStructs = readMessages(bsel,'DataFormat','struct');

mag_x = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStructs);
mag_y = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStructs);
mag_z = cellfun(@(m) double(m.MagField.MagneticField_.Z),msgStructs);

omega_x = cellfun(@(m) double(m.IMU.AngularVelocity.X),msgStructs);
omega_y = cellfun(@(m) double(m.IMU.AngularVelocity.Y),msgStructs);
omega_z = cellfun(@(m) double(m.IMU.AngularVelocity.Z),msgStructs);

orientation_x = cellfun(@(m) double(m.IMU.Orientation.X),msgStructs);
orientation_y = cellfun(@(m) double(m.IMU.Orientation.Y),msgStructs);
orientation_z = cellfun(@(m) double(m.IMU.Orientation.Z),msgStructs);
orientation_w = cellfun(@(m) double(m.IMU.Orientation.W),msgStructs);

acc_x = cellfun(@(m) double(m.IMU.LinearAcceleration.X),msgStructs);
acc_y = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),msgStructs);
acc_z = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),msgStructs);

imu_time_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs);
imu_time_nano_sec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs);
imu_time_points = double(imu_time_sec + ( imu_time_nano_sec * 10^(-9)));
imu_time = imu_time_points - imu_time_points(1);

%quat to euler
quat = [orientation_w orientation_x orientation_y orientation_z];
eulZYX_rad = quat2eul(quat);
yaw = eulZYX_rad (:,1);
pitch = eulZYX_rad (:,2);
roll = eulZYX_rad (:,3);

%calibration matrix - from magnetometer calibration.m
scale_matrix = [0.8760 0.0243; 0.0243 0.9952];
offset_magx = -0.1604;
offset_magy = 0.0197;
corrected_magX = mag_x - offset_magx;
corrected_magY = mag_y - offset_magy;
calibrated_mag =  (scale_matrix*[corrected_magX,corrected_magY]')';

% yaw_from_magnetometer
calib_mag_yaw= (atan2(-calibrated_mag(:,2),calibrated_mag(:,1)));
mag_yaw_raw = atan2(-mag_y,mag_x);
unwrapped_mag_yaw = unwrap(calib_mag_yaw);
figure;
plot(imu_time, calib_mag_yaw, "DisplayName"," Corrected yaw",'LineWidth',2.0);
hold on;
plot(imu_time,mag_yaw_raw,"DisplayName"," Raw magnetometer yaw",'LineWidth',2.0);
xlabel('time (s)')
ylabel('yaw (rad)')
title('Comparision of Yaw Angles from Magnetometer')
legend;

% yaw_from_gyroscope
gyro_yaw = cumtrapz(imu_time,omega_z)+ calib_mag_yaw(1);
wrapped_gyro_yaw = wrapToPi(gyro_yaw);
figure;
plot(imu_time,calib_mag_yaw,"DisplayName"," Corrected Magnetometer Yaw",'LineWidth',2.0);
hold on;
plot(imu_time,wrapped_gyro_yaw,"DisplayName","Yaw from Gyro ",'LineWidth',2.0);
xlabel('time (s)')
ylabel('yaw (rad)')
title('Magnetometer vs. Yaw Integrated from Gyro')
legend;
 
%Low pass filter on magnetometer
mag_low_pass= lowpass(unwrapped_mag_yaw, 0.001, 40);

%high pass filter on gyro yaw
gyro_high_pass = highpass(gyro_yaw,0.01,40);

%complemetary filter
a_c = 0.4;
filtered_yaw = a_c*mag_low_pass + (1-a_c)*gyro_high_pass;

figure;
plot(imu_time,(mag_low_pass),"DisplayName","Yaw (Mag) - low pass filter",'LineWidth',2.0);
hold on;
plot(imu_time,(gyro_high_pass),"DisplayName","Yaw (Gyro) - high pass filter",'LineWidth',2.0);
hold on;
plot(imu_time, (filtered_yaw), "DisplayName","Yaw - Complementary filter",'LineWidth',2.0);
hold on;
xlabel('time (s)')
ylabel('yaw (rad)')
title('Low pass vs high pass vs complementary filters')
legend;


%complementary filter vs imu yaw
figure;
plot(imu_time,unwrap(yaw),"DisplayName","Yaw from IMU",'LineWidth',2.0);
hold on;
plot(imu_time, (filtered_yaw), "DisplayName","Yaw - Complementary filter",'LineWidth',2.0);
hold on;
xlabel('time (s)')
ylabel('yaw (rad)')
title('Yaw from the Complementary filter & Yaw angle computed by the IMU together')
legend;