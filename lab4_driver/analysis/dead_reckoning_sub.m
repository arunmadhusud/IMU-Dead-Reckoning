
clc;
close all;

%open bag file
bag = rosbag('/home/marley/catkin_ws/src/lab4_driver/bag_files/data_driving.bag');

%rosbag info 'data_going_in_circles.bag';

bsel1 = select(bag,'Topic','/imu');
msgStructs1 = readMessages(bsel1,'DataFormat','struct');
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


scale_matrix = [0.8760 0.0243; 0.0243 0.9952];
offset_magx = -0.1604;
offset_magy = 0.0197;
corrected_magX = mag_x - offset_magx;
corrected_magY = mag_y - offset_magy;
mag_calib =  (scale_matrix*[corrected_magX,corrected_magY]')';

calib_mag_yaw= (atan2(-mag_calib(:,2),mag_calib(:,1)));
mag_yaw_raw = atan2(-mag_y,mag_x);
unwrapped_mag_yaw = unwrap(calib_mag_yaw);
gyro_yaw = cumtrapz(imu_time,omega_z)+ calib_mag_yaw(1);
wrapped_gyro_yaw = wrapToPi(gyro_yaw);

%Low pass filter on magnetometer
mag_low_pass= lowpass(unwrapped_mag_yaw,0.001,40);

%high pass filter on gyro yaw
gyro_high_pass = highpass(gyro_yaw,0.01,40);

%complemetary filter
a_c = 0.992;
filtered_yaw = a_c*mag_low_pass + (1-a_c)*gyro_high_pass;


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
    vel_gps(1) =  sqrt((UTM_easting(2)-UTM_easting(1))^2+(UTM_northing(2)-UTM_northing(1))^2);   
    utm_distance(i-1) = sqrt((UTM_easting(i)-UTM_easting(i-1))^2+(UTM_northing(i)-UTM_northing(i-1))^2);
    gps_time_d(i-1) = abs(gps_time(i) - gps_time(i-1));
    if gps_time_d(i-1) ==0
        vel_gps(i) = vel_gps(i-1);
    else
        vel_gps(i) = (utm_distance(i-1)/gps_time_d(i-1));  
    end
end
gps_time = sort(gps_time);

% moving average filter - accelerometer

a = 0.992;
accel_filt = zeros(length(acc_x),1);
accel_filt(1) = acc_x(1);
for i = 2 : length(acc_x)    
    accel_filt(i) = (1-a)*acc_x(i) + a * accel_filt(i-1);
end

%dynamically change our acceleration values
%measure start - end values for bias removal

accel_chg = diff(accel_filt);
len = 0;
start_end_time = [];
for i = 2 : length(accel_chg)
    if abs(accel_chg(i)) < 0.001
        len = len +1;
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
    avg = mean(accel_filt(start_end_time(i-1):start_end_time(i)));
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
        corr_acc(j) = accel_filt(j)- avg;
    end
end

% velocity from corrected
fwd_correct_vel = cumtrapz(imu_time,corr_acc);
fwd_correct_vel(end) = 0;
vel_gps(length(vel_gps)) =0;

% Dead_Reckoning



figure;
plot(imu_time,unwrap(yaw),"DisplayName","IMU-Yaw");
hold on;
plot(imu_time,(filtered_yaw),"DisplayName","filtered-yaw");
hold on;
xlabel('time (s)')
ylabel('yaw (rad)')
title('Yaw (IMU) vs Yaw (complementary filter)')
legend;


%integrate forward vel
imu_disp = cumtrapz(imu_time,fwd_correct_vel);
gps_disp = cumtrapz(gps_time,vel_gps);

figure;
plot(imu_time,imu_disp,"red","DisplayName","displacement (Fwd vel)");
hold on;
plot(gps_time,gps_disp,"blue","DisplayName","displacement (GPS)");
xlabel('time (s)')
ylabel('displacement (m)')
title('Displacement (Fwd vel) vs Displacement (GPS)')
legend;

%  w*x dot vs acc_y
accel_x_obs = corr_acc;
vel_x_obs = fwd_correct_vel;
for  i = 1 : length(omega_z)
    accel_y_obs(i) = omega_z(i)* fwd_correct_vel(i) ;
end

figure;

plot(imu_time,accel_y_obs,"Displayname","𝜔𝑋̇");
hold on;
plot(imu_time,acc_y,"Displayname","𝑦̈𝑜𝑏𝑠");
xlabel('time (s)')
ylabel('acceleration (m/s^2)')
title('𝜔𝑋̇ and 𝑦̈𝑜𝑏𝑠 plotted together')
legend;


% Denote this vector by (ve,vn) 

Vn =[];
Ve= [];
wrap_filter_yaw = wrapToPi(filtered_yaw);

for i = 1:length(wrap_filter_yaw)
    yaw_angle = wrap_filter_yaw(i); 
    Vn(i) = vel_x_obs(i) * cos(yaw_angle);
    Ve(i) = vel_x_obs(i) * sin(yaw_angle);
end

% integrate [Ve, Vn] 
x_e = (cumtrapz(imu_time, Ve))';
x_n = (cumtrapz(imu_time, Vn))';

% correction for magnetometer
utm_easting = UTM_easting - UTM_easting(1);
utm_northing = UTM_northing - UTM_northing(1);

% Heading correction for magnetometer using manual values of the staright line 
% from gps trajectory plot
GPS_slope = (utm_northing(25)-utm_northing(1))/(utm_easting(25) - utm_easting(1));
imu_slope = (x_n(2200)-x_n(1))/(x_e(2200)-x_e(1));

gps_th = atan(GPS_slope) + pi();
imu_th = atan(imu_slope) + pi();

heading = gps_th - imu_th;
rot_mat_head = [cos(heading) -sin(heading); sin(heading) cos(heading)];

rotated_traj = (rot_mat_head * [x_e, x_n]')';

figure;
subplot(2,1,1)
plot(x_e, x_n,"red", utm_easting, utm_northing,"blue");
title("Plot of estimated trajectory with the GPS vs IMU (before adjustment)");
xlabel("X (m)");
ylabel("Y (m)");
legend("imu-traj","gps-traj");
axis('equal')
subplot(2,1,2)
plot(rotated_traj(:,1), rotated_traj(:,2),"red", utm_easting, utm_northing,"blue");
title("Plot of estimated trajectory with the GPS vs IMU (after adjustment)");
xlabel("X (m)");
ylabel("Y (m)");
legend("imu-traj","gps-traj");
axis('equal')











