clc;
close all;

%open bag file
bag = rosbag('/home/marley/catkin_ws/src/lab4_driver/bag_files/data_going_in_circles.bag');

%rosbag info 'data_going_in_circles.bag';

bsel = select(bag,'Topic','/imu');
msgStructs = readMessages(bsel,'DataFormat','struct');
Mag_x = cellfun(@(m) double(m.MagField.MagneticField_.X),msgStructs);
Mag_y = cellfun(@(m) double(m.MagField.MagneticField_.Y),msgStructs);
mag = [Mag_x,Mag_y];


% fit ellipse
[a,b,orientation_rad,x0,y0] = fit_ellipse(Mag_x,Mag_y);

%Hard iron distortion
offset_x = x0;
offset_y = y0;
mag_hard_corrected_x = Mag_x - offset_x;
mag_hard_corrected_y = Mag_y - offset_y;

%soft iron distortion
theta = orientation_rad;
R = [cos(theta) sin(theta);-sin(theta) cos(theta)];
rotated_mag = (R*[mag_hard_corrected_x,mag_hard_corrected_y]')';

scale_factor = b/a;
scale_matrix= [scale_factor 0;0 1];
scaled_mag = (scale_matrix*rotated_mag')';

theta = -theta;
R_back = [cos(theta) sin(theta);-sin(theta) cos(theta)];
corrected_mag= (R_back*scaled_mag')';

figure;
scatter(corrected_mag(:,1),corrected_mag(:,2), 10 ,"filled","DisplayName","After calibration");
hold on;
scatter(Mag_x, Mag_y, 10, "filled", "DisplayName", "Before calibration");
hold on;
xlabel("Magnetic Field X (Gauss)");
ylabel("Magnetic Field Y (Gauss)");
axis equal;

title('Plot - Magnetometer X-Y plot before and after hard and soft iron calibration');
legend;


SoftIronRotationBack = R_back*scale_matrix*R;


imu_timePoints_sec = cellfun(@(m) double(m.Header.Stamp.Sec),msgStructs);
imu_timePoints_nanosec = cellfun(@(m) double(m.Header.Stamp.Nsec),msgStructs);
imu_timePoints = double(imu_timePoints_sec + ( imu_timePoints_nanosec * 10^(-9)));
imu_time = imu_timePoints - imu_timePoints(1);


figure;
scatter(imu_time, Mag_x, 10 ,"filled","DisplayName","Before Calibration");
hold on;
scatter(imu_time, corrected_mag(:,1), 10, "filled", "DisplayName", "After calibration");
hold on;
xlabel("Time (s)");
ylabel("Magnetic Field X (Gauss)");
title('Plot - time series Magnetic Field X  before and after the correction');
legend;

figure;
scatter(imu_time, Mag_y, 10 ,"filled","DisplayName","Before Calibration");
hold on;
scatter(imu_time, corrected_mag(:,2), 10, "filled", "DisplayName", "After calibration");
hold on;
xlabel("Time (s)");
ylabel("Magnetic Field Y (Gauss)");
title('Plot - time series Magnetic Field Y before and after the correction');
legend;

%--------------------------------------------------------------------------------------------

function [a,b,orientation_rad,X0,Y0] = fit_ellipse(x,y,axis_handle)
% initialize
orientation_tolerance = 1e-3;

% empty warning stack
warning( '' );

% prepare vectors, must be column vectors
x = x(:);
y = y(:);

% remove bias of the ellipse - to make matrix inversion more accurate. (will be added later on).
mean_x = mean(x);
mean_y = mean(y);
x = x-mean_x;
y = y-mean_y;

% the estimation for the conic equation of the ellipse
X = [x.^2, x.*y, y.^2, x, y ];
a = sum(X)/(X'*X);

% check for warnings
if ~isempty( lastwarn )
    disp( 'stopped because of a warning regarding matrix inversion' );
    ellipse_t = [];
    return
end

% extract parameters from the conic equation
[a,b,c,d,e] = deal( a(1),a(2),a(3),a(4),a(5) );

% remove the orientation from the ellipse
if ( min(abs(b/a),abs(b/c)) > orientation_tolerance )
    
    orientation_rad = 1/2 * atan( b/(c-a) );
    cos_phi = cos( orientation_rad );
    sin_phi = sin( orientation_rad );
    [a,b,c,d,e] = deal(...
        a*cos_phi^2 - b*cos_phi*sin_phi + c*sin_phi^2,...
        0,...
        a*sin_phi^2 + b*cos_phi*sin_phi + c*cos_phi^2,...
        d*cos_phi - e*sin_phi,...
        d*sin_phi + e*cos_phi );
    [mean_x,mean_y] = deal( ...
        cos_phi*mean_x - sin_phi*mean_y,...
        sin_phi*mean_x + cos_phi*mean_y );
else
    orientation_rad = 0;
    cos_phi = cos( orientation_rad );
    sin_phi = sin( orientation_rad );
end

% check if conic equation represents an ellipse
test = a*c;
switch (1)
case (test>0),  status = '';
case (test==0), status = 'Parabola found';  warning( 'fit_ellipse: Did not locate an ellipse' );
case (test<0),  status = 'Hyperbola found'; warning( 'fit_ellipse: Did not locate an ellipse' );
end

% if we found an ellipse return it's data
if (test>0)
    
    % make sure coefficients are positive as required
    if (a<0), [a,c,d,e] = deal( -a,-c,-d,-e ); end
    
    % final ellipse parameters
    X0          = mean_x - d/2/a;
    Y0          = mean_y - e/2/c;
    F           = 1 + (d^2)/(4*a) + (e^2)/(4*c);
    [a,b]       = deal( sqrt( F/a ),sqrt( F/c ) );    
    long_axis   = 2*max(a,b);
    short_axis  = 2*min(a,b);

    % rotate the axes backwards to find the center point of the original TILTED ellipse
    R           = [ cos_phi sin_phi; -sin_phi cos_phi ];
    P_in        = R * [X0;Y0];
    X0_in       = P_in(1);
    Y0_in       = P_in(2);
    
    % pack ellipse into a structure
    ellipse_t = struct( ...
        'a',a,...
        'b',b,...
        'phi',orientation_rad,...
        'X0',X0,...
        'Y0',Y0,...
        'X0_in',X0_in,...
        'Y0_in',Y0_in,...
        'long_axis',long_axis,...
        'short_axis',short_axis,...
        'status','' );
else
    % report an empty structure
    ellipse_t = struct( ...
        'a',[],...
        'b',[],...
        'phi',[],...
        'X0',[],...
        'Y0',[],...
        'X0_in',[],...
        'Y0_in',[],...
        'long_axis',[],...
        'short_axis',[],...
        'status',status );
end

% check if we need to plot an ellipse with it's axes.
if (nargin>2) & ~isempty( axis_handle ) & (test>0)
    
    % rotation matrix to rotate the axes with respect to an angle phi
    R = [ cos_phi sin_phi; -sin_phi cos_phi ];
    
    % the axes
    ver_line        = [ [X0 X0]; Y0+b*[-1 1] ];
    horz_line       = [ X0+a*[-1 1]; [Y0 Y0] ];
    new_ver_line    = R*ver_line;
    new_horz_line   = R*horz_line;
    
    % the ellipse
    theta_r         = linspace(0,2*pi);
    ellipse_x_r     = X0 + a*cos( theta_r );
    ellipse_y_r     = Y0 + b*sin( theta_r );
    rotated_ellipse = R * [ellipse_x_r;ellipse_y_r];
    
    % draw
    hold_state = get( axis_handle,'NextPlot' );
    set( axis_handle,'NextPlot','add' );
    plot( new_ver_line(1,:),new_ver_line(2,:),'r' );
    plot( new_horz_line(1,:),new_horz_line(2,:),'r' );
    plot( rotated_ellipse(1,:),rotated_ellipse(2,:),'r' );
    set( axis_handle,'NextPlot',hold_state );
end
end