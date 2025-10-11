function plotAircraftTrajectoryAndAttitudes(ac_data1, trange)
 % Read STL file
fv = stlread('Cyclone2_whole.stl');

% Extract vertex and face data
vertices = fv.Points;             % 'Points' contains vertex coordinates
faces = fv.ConnectivityList;      % 'ConnectivityList' contains faces
clear fv;

% Apply base rotation matrix (to map STL coordinates properly)
R = [0 -1 0;
     0 0 -1;
     1 0 0];

% Transform vertices by the base rotation
converted_vertices = (R * vertices')';
clear vertices;
patched_data = struct('faces', faces, 'vertices', converted_vertices);

% Slice attitude data within the time range
datarange1 = find(ac_data1.AHRS_REF_QUAT.timestamp>trange(1),1,'first')-1;
datarange2 = find(ac_data1.AHRS_REF_QUAT.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;
clear datarange1 datarange2;
attitude_timestamps = ac_data1.AHRS_REF_QUAT.timestamp(datarange);
quat1 = double([ac_data1.AHRS_REF_QUAT.body_qi(datarange) ...
                ac_data1.AHRS_REF_QUAT.body_qx(datarange) ...
                ac_data1.AHRS_REF_QUAT.body_qy(datarange) ...
                ac_data1.AHRS_REF_QUAT.body_qz(datarange)]);
clear datarange;

% Slice position data within the time range
datarange1 = find(ac_data1.ROTORCRAFT_FP.timestamp>trange(1),1,'first')-1;
datarange2 = find(ac_data1.ROTORCRAFT_FP.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;
clear datarange1 datarange2;
pos_timestamps = ac_data1.ROTORCRAFT_FP.timestamp(datarange);
x = ac_data1.ROTORCRAFT_FP.east_alt(datarange);
y = ac_data1.ROTORCRAFT_FP.north_alt(datarange);
z = ac_data1.ROTORCRAFT_FP.up_alt(datarange);
clear datarange;

% Define the UTM projection (kept here for potential waypoint/GPS use)
utmstruct = defaultm('utm');
utmstruct.zone = '31U';
utmstruct.geoid = wgs84Ellipsoid;
utmstruct = defaultm(utmstruct);

% Slice GPS data within the time range
datarange1 = find(ac_data1.GPS_INT.timestamp>213.3,1,'first')-1;
datarange2 = find(ac_data1.GPS_INT.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;
clear datarange1 datarange2;
GPS_timestamps = ac_data1.GPS_INT.timestamp(datarange);
lat = ac_data1.GPS_INT.lat_alt(datarange);
lon = ac_data1.GPS_INT.lon_alt(datarange);
clear datarange;

% % Example of converting GPS waypoints to local meters
% % Get origin from the first GPS coordinate
% [origin_x, origin_y] = projfwd(utmstruct, lat(1), lon(1));
% [waypoints1_x,waypoints1_y] = projfwd(utmstruct, 52.168580, 4.414417);
% % Convert waypoint GPS to meters relative to origin
% waypoints1_meters = [waypoints1_x - origin_x, waypoints1_y - origin_y, 26.6];
% [waypoints2_x,waypoints2_y] = projfwd(utmstruct, 52.168169, 4.412862);
% waypoints2_meters = [waypoints2_x - origin_x, waypoints2_y - origin_y, 26.6];

% Slice airspeed within the time range
datarange1 = find(ac_data1.AIR_DATA.timestamp>trange(1),1,'first')-1;
datarange2 = find(ac_data1.AIR_DATA.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;
clear datarange1 datarange2;
datarange1 = find(ac_data1.AIR_DATA.timestamp>trange(1),1,'first')-1;
datarange2 = find(ac_data1.AIR_DATA.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;
clear datarange1 datarange2;
airspeed_timestamps = ac_data1.AIR_DATA.timestamp(datarange);
airspeed = ac_data1.AIR_DATA.airspeed(datarange);
clear datarange;

% Slice INS velocity within the time range
datarange1 = find(ac_data1.INS.timestamp>trange(1),1,'first')-1;
datarange2 = find(ac_data1.INS.timestamp>trange(2),1,'first')-1;
datarange = datarange1:datarange2;
clear datarange1 datarange2;
vz_timestamps = ac_data1.INS.timestamp(datarange);
vz = - ac_data1.INS.ins_zd_alt(datarange);
V = sqrt(ac_data1.INS.ins_xd_alt(datarange).^2 + ac_data1.INS.ins_yd_alt(datarange).^2 + ac_data1.INS.ins_zd_alt(datarange).^2);
clear datarange;

% Interpolate airspeed / vz / |V| to match pos_timestamps
airspeed_interp = interp1(airspeed_timestamps, airspeed, pos_timestamps, 'linear', 'extrap');
vz_interp = interp1(vz_timestamps, vz, pos_timestamps, 'linear', 'extrap');
V_interp = interp1(vz_timestamps, V, pos_timestamps, 'linear', 'extrap');

figure;
% Create an axes for plotting the trajectory/model
ax = axes;

% % Example: show two waypoints as red dots (kept commented)
% plot3(waypoints1_meters(1), waypoints1_meters(2), waypoints1_meters(3), ...
%       'r.', 'MarkerSize', 30); hold on;
% plot3(waypoints2_meters(1), waypoints2_meters(2), waypoints2_meters(3), ...
%       'r.', 'MarkerSize', 30);

% Choose the set of timestamps at which to place the STL model
% selected_times = start_time:time_interval:max(attitude_timestamps);
% selected_times = [163.066, 172.9361, 177.4361,179.6861, 181.6861, 192.6861, 198.9361,209.1861,220.4361,233.6861,239.6861,244.4361,250.9361];
% Figure 9a preset:
% selected_times = [126.8, 128.8, 130.8, 132.8, 133.3, 133.8, 134.3,134.8,135.3];
% Figure 9c preset:
selected_times = [250 300];
% selected_times = [477.5, 480.5,483.5,486.5,489.5,491.015];
% selected_times = [224.323, 227.323,230.323,233.323,236.323,239.323,242.323,245.073];

% Render the STL model at each selected timestamp
for t = 1:length(selected_times)
    % Find nearest trajectory timestamp
    [~, idx] = min(abs(pos_timestamps - selected_times(t)));

    % Corresponding trajectory point
    x_pos = x(idx);
    y_pos = y(idx);
    z_pos = z(idx);

    % Find nearest attitude timestamp
    [~, attitude_idx] = min(abs(attitude_timestamps - selected_times(t)));

    % Corresponding quaternion
    qw = quat1(attitude_idx, 1);
    qx = quat1(attitude_idx, 2);
    qy = quat1(attitude_idx, 3);
    qz = quat1(attitude_idx, 4);

    % Convert quaternion to rotation matrix (order: [qw, qx, qy, qz])
    R = quat2rotm([qw, qx, qy, qz]);
    clear qx qy qz qw;

    % Rotate STL vertices by attitude
    rotated_vertices = (R * patched_data.vertices')';

% Optional additional body-frame rotations (Z then X)
R_zb = makehgtform('zrotate', deg2rad(-90));
R_zb = R_zb(1:3, 1:3);            % extract 3x3 rotation
% Apply the Z-rotation
mid_vertices = (R_zb * rotated_vertices')';
clear R_zb rotated_vertices;

R_xb = makehgtform('xrotate', deg2rad(180));
R_xb = R_xb(1:3, 1:3);            % extract 3x3 rotation
% Apply the X-rotation and scale
final_vertices = 0.03*(R_xb * mid_vertices')';
clear R_xb mid_vertices;

% Translate the model to the selected trajectory point
translated_vertices = final_vertices + repmat([x_pos, y_pos, z_pos], size(final_vertices, 1), 1);

% Prepare per-vertex colors (same RGB for all vertices)
clear final_vertices;
colors = repmat([190 190 190]/255, size(translated_vertices, 1), 1);

patch('Faces', faces, 'Vertices', translated_vertices, ...
      'FaceVertexCData', colors, ...   % per-vertex color map
      'FaceColor', 'interp', ...       % interpolate colors
      'EdgeColor', 'none', ...
      'FaceAlpha', 0.8,'Parent', ax);
  clear colors translated_vertices;
end

% Lighting/material options (kept as examples, commented)
% material([0.3 0.4 0.1 10]);  % [ambient, diffuse, specular, shininess]
% camlight(ax, 'headlight');
% lighting(ax, 'gouraud');
% shading(ax, 'interp');       % smooth shading
% hold on;

% Plot the colored 3D trajectory (here: colored by interpolated v_z)
% surface([x'; x'], [y'; y'], [z'; z'], [airspeed_interp'; airspeed_interp'], ...
%     'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 3, 'Parent', ax);
surface([x'; x'], [y'; y'], [z'; z'], [vz_interp'; vz_interp'], ...
    'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 3, 'Parent', ax);
% surface([x'; x'], [y'; y'], [z'; z'], [V_interp'; V_interp'], ...
%     'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 3, 'Parent', ax);

% Colormap and colorbar
colormap(ax, jet);
colorbar('peer', ax);
% caxis([0 17]);

% Store data in the figure for the datacursor callback
setappdata(gcf, 'pos_timestamps', pos_timestamps);
setappdata(gcf, 'x_data', x);
setappdata(gcf, 'y_data', y);
setappdata(gcf, 'z_data', z);

% Axes labels and styling
xlabel(ax, 'X [m]', 'FontSize', 14);
ylabel(ax, 'Y [m]', 'FontSize', 14);
zlabel(ax, 'Z [m]', 'FontSize', 14);
% zlim([0,9]);
xlim([-3,3]);
ylim([-3,3]);
set(gca,'FontName','Times New Roman','FontSize',14);
axis(ax, 'equal');
axis tight;
grid(ax, 'on');
view(3);

% Optional title
% title(ax, 'Aircraft Attitudes at Selected Times Along Trajectory');

% Enable datacursor and use custom callback to display time
dcm = datacursormode(gcf);
set(dcm, 'UpdateFcn', @displayTime);

% Export figure to PDF (legend_airspeed_turn.pdf) at 600 dpi
print('legend_airspeed_turn', '-dpdf', '-r600')

end