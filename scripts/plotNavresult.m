function plotNavresult(navresult_filepath)
%PLOTNAVRESULT Plot the KF-GINS navigation result with MATLAB.
%
%   plotNavresult()
%   plotNavresult(navresult_filepath)
%
% The navigation result columns are expected to be:
%   1      GPS week
%   2      GPS seconds of week
%   3:5    latitude [deg], longitude [deg], height [m]
%   6:8    north/east/down velocity [m/s]
%   9:11   roll/pitch/yaw [deg]

% If no path is supplied, the result file in ../dataset is used. The
% default path is resolved relative to this M-file, not MATLAB's current
% working directory.

    if nargin < 1 || isempty(navresult_filepath)
        script_dir = fileparts(mfilename('fullpath'));
        navresult_filepath = fullfile(script_dir, '..', 'dataset', ...
                                      'KF_GINS_Navresult.nav');
    end

    if ~isfile(navresult_filepath)
        error('Navigation result file not found: %s', navresult_filepath);
    end

    navresult = readmatrix(navresult_filepath, 'FileType', 'text');
    if size(navresult, 2) < 11
        error('Navigation result must contain at least 11 columns.');
    end

    % Convert latitude and longitude from degrees to radians.
    navresult(:, 3:4) = deg2rad(navresult(:, 3:4));

    % Use the first navigation position as the local NED origin.
    origin = navresult(1, 3:5);
    [rm, rn] = radiusmn(origin(1));
    delta_blh = navresult(:, 3:5) - origin;

    north = delta_blh(:, 1) .* (rm + origin(3));
    east  = delta_blh(:, 2) .* (rn + origin(3)) .* cos(origin(1));

    fprintf('Plotting estimated navigation result.\n');

    figure('Name', 'Horizontal Position');
    plot(east, north, 'LineWidth', 1.0);
    axis equal;
    xlabel('East [m]');
    ylabel('North [m]');
    title('Horizontal Position');
    grid on;

    figure('Name', 'Height');
    plot(navresult(:, 2), navresult(:, 5), 'LineWidth', 1.0);
    xlabel('Time [s]');
    ylabel('Height [m]');
    title('Height');
    grid on;

    figure('Name', 'Velocity');
    plot(navresult(:, 2), navresult(:, 6:8), 'LineWidth', 1.0);
    legend('North', 'East', 'Down', 'Location', 'best');
    xlabel('Time [s]');
    ylabel('Velocity [m/s]');
    title('Velocity');
    grid on;

    figure('Name', 'Attitude');
    plot(navresult(:, 2), navresult(:, 9:11), 'LineWidth', 1.0);
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Attitude');
    grid on;
end


function [rm, rn] = radiusmn(lat)
%RADIUSMN Compute WGS-84 meridian and prime-vertical radii.
    wgs84_ra = 6378137.0;
    wgs84_e1 = 0.00669437999013;

    temp = 1.0 - wgs84_e1 * sin(lat)^2;
    sqrt_temp = sqrt(temp);
    rm = wgs84_ra * (1.0 - wgs84_e1) / (sqrt_temp * temp);
    rn = wgs84_ra / sqrt_temp;
end
