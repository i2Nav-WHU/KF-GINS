function naverror = plotNavError(navresult_filepath, refresult_filepath)
%PLOTNAVERROR Calculate and plot KF-GINS navigation errors.
%
%   plotNavError()
%   plotNavError(navresult_filepath, refresult_filepath)
%   naverror = plotNavError(...)
%
% Expected columns in both navigation files:
%   1      GPS week
%   2      GPS seconds of week
%   3:5    latitude [deg], longitude [deg], height [m]
%   6:8    north/east/down velocity [m/s]
%   9:11   roll/pitch/yaw [deg]
%
% The returned naverror matrix uses the same column layout, except that
% columns 3:5 are north/east/down position errors in metres.

    script_dir = fileparts(mfilename('fullpath'));
    dataset_dir = fullfile(script_dir, '..', 'dataset');

    if nargin < 1 || isempty(navresult_filepath)
        navresult_filepath = fullfile(dataset_dir, 'KF_GINS_Navresult.nav');
    end
    if nargin < 2 || isempty(refresult_filepath)
        refresult_filepath = fullfile(dataset_dir, 'truth.nav');
    end

    if ~isfile(navresult_filepath)
        error('Navigation result file not found: %s', navresult_filepath);
    end
    if ~isfile(refresult_filepath)
        error('Reference result file not found: %s', refresult_filepath);
    end

    navresult = readmatrix(navresult_filepath, 'FileType', 'text');
    refresult = readmatrix(refresult_filepath, 'FileType', 'text');
    if size(navresult, 2) < 11 || size(refresult, 2) < 11
        error('Navigation and reference results must contain at least 11 columns.');
    end

    % Remove possible completely empty rows introduced by text import.
    navresult = navresult(~all(isnan(navresult), 2), :);
    refresult = refresult(~all(isnan(refresult), 2), :);

    % Unwrap yaw in degrees before interpolation and subtraction.
    navresult(:, 11) = rad2deg(unwrap(deg2rad(navresult(:, 11))));
    refresult(:, 11) = rad2deg(unwrap(deg2rad(refresult(:, 11))));

    % interp1 requires unique sample times. Keep the last reference sample
    % at duplicate epochs, matching NumPy's effective handling in this data.
    [~, unique_ref_index] = unique(refresult(:, 2), 'last');
    refresult = refresult(unique_ref_index, :);

    % Retain only the common time interval.
    start_time = max(navresult(1, 2), refresult(1, 2));
    end_time = min(navresult(end, 2), refresult(end, 2));
    if start_time > end_time
        error('Navigation and reference results have no overlapping time interval.');
    end

    use_nav = navresult(:, 2) >= start_time & navresult(:, 2) <= end_time;
    navresult = navresult(use_nav, :);
    if isempty(navresult)
        error('No navigation samples exist in the overlapping time interval.');
    end

    % Convert latitude and longitude from degrees to radians.
    navresult(:, 3:4) = deg2rad(navresult(:, 3:4));
    refresult(:, 3:4) = deg2rad(refresult(:, 3:4));

    % Interpolate the reference solution to each navigation-result epoch.
    refinter = zeros(size(navresult));
    refinter(:, 2) = navresult(:, 2);
    for col = 3:11
        refinter(:, col) = interp1(refresult(:, 2), refresult(:, col), ...
                                   navresult(:, 2), 'linear');
    end

    % Calculate position, velocity and attitude differences.
    naverror = zeros(size(navresult));
    naverror(:, 2) = navresult(:, 2);
    naverror(:, 3:11) = navresult(:, 3:11) - refinter(:, 3:11);

    % Wrap yaw error into [-180, 180] degrees without requiring a toolbox.
    naverror(:, 11) = mod(naverror(:, 11) + 180, 360) - 180;

    % Convert latitude/longitude/height errors to local NED errors in metres.
    origin = navresult(1, 3:5);
    [rm, rn] = radiusmn(origin(1));
    north_error = naverror(:, 3) .* (rm + origin(3));
    east_error = naverror(:, 4) .* (rn + origin(3)) .* cos(origin(1));
    down_error = -naverror(:, 5);
    naverror(:, 3:5) = [north_error, east_error, down_error];

    fprintf('Navigation result error calculation finished.\n');
    fprintf('Plotting navigation errors.\n');

    figure('Name', 'Position Error');
    plot(naverror(:, 2), naverror(:, 3:5), 'LineWidth', 1.0);
    legend('North', 'East', 'Down', 'Location', 'best');
    xlabel('Time [s]');
    ylabel('Error [m]');
    title('Position Error');
    grid on;

    figure('Name', 'Velocity Error');
    plot(naverror(:, 2), naverror(:, 6:8), 'LineWidth', 1.0);
    legend('North', 'East', 'Down', 'Location', 'best');
    xlabel('Time [s]');
    ylabel('Error [m/s]');
    title('Velocity Error');
    grid on;

    figure('Name', 'Attitude Error');
    plot(naverror(:, 2), naverror(:, 9:11), 'LineWidth', 1.0);
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    xlabel('Time [s]');
    ylabel('Error [deg]');
    title('Attitude Error');
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
