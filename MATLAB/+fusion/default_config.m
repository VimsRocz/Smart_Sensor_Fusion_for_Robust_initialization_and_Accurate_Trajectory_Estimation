function cfg = default_config(overrides)
%DEFAULT_CONFIG Return validated settings shared by Tasks 1-7.

cfg = struct( ...
    'imu_measurement_type', 'delta', ...
    'static_samples', 400, ...
    'gravity_override_mps2', [], ...
    'earth_rate_rps', 7.292115e-5, ...
    'gravity_weight', 0.9999, ...
    'earth_rate_weight', 0.0001, ...
    'process_accel_std_mps2', 20.0, ...
    'gnss_position_std_m', 2.0, ...
    'gnss_velocity_std_mps', 0.5, ...
    'max_specific_force_mps2', 100.0, ...
    'max_angular_rate_rps', 20.0, ...
    'truth_quaternion_order', 'xyzw', ...
    'truth_quaternion_frame', 'body_to_ecef', ...
    'truth_attitude_time_offset_s', -0.05, ...
    'max_plot_points', 50000, ...
    'plots', true);

if nargin > 0 && ~isempty(overrides)
    if ~isstruct(overrides)
        error('fusion:Config', 'config must be a struct.');
    end
    names = fieldnames(overrides);
    for i = 1:numel(names)
        if ~isfield(cfg, names{i})
            error('fusion:Config', 'Unknown config key: %s', names{i});
        end
        cfg.(names{i}) = overrides.(names{i});
    end
end

if ~ismember(lower(string(cfg.imu_measurement_type)), ["delta", "rate"])
    error('fusion:Config', 'imu_measurement_type must be delta or rate.');
end
if ~ismember(lower(string(cfg.truth_quaternion_order)), ["wxyz", "xyzw"])
    error('fusion:Config', 'truth_quaternion_order must be wxyz or xyzw.');
end
if ~ismember(lower(string(cfg.truth_quaternion_frame)), ["body_to_ned", "body_to_ecef"])
    error('fusion:Config', 'truth_quaternion_frame must be body_to_ned or body_to_ecef.');
end
if ~isfinite(cfg.truth_attitude_time_offset_s) || abs(cfg.truth_attitude_time_offset_s)>10
    error('fusion:Config', 'truth_attitude_time_offset_s must be finite and within +/-10 s.');
end
if cfg.static_samples < 20
    error('fusion:Config', 'static_samples must be at least 20.');
end
positive = {'earth_rate_rps','gravity_weight', ...
    'earth_rate_weight','process_accel_std_mps2','gnss_position_std_m', ...
    'gnss_velocity_std_mps','max_specific_force_mps2','max_angular_rate_rps','max_plot_points'};
for i = 1:numel(positive)
    if cfg.(positive{i}) <= 0
        error('fusion:Config', '%s must be positive.', positive{i});
    end
end
if ~isempty(cfg.gravity_override_mps2) && cfg.gravity_override_mps2<=0
    error('fusion:Config', 'gravity_override_mps2 must be positive when supplied.');
end
end
