function comparison = run_all_methods(varargin)
%RUN_ALL_METHODS Run TRIAD, Davenport and SVD and compare Task 7 metrics.
%
% Accepts the same name/value arguments as run_pipeline, except 'method'.
% Writes method_comparison.json/.csv and the cross-method figures into a
% sibling "comparison" directory, using the shared fusion.catalog naming.

methods = {'TRIAD', 'Davenport', 'SVD'};
results = struct();
for i = 1:numel(methods)
    results.(methods{i}) = run_pipeline(varargin{:}, 'method', methods{i});
end

root = fileparts(results.TRIAD.run_dir);
c = fusion.catalog();
comparisonDir = fullfile(root, c.comparison_slug);
if ~exist(comparisonDir, 'dir'); mkdir(comparisonDir); end

metrics = struct();
for i = 1:numel(methods)
    taskList = results.(methods{i}).tasks;
    if numel(taskList) >= 7 && ~isempty(taskList{7})
        metrics.(methods{i}) = taskList{7}.metrics;
    else
        metrics.(methods{i}) = struct();
    end
end

comparison = struct('schema_version', '3.0', 'methods', {methods}, 'metrics', metrics, ...
    'note', ['Task 1-2 inputs are common; Task 3 attitude differs; ' ...
             'Tasks 4-7 consume that method-specific attitude.']);
fusion.write_json(fullfile(comparisonDir, 'method_comparison.json'), comparison);

positionRMSE = nan(3, 1); velocityRMSE = nan(3, 1);
attitudeRMSE = nan(3, 1); heightRMSE = nan(3, 1);
for i = 1:3
    m = metrics.(methods{i});
    if isfield(m, 'position_rmse_m');   positionRMSE(i) = m.position_rmse_m;   end
    if isfield(m, 'velocity_rmse_mps'); velocityRMSE(i) = m.velocity_rmse_mps; end
    if isfield(m, 'attitude_rmse_deg'); attitudeRMSE(i) = m.attitude_rmse_deg; end
    if isfield(m, 'height_rmse_m');     heightRMSE(i)   = m.height_rmse_m;     end
end
tableOut = table(string(methods(:)), positionRMSE, velocityRMSE, heightRMSE, attitudeRMSE, ...
    'VariableNames', {'method', 'position_rmse_m', 'velocity_rmse_mps', ...
                      'height_rmse_m', 'attitude_rmse_deg'});
writetable(tableOut, fullfile(comparisonDir, 'method_comparison.csv'));

cfg = results.TRIAD.manifest.config;
if isfield(cfg, 'plots') && cfg.plots
    figCtx = fusion.figures('context', results.TRIAD.manifest.run_id, ...
        strjoin(methods, '+'), results.TRIAD.figure_context.datasets);
    figCtx = comparison_figures(figCtx, results, methods, metrics, comparisonDir, cfg);
    fusion.figures('write_index', figCtx, comparisonDir);
end

fprintf('All-method comparison complete: %s\n', comparisonDir);
end

% ---------------------------------------------------------------------------
function figCtx = comparison_figures(figCtx, results, methods, metrics, outDir, cfg)

% C.1 initial attitude per method -------------------------------------------
f = figure('Visible', 'off'); tiledlayout(f, 1, 2);
quaternions = nan(numel(methods), 4); errors = nan(numel(methods), 2);
for i = 1:numel(methods)
    taskList = results.(methods{i}).tasks;
    if numel(taskList) >= 3 && ~isempty(taskList{3})
        quaternions(i, :) = taskList{3}.quaternion_wxyz_body_to_ned(:)';
        errors(i, :) = [taskList{3}.gravity_error_deg, taskList{3}.earth_rate_error_deg];
    end
end
nexttile; bar(quaternions'); grid on; ylim([-1.15, 1.15]);
xticklabels({'qw', 'qx', 'qy', 'qz'}); ylabel('Component value');
title('Initial quaternion'); legend(methods, 'Location', 'best');
nexttile; bar(errors'); grid on; set(gca, 'YScale', 'log');
xticklabels({'Gravity', 'Earth rate'}); ylabel('Alignment error [deg]');
title('Wahba residual'); legend(methods, 'Location', 'best');
figCtx = fusion.figures('save', figCtx, 0, 'initial_attitude_by_method', outDir, f);

% C.2 fused position per method ---------------------------------------------
f = figure('Visible', 'off'); tiledlayout(f, 1, 3);
labels = {'North', 'East', 'Down'};
axesHandles = gobjects(1, 3);
for j = 1:3
    axesHandles(j) = nexttile; hold(axesHandles(j), 'on'); grid(axesHandles(j), 'on');
    title(axesHandles(j), labels{j});
    xlabel(axesHandles(j), 'Time [s]');
    if j == 1; ylabel(axesHandles(j), 'Position [m]'); end
end
for i = 1:numel(methods)
    taskList = results.(methods{i}).tasks;
    if numel(taskList) < 5 || isempty(taskList{5}); continue; end
    t = taskList{5}.time_s;
    idx = 1:max(1, ceil(numel(t) / cfg.max_plot_points)):numel(t);
    for j = 1:3
        plot(axesHandles(j), t(idx), taskList{5}.position(idx, j), 'DisplayName', methods{i});
    end
end
legend(axesHandles(1), 'Location', 'best');
figCtx = fusion.figures('save', figCtx, 0, 'fused_position_by_method', outDir, f);

% C.3 / C.4 truth-dependent comparisons --------------------------------------
hasOverlay = false; hasAttitude = false;
for i = 1:numel(methods)
    taskList = results.(methods{i}).tasks;
    if numel(taskList) >= 6 && ~isempty(taskList{6}) && strcmp(taskList{6}.status, 'complete')
        hasOverlay = true;
        if ~isempty(taskList{6}.overlay.truth_quaternion); hasAttitude = true; end
    end
end

if hasOverlay
    f = figure('Visible', 'off'); tiledlayout(f, 2, 1);
    axTop = nexttile; hold(axTop, 'on'); grid(axTop, 'on');
    ylabel(axTop, '|position error| [m]');
    axBottom = nexttile; hold(axBottom, 'on'); grid(axBottom, 'on');
    ylabel(axBottom, 'Height error [m]'); xlabel(axBottom, 'Time [s]');
    for i = 1:numel(methods)
        taskList = results.(methods{i}).tasks;
        if numel(taskList) < 6 || isempty(taskList{6}) || ~strcmp(taskList{6}.status, 'complete')
            continue
        end
        o = taskList{6}.overlay;
        idx = 1:max(1, ceil(numel(o.time_s) / cfg.max_plot_points)):numel(o.time_s);
        err = o.estimated_position - o.truth_position;
        plot(axTop, o.time_s(idx), vecnorm(err(idx, :), 2, 2), 'DisplayName', methods{i});
        plot(axBottom, o.time_s(idx), -err(idx, 3), 'DisplayName', methods{i});
    end
    legend(axTop, 'Location', 'best');
    figCtx = fusion.figures('save', figCtx, 0, 'position_error_by_method', outDir, f);
else
    figCtx = fusion.figures('skip', figCtx, 0, 'position_error_by_method', ...
        'no truth overlay available');
end

if hasOverlay && hasAttitude
    f = figure('Visible', 'off'); tiledlayout(f, 1, 1);
    ax = nexttile; hold(ax, 'on'); grid(ax, 'on');
    xlabel(ax, 'Time [s]'); ylabel(ax, 'Attitude error [deg]');
    for i = 1:numel(methods)
        taskList = results.(methods{i}).tasks;
        if numel(taskList) < 6 || isempty(taskList{6}) || ~strcmp(taskList{6}.status, 'complete')
            continue
        end
        o = taskList{6}.overlay;
        if isempty(o.truth_quaternion); continue; end
        idx = 1:max(1, ceil(numel(o.time_s) / cfg.max_plot_points)):numel(o.time_s);
        dots = min(1, abs(sum(o.estimated_quaternion .* o.truth_quaternion, 2)));
        plot(ax, o.time_s(idx), 2 * acosd(dots(idx)), 'DisplayName', methods{i});
    end
    legend(ax, 'Location', 'best');
    figCtx = fusion.figures('save', figCtx, 0, 'attitude_error_by_method', outDir, f);
else
    figCtx = fusion.figures('skip', figCtx, 0, 'attitude_error_by_method', ...
        'no truth attitude available');
end

% C.5 metric bars ------------------------------------------------------------
keys = {};
for i = 1:numel(methods)
    names = fieldnames(metrics.(methods{i}));
    for k = 1:numel(names)
        value = metrics.(methods{i}).(names{k});
        if isnumeric(value) && isscalar(value) && ~any(strcmp(keys, names{k}))
            keys{end+1} = names{k}; %#ok<AGROW>
        end
    end
end
keys = sort(keys);
f = figure('Visible', 'off');
if isempty(keys)
    tiledlayout(f, 1, 1); nexttile; axis off;
    text(0.5, 0.5, 'no comparable metrics', 'HorizontalAlignment', 'center');
else
    columns = min(4, numel(keys));
    rows = ceil(numel(keys) / columns);
    tiledlayout(f, rows, columns);
    for k = 1:numel(keys)
        values = nan(1, numel(methods));
        for i = 1:numel(methods)
            if isfield(metrics.(methods{i}), keys{k})
                values(i) = metrics.(methods{i}).(keys{k});
            end
        end
        nexttile; bar(categorical(methods, methods), values); grid on;
        title(keys{k}, 'Interpreter', 'none', 'FontSize', 9);
    end
end
figCtx = fusion.figures('save', figCtx, 0, 'metric_bars_by_method', outDir, f);
end
