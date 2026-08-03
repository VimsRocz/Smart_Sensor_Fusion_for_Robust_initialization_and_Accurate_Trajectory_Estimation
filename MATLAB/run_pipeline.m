function result = run_pipeline(varargin)
%RUN_PIPELINE Canonical MATLAB entry point for Tasks 1-7.
%
% result = run_pipeline('imu', path, 'gnss', path, 'truth', path, ...
%   'method', 'TRIAD', 'tasks', '1-7', 'output', 'results', 'config', struct())
%
% Directory names and figure filenames come from fusion.catalog, so they match
% the Python pipeline exactly. Each run writes figures_index.csv/.json listing
% every catalog figure with its task, subtask, coordinate frame and datasets.

matlabRoot = fileparts(mfilename('fullpath'));
repoRoot = fileparts(matlabRoot);
addpath(matlabRoot);

p = inputParser; p.FunctionName = 'run_pipeline';
addParameter(p, 'imu', fullfile(repoRoot, 'DATA', 'IMU', 'IMU_X001_small.dat'));
addParameter(p, 'gnss', fullfile(repoRoot, 'DATA', 'GNSS', 'GNSS_X001_small.csv'));
addParameter(p, 'truth', fullfile(repoRoot, 'DATA', 'Truth', 'STATE_X001_small.txt'));
addParameter(p, 'method', 'TRIAD');
addParameter(p, 'tasks', '1-7');
addParameter(p, 'output', fullfile(repoRoot, 'results', 'matlab'));
addParameter(p, 'run_id', '');
addParameter(p, 'config', struct());
parse(p, varargin{:});
args = p.Results;

cfg = fusion.default_config(args.config);
[requested, expanded] = fusion.parse_tasks(args.tasks);

method = char(string(args.method));
valid = {'TRIAD', 'Davenport', 'SVD'};
index = find(strcmpi(method, valid), 1);
if isempty(index)
    error('fusion:Method', 'Method must be TRIAD, Davenport or SVD.');
end
method = valid{index};

data = fusion.read_inputs(args.imu, args.gnss, args.truth, cfg);

[~, imuStem] = fileparts(args.imu);
[~, gnssStem] = fileparts(args.gnss);
if strlength(string(args.run_id)) == 0
    runId = [imuStem '__' gnssStem];
else
    runId = char(args.run_id);
end
runId = regexprep(runId, '[^A-Za-z0-9_.-]', '_');

runDir = fullfile(args.output, runId, lower(method));
if ~exist(runDir, 'dir'); mkdir(runDir); end

truthStem = '';
if ~isempty(data.truth)
    [~, truthStem] = fileparts(args.truth);
end
figCtx = fusion.figures('context', runId, method, ...
    struct('imu', imuStem, 'gnss', gnssStem, 'truth', truthStem));

started = datetime('now', 'TimeZone', 'UTC');
manifest = struct('schema_version', '3.0', 'status', 'running', ...
    'run_id', runId, 'method', method, 'requested_tasks', requested, ...
    'executed_tasks', expanded, 'auto_dependencies', setdiff(expanded, requested), ...
    'started_utc', char(started), ...
    'inputs', struct('imu', char(args.imu), 'gnss', char(args.gnss), 'truth', char(args.truth)), ...
    'dataset_tags', figCtx.datasets, 'config', cfg);
fusion.write_json(fullfile(runDir, 'manifest.json'), manifest);

tasks = cell(1, max(expanded));
last = max(expanded);

[tasks{1}, figCtx] = fusion.task1(data, cfg, task_dir(runDir, 1), figCtx);
if last >= 2
    [tasks{2}, figCtx] = fusion.task2(data.imu, tasks{1}, cfg, task_dir(runDir, 2), figCtx);
end
if last >= 3
    [tasks{3}, figCtx] = fusion.task3(method, tasks{1}, tasks{2}, cfg, task_dir(runDir, 3), figCtx);
end
if last >= 4
    [tasks{4}, figCtx] = fusion.task4(data.imu, data.gnss, tasks{1}, tasks{3}, cfg, task_dir(runDir, 4), figCtx);
end
if last >= 5
    [tasks{5}, figCtx] = fusion.task5(data.imu, data.gnss, tasks{1}, tasks{4}, cfg, task_dir(runDir, 5), figCtx);
end
if last >= 6
    [tasks{6}, figCtx] = fusion.task6(data.truth, tasks{1}, tasks{5}, cfg, task_dir(runDir, 6), figCtx);
end
if last >= 7
    [tasks{7}, figCtx] = fusion.task7(tasks{5}, tasks{6}, cfg, task_dir(runDir, 7), figCtx);
end

fusion.figures('write_index', figCtx, runDir);

finished = datetime('now', 'TimeZone', 'UTC');
manifest.status = 'complete';
manifest.finished_utc = char(finished);
manifest.elapsed_s = seconds(finished - started);
manifest.figures = struct( ...
    'written', sum(cellfun(@(r) strcmp(r.status, 'written'), figCtx.records)), ...
    'index_csv', fullfile(runDir, 'figures_index.csv'));
if numel(tasks) >= 7 && ~isempty(tasks{7})
    manifest.metrics = tasks{7}.metrics;
end
fusion.write_json(fullfile(runDir, 'manifest.json'), manifest);

result = struct('run_dir', runDir, 'manifest', manifest, 'tasks', {tasks}, ...
    'figure_context', figCtx);
fprintf('%s complete: %s\n', method, runDir);
fprintf('  figures written: %d -> %s\n', manifest.figures.written, manifest.figures.index_csv);
end

function path = task_dir(runDir, number)
c = fusion.catalog();
path = fullfile(runDir, c.tasks(number).directory);
if ~exist(path, 'dir'); mkdir(path); end
end
