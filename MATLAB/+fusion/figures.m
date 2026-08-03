function varargout = figures(action, varargin)
%FIGURES Naming, stamping and indexing of pipeline figures.
%
%   Mirrors PYTHON/fusion_pipeline/figures.py so both implementations write
%   identically named files:
%
%     <run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>_<figure-slug>
%         _frame-<FRAME>_data-<DATASETS>.png
%
%   ctx = fusion.figures('context', runId, method, datasets)
%       datasets is a struct with char fields imu, gnss and truth ('' when a
%       file was not supplied).
%
%   name = fusion.figures('filename', ctx, taskNumber, figureSlug)
%   ctx  = fusion.figures('save', ctx, taskNumber, figureSlug, outDir, f, dpi)
%       Stamps the identifying title/footer onto figure handle f, writes the
%       PNG, closes f, and appends a record to ctx.records.
%   ctx  = fusion.figures('skip', ctx, taskNumber, figureSlug, reason)
%   fusion.figures('write_index', ctx, runDir)
%       Writes figures_index.csv and figures_index.json. Any catalog figure
%       that was neither saved nor skipped is recorded as 'not_implemented'
%       so the index always describes the complete catalog.

switch lower(char(action))
    case 'context';     varargout{1} = make_context(varargin{:});
    case 'filename';    varargout{1} = make_filename(varargin{:});
    case 'save';        varargout{1} = save_figure(varargin{:});
    case 'skip';        varargout{1} = skip_figure(varargin{:});
    case 'write_index'; write_index(varargin{:});
    otherwise
        error('fusion:Figures', 'Unknown figures action "%s".', action);
end
end

% ---------------------------------------------------------------------------
function ctx = make_context(runId, method, datasets)
ctx.run_id = safe_token(runId);
ctx.method = char(method);
ctx.datasets = struct( ...
    'imu',   token_or_empty(getfield_default(datasets, 'imu', '')), ...
    'gnss',  token_or_empty(getfield_default(datasets, 'gnss', '')), ...
    'truth', token_or_empty(getfield_default(datasets, 'truth', '')));
ctx.records = {};
end

function value = getfield_default(s, name, fallback)
if isstruct(s) && isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = fallback;
end
end

function out = token_or_empty(value)
if isempty(value)
    out = '';
else
    out = safe_token(value);
end
end

function out = safe_token(value)
out = regexprep(char(value), '[^A-Za-z0-9_.\-]', '_');
out = regexprep(out, '_{2,}', '_');
out = regexprep(out, '^_+|_+$', '');
if isempty(out); out = 'unnamed'; end
end

% ---------------------------------------------------------------------------
function [task, figure_spec] = lookup(taskNumber, figureSlug)
c = fusion.catalog();
if taskNumber == 0
    task = [];
    specs = c.comparison_figures;
else
    task = c.tasks(taskNumber);
    specs = task.figures;
end
figure_spec = [];
for k = 1:numel(specs)
    if strcmp(specs{k}.slug, figureSlug)
        figure_spec = specs{k};
        return
    end
end
error('fusion:Figures', 'Task %d has no figure "%s" in the catalog.', taskNumber, figureSlug);
end

function tag = data_tag(ctx, sources)
parts = {};
for k = 1:numel(sources)
    value = ctx.datasets.(sources{k});
    if ~isempty(value)
        parts{end+1} = value; %#ok<AGROW>
    end
end
if isempty(parts)
    tag = 'none';
else
    tag = strjoin(parts, '+');
end
end

function name = make_filename(ctx, taskNumber, figureSlug)
[task, spec] = lookup(taskNumber, figureSlug);
if isempty(task)
    name = sprintf('%s_ALLMETHODS_comparison_sub%s_%s_frame-%s_data-%s.png', ...
        ctx.run_id, spec.subtask, spec.slug, spec.frame, data_tag(ctx, spec.sources));
else
    name = sprintf('%s_%s_task%02d_sub%s_%s_%s_frame-%s_data-%s.png', ...
        ctx.run_id, safe_token(ctx.method), task.number, spec.subtask, ...
        task.slug, spec.slug, spec.frame, data_tag(ctx, spec.sources));
end
end

% ---------------------------------------------------------------------------
function ctx = save_figure(ctx, taskNumber, figureSlug, outDir, f, dpi)
if nargin < 6 || isempty(dpi); dpi = 160; end
[task, spec] = lookup(taskNumber, figureSlug);
name = make_filename(ctx, taskNumber, figureSlug);
if ~exist(outDir, 'dir'); mkdir(outDir); end
target = fullfile(outDir, name);

stamp(f, task, spec, ctx);
exportgraphics(f, target, 'Resolution', dpi);
close(f);

ctx.records{end+1} = make_record(task, spec, ctx, 'written', '', name, target);
end

function ctx = skip_figure(ctx, taskNumber, figureSlug, reason)
[task, spec] = lookup(taskNumber, figureSlug);
ctx.records{end+1} = make_record(task, spec, ctx, 'skipped', reason, '', '');
end

function record = make_record(task, spec, ctx, status, reason, name, target)
c = fusion.catalog();
meaning = '';
index = find(strcmp(c.frames(:, 1), spec.frame), 1);
if ~isempty(index); meaning = c.frames{index, 2}; end
if isempty(task)
    number = 0; taskName = 'Cross-method comparison';
    directory = c.comparison_slug; subtaskName = spec.title;
else
    number = task.number; taskName = task.name; directory = task.directory;
    subtaskName = '';
    for k = 1:numel(task.subtasks)
        if strcmp(task.subtasks{k}.number, spec.subtask)
            subtaskName = task.subtasks{k}.name;
        end
    end
end
record = struct( ...
    'task', number, 'task_name', taskName, 'task_directory', directory, ...
    'subtask', spec.subtask, 'subtask_name', subtaskName, ...
    'figure', spec.slug, 'figure_title', spec.title, ...
    'coordinate_frame', spec.frame, 'coordinate_frame_meaning', meaning, ...
    'sensor_data', data_tag(ctx, spec.sources), 'method', ctx.method, ...
    'status', status, 'reason', reason, 'filename', name, 'path', target);
end

function stamp(f, task, spec, ctx)
if isempty(task)
    heading = sprintf('Cross-method comparison %s — %s', spec.subtask, spec.title);
else
    heading = sprintf('Task %d · Subtask %s — %s\n%s', ...
        task.number, spec.subtask, task.name, spec.title);
end
sgtitle(f, heading, 'FontWeight', 'bold', 'FontSize', 12);

c = fusion.catalog();
index = find(strcmp(c.frames(:, 1), spec.frame), 1);
meaning = ''; if ~isempty(index); meaning = c.frames{index, 2}; end
footer = sprintf('frame: %s — %s    |    data: %s    |    run: %s    |    method: %s', ...
    spec.frame, meaning, strrep(data_tag(ctx, spec.sources), '+', ' + '), ...
    ctx.run_id, ctx.method);
annotation(f, 'textbox', [0 0 1 0.035], 'String', footer, ...
    'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
    'EdgeColor', 'none', 'FontSize', 7.5, 'Color', [0.27 0.27 0.27], ...
    'Interpreter', 'none', 'FitBoxToText', 'off');
end

% ---------------------------------------------------------------------------
function write_index(ctx, runDir)
records = complete_records(ctx);
if ~exist(runDir, 'dir'); mkdir(runDir); end

columns = {'task', 'task_name', 'subtask', 'subtask_name', 'figure', ...
    'figure_title', 'coordinate_frame', 'sensor_data', 'method', 'status', ...
    'task_directory', 'filename'};
rows = cell(numel(records), numel(columns));
for r = 1:numel(records)
    for k = 1:numel(columns)
        value = records{r}.(columns{k});
        if isnumeric(value); value = num2str(value); end
        rows{r, k} = string(value);
    end
end
writetable(cell2table(rows, 'VariableNames', columns), ...
    fullfile(runDir, 'figures_index.csv'));

written = sum(cellfun(@(r) strcmp(r.status, 'written'), records));
payload = struct( ...
    'run_id', ctx.run_id, 'method', ctx.method, ...
    'figure_name_pattern', ['<run-id>_<METHOD>_task<NN>_sub<T.S>_<task-slug>' ...
        '_<figure-slug>_frame-<FRAME>_data-<DATASETS>.png'], ...
    'datasets', ctx.datasets, 'figures_written', written, ...
    'figures_total', numel(records), 'figures', {records});
fusion.write_json(fullfile(runDir, 'figures_index.json'), payload);
end

function records = complete_records(ctx)
%COMPLETE_RECORDS Add a 'not_implemented' row for any unreported catalog figure.
records = ctx.records;
seen = cellfun(@(r) sprintf('%d/%s', r.task, r.figure), records, 'UniformOutput', false);
c = fusion.catalog();
for t = 1:numel(c.tasks)
    task = c.tasks(t);
    for k = 1:numel(task.figures)
        spec = task.figures{k};
        key = sprintf('%d/%s', task.number, spec.slug);
        if ~any(strcmp(seen, key))
            records{end+1} = make_record(task, spec, ctx, 'not_implemented', ...
                'This figure exists in the Python pipeline only', '', ''); %#ok<AGROW>
        end
    end
end
end
