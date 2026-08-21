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
%       PNG, publication PDF, plotted-data MAT and native MATLAB FIG while f
%       is still open, closes f, and appends a record to ctx.records.
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
[folder, stem] = fileparts(target);
figTarget = fullfile(folder, [stem '.fig']);
pdfTarget = fullfile(folder, [stem '.pdf']);
matTarget = fullfile(folder, [stem '.mat']);

stamp(f, task, spec, ctx);
cleanup = onCleanup(@() close_if_valid(f));
exportgraphics(f, target, 'Resolution', dpi);
pdfStatus = 'written';
try
    exportgraphics(f, pdfTarget, 'ContentType', 'vector');
catch exception
    pdfStatus = 'failed';
    warning('fusion:Figures:PdfSaveFailed', ...
        'Could not save PDF %s: %s', pdfTarget, exception.message);
end
plot_data = extract_figure_data(f); %#ok<NASGU>
try
    save(matTarget, 'plot_data', '-v7');
catch exception
    error('fusion:Figures:MatSaveFailed', ...
        'Could not save plotted-data MAT %s: %s', matTarget, exception.message);
end
try
    savefig(f, figTarget);
catch exception
    error('fusion:Figures:FigSaveFailed', ...
        'Could not save native editable FIG %s: %s', figTarget, exception.message);
end
if ~isfile(figTarget)
    error('fusion:Figures:FigNotWritten', ...
        'MATLAB did not create the native editable FIG: %s', figTarget);
end
clear cleanup
close_if_valid(f);

ctx.records{end+1} = make_record(task, spec, ctx, 'written', '', name, target, ...
    [stem '.fig'], figTarget, 'written', [stem '.pdf'], pdfTarget, pdfStatus, ...
    [stem '.mat'], matTarget, 'written');
end

function ctx = skip_figure(ctx, taskNumber, figureSlug, reason)
[task, spec] = lookup(taskNumber, figureSlug);
ctx.records{end+1} = make_record(task, spec, ctx, 'skipped', reason, '', '');
end

function record = make_record(task, spec, ctx, status, reason, name, target, ...
    figName, figTarget, figStatus, pdfName, pdfTarget, pdfStatus, ...
    matName, matTarget, matStatus)
if nargin < 8; figName = ''; end
if nargin < 9; figTarget = ''; end
if nargin < 10; figStatus = ''; end
if nargin < 11; pdfName = ''; end
if nargin < 12; pdfTarget = ''; end
if nargin < 13; pdfStatus = ''; end
if nargin < 14; matName = ''; end
if nargin < 15; matTarget = ''; end
if nargin < 16; matStatus = ''; end
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
    'status', status, 'reason', reason, 'filename', name, 'path', target, ...
    'fig_filename', figName, 'fig_path', figTarget, 'fig_status', figStatus, ...
    'pdf_filename', pdfName, 'pdf_path', pdfTarget, 'pdf_status', pdfStatus, ...
    'mat_filename', matName, 'mat_path', matTarget, 'mat_status', matStatus);
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

function plotData = extract_figure_data(f)
plotData = struct( ...
    'schema_version', 'sensor-fusion-matlab-figure-v1', ...
    'figure_name', string(f.Name), ...
    'figure_position', double(f.Position), ...
    'axes', {{}});
axesHandles = flipud(findall(f, 'Type', 'axes'));
for axesIndex = 1:numel(axesHandles)
    ax = axesHandles(axesIndex);
    axisData = struct( ...
        'position', double(ax.Position), ...
        'title', graphics_text(ax.Title), ...
        'xlabel', graphics_text(ax.XLabel), ...
        'ylabel', graphics_text(ax.YLabel), ...
        'xlim', double(ax.XLim), ...
        'ylim', double(ax.YLim), ...
        'xscale', string(ax.XScale), ...
        'yscale', string(ax.YScale), ...
        'lines', {{}}, 'scatters', {{}}, 'bars', {{}}, 'images', {{}});

    lines = flipud(findall(ax, 'Type', 'line'));
    for index = 1:numel(lines)
        item = lines(index);
        axisData.lines{end+1} = struct( ...
            'x', double(item.XData), 'y', double(item.YData), ...
            'display_name', string(item.DisplayName), ...
            'color', double(item.Color), 'line_style', string(item.LineStyle), ...
            'line_width', double(item.LineWidth), 'marker', string(item.Marker)); %#ok<AGROW>
    end

    scatters = flipud(findall(ax, 'Type', 'scatter'));
    for index = 1:numel(scatters)
        item = scatters(index);
        axisData.scatters{end+1} = struct( ...
            'x', double(item.XData), 'y', double(item.YData), ...
            'size_data', double(item.SizeData), ...
            'display_name', string(item.DisplayName)); %#ok<AGROW>
    end

    bars = flipud(findall(ax, 'Type', 'bar'));
    for index = 1:numel(bars)
        item = bars(index);
        axisData.bars{end+1} = struct( ...
            'x', double(item.XData), 'y', double(item.YData), ...
            'display_name', string(item.DisplayName)); %#ok<AGROW>
    end

    images = flipud(findall(ax, 'Type', 'image'));
    for index = 1:numel(images)
        item = images(index);
        axisData.images{end+1} = struct( ...
            'cdata', item.CData, 'xdata', double(item.XData), ...
            'ydata', double(item.YData)); %#ok<AGROW>
    end
    plotData.axes{end+1} = axisData;
end
end

function value = graphics_text(handle)
raw = handle.String;
if iscell(raw); value = strjoin(string(raw), newline);
else; value = string(raw); end
end

function close_if_valid(f)
if isgraphics(f); close(f); end
end

% ---------------------------------------------------------------------------
function write_index(ctx, runDir)
records = complete_records(ctx);
if ~exist(runDir, 'dir'); mkdir(runDir); end

columns = {'task', 'task_name', 'subtask', 'subtask_name', 'figure', ...
    'figure_title', 'coordinate_frame', 'sensor_data', 'method', 'status', ...
    'task_directory', 'filename', 'pdf_filename', 'pdf_status', ...
    'fig_filename', 'fig_status', 'mat_filename', 'mat_status'};
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
