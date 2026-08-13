function fig = show_task_plot(matfile)
%SHOW_TASK_PLOT Load a pipeline .mat companion and draw its figure in MATLAB.
%
%   show_task_plot('TRIAD_IMU_X001_GNSS_X001_task6_4_attitude_quaternion_BodyToNED.mat')
%   show_task_plot                      % pick from the .mat files in this folder
%   fig = show_task_plot(...)           % also return the figure handle
%
% Each .mat written by the Python pipeline carries the plotted arrays plus the
% metadata needed to redraw them (plot_title, x_label, y_labels, series_names).
% Loading a .mat only puts variables in the workspace, so this function does the
% drawing. The figure is a normal MATLAB figure: editable, zoomable, saveable.

if nargin < 1 || isempty(matfile)
    files = dir(fullfile(pwd, '*_task*.mat'));
    if isempty(files)
        error('show_task_plot:NoFiles', ...
            'No *_task*.mat files in %s. cd to the results folder first.', pwd);
    end
    names = {files.name};
    [sel, ok] = listdlg('ListString', names, 'SelectionMode', 'single', ...
        'Name', 'Select a plot', 'ListSize', [640 400]);
    if ~ok; fig = []; return; end
    matfile = fullfile(files(sel).folder, names{sel});
end

if ~isfile(matfile)
    % Allow a bare name relative to the working directory.
    alt = fullfile(pwd, matfile);
    if isfile(alt); matfile = alt; else
        error('show_task_plot:NotFound', 'File not found: %s', matfile);
    end
end

S = load(matfile);
[~, stem] = fileparts(matfile);

% ---- metadata (all optional) -------------------------------------------
ttl     = getfield_default(S, 'plot_title', strrep(stem, '_', ' '));
xlab    = getfield_default(S, 'x_label', 'Time [s]');
ylabs   = tocellstr(getfield_default(S, 'y_labels', {}));
series  = tocellstr(getfield_default(S, 'series_names', {}));
colnames = tocellstr(getfield_default(S, 'col_names', {}));

% ---- the x vector -------------------------------------------------------
if isfield(S, 't'); x = double(S.t(:)); else; x = []; end

% ---- which variables are plottable -------------------------------------
skip = {'t', 'plot_title', 'x_label', 'y_labels', 'series_names', ...
        'col_names', 'frame', 'labels', 'rotation'};
names = fieldnames(S);
data = {}; dnames = {};
for k = 1:numel(names)
    n = names{k};
    if any(strcmp(n, skip)); continue; end
    v = S.(n);
    if ~isnumeric(v) || isempty(v); continue; end
    v = double(v);
    if isvector(v) && ~isempty(x) && numel(v) == numel(x)
        data{end+1} = v(:); dnames{end+1} = n; %#ok<AGROW>
    elseif ismatrix(v) && ~isempty(x) && size(v,1) == numel(x) && size(v,2) <= 4
        data{end+1} = v; dnames{end+1} = n; %#ok<AGROW>
    elseif isvector(v) && numel(v) <= 4 && isempty(x)
        data{end+1} = v(:).'; dnames{end+1} = n; %#ok<AGROW>
    end
end

if isempty(data)
    % Nothing time-series shaped: show the scalars so the file is still useful.
    fig = figure('Name', stem, 'NumberTitle', 'off');
    axis off; txt = {};
    for k = 1:numel(names)
        v = S.(names{k});
        if isnumeric(v) && numel(v) <= 12
            txt{end+1} = sprintf('%-22s %s', names{k}, mat2str(double(v), 6)); %#ok<AGROW>
        end
    end
    text(0.02, 0.98, txt, 'Units', 'normalized', 'VerticalAlignment', 'top', ...
        'FontName', 'Courier', 'Interpreter', 'none');
    title(ttl, 'Interpreter', 'none');
    return
end

% ---- draw ---------------------------------------------------------------
ncol = max(cellfun(@(v) size(v,2), data));
nrow = numel(data);
fig = figure('Name', stem, 'NumberTitle', 'off', ...
             'Position', [80 80 min(420*ncol, 1600) min(300*nrow, 1000)]);
tl = tiledlayout(fig, nrow, ncol, 'TileSpacing', 'compact', 'Padding', 'compact');

for r = 1:nrow
    v = data{r};
    for c = 1:size(v, 2)
        ax = nexttile(tl, (r-1)*ncol + c);
        if isempty(x)
            bar(ax, v(:, c));
        else
            plot(ax, x, v(:, c), 'LineWidth', 0.9);
        end
        grid(ax, 'on');
        if r == nrow && ~isempty(x); xlabel(ax, xlab); end
        if c == 1
            if numel(ylabs) >= r; ylabel(ax, ylabs{r});
            else; ylabel(ax, strrep(dnames{r}, '_', ' ')); end
        end
        if r == 1
            if numel(colnames) >= c; title(ax, colnames{c});
            elseif size(v,2) > 1;    title(ax, sprintf('component %d', c)); end
        end
    end
end

if ~isempty(series)
    lgd = legend(series, 'Orientation', 'horizontal');
    lgd.Layout.Tile = 'north';
end
title(tl, ttl, 'Interpreter', 'none');
end

% -------------------------------------------------------------------------
function v = getfield_default(S, name, fallback)
if isfield(S, name) && ~isempty(S.(name)); v = S.(name); else; v = fallback; end
end

function c = tocellstr(v)
if isempty(v);        c = {};
elseif iscell(v);     c = cellfun(@char, v, 'UniformOutput', false);
elseif ischar(v);     c = {v};
elseif isstring(v);   c = cellstr(v);
else;                 c = {};
end
end
