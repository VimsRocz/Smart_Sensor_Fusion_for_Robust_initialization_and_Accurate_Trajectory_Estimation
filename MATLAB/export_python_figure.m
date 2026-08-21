function mode = export_python_figure(mat_path, png_path, fig_path)
%EXPORT_PYTHON_FIGURE Rebuild an interactive MATLAB FIG from Python plot data.
%
% The MAT companion contains axes positions, labels, limits and the numeric
% values behind lines, scatter points, rectangles and images. Reconstructing
% those objects before SAVEFIG keeps MATLAB zoom, pan, data tips and property
% editing available. Older outputs without plot data fall back to a raster FIG.

arguments
    mat_path {mustBeTextScalar}
    png_path {mustBeTextScalar}
    fig_path {mustBeTextScalar}
end
mat_path = string(mat_path);
png_path = string(png_path);
fig_path = string(fig_path);

if isfile(mat_path)
    data = load(mat_path);
else
    data = struct();
end

if isfield(data, 'figure_schema_version') || has_legacy_axes(data)
    fig = reconstruct_figure(data, mat_path, png_path);
    mode = "editable-data";
else
    fig = raster_figure(png_path, mat_path);
    mode = "raster-fallback";
end

cleanup = onCleanup(@() close_if_valid(fig));
savefig(fig, fig_path);
if ~isfile(fig_path)
    error('export_python_figure:FigNotWritten', ...
        'MATLAB did not create the requested FIG: %s', fig_path);
end
clear cleanup
close_if_valid(fig);
end


function tf = has_legacy_axes(data)
names = fieldnames(data);
tf = any(~cellfun('isempty', regexp(names, '^ax\d+_line\d+_[xy]$', 'once')));
end


function fig = reconstruct_figure(data, matPath, pngPath)
sizeInches = numeric_field(data, 'figure_size_inches', [12 7]);
if numel(sizeInches) < 2; sizeInches = [12 7]; end
pixelSize = max([640 420], round(100 * sizeInches(1:2)));
fig = figure('Visible', 'off', 'Color', 'white', ...
    'Position', [100 100 pixelSize(1) pixelSize(2)], ...
    'Name', char(text_field(data, 'figure_title', 'Python sensor-fusion figure')), ...
    'NumberTitle', 'off');

axesCount = scalar_field(data, 'axes_count', infer_axes_count(data));
for axesIndex = 1:axesCount
    prefix = "ax" + string(axesIndex);
    fallbackPosition = layout_position(axesIndex, axesCount);
    position = numeric_field(data, prefix + "_position", fallbackPosition);
    if numel(position) ~= 4; position = fallbackPosition; end
    position = max(0, min(1, position(:)'));
    if position(3) <= 0; position(3) = fallbackPosition(3); end
    if position(4) <= 0; position(4) = fallbackPosition(4); end

    ax = axes(fig, 'Position', position, 'Box', 'on');
    hold(ax, 'on');
    faceColor = numeric_field(data, prefix + "_facecolor", [1 1 1 1]);
    if numel(faceColor) >= 3; ax.Color = faceColor(1:3); end

    draw_images(ax, data, prefix);
    [legendHandles, legendLabels] = draw_rectangles(ax, data, prefix);
    [lineHandles, lineLabels] = draw_lines(ax, data, prefix);
    [scatterHandles, scatterLabels] = draw_scatters(ax, data, prefix);
    draw_text(ax, data, prefix);

    legendHandles = [legendHandles lineHandles scatterHandles]; %#ok<AGROW>
    legendLabels = [legendLabels lineLabels scatterLabels]; %#ok<AGROW>

    title(ax, text_field(data, prefix + "_title", ''), 'Interpreter', 'none');
    xlabel(ax, text_field(data, prefix + "_xlabel", ''), 'Interpreter', 'none');
    ylabel(ax, text_field(data, prefix + "_ylabel", ''), 'Interpreter', 'none');
    set_scale(ax, 'XScale', text_field(data, prefix + "_xscale", 'linear'));
    set_scale(ax, 'YScale', text_field(data, prefix + "_yscale", 'linear'));
    set_limits(ax, 'XLim', numeric_field(data, prefix + "_xlim", []));
    set_limits(ax, 'YLim', numeric_field(data, prefix + "_ylim", []));
    set_ticks(ax, 'XTick', 'XTickLabel', data, prefix + "_xticks", ...
        prefix + "_xticklabels");
    set_ticks(ax, 'YTick', 'YTickLabel', data, prefix + "_yticks", ...
        prefix + "_yticklabels");
    aspect = text_field(data, prefix + "_aspect", 'auto');
    if strcmpi(aspect, 'equal') || ...
            scalar_field(data, prefix + "_aspect_equal", 0) ~= 0
        axis(ax, 'equal');
    end
    if scalar_field(data, prefix + "_grid", 1) ~= 0; grid(ax, 'on'); end
    if scalar_field(data, prefix + "_axis_on", 1) == 0; axis(ax, 'off'); end
    if ~isempty(legendHandles)
        legend(ax, legendHandles, cellstr(legendLabels), ...
            'Interpreter', 'none', 'Location', 'best');
    end
end

figureTitle = text_field(data, 'figure_title', '');
if strlength(figureTitle) > 0
    annotation(fig, 'textbox', [0 0.945 1 0.05], 'String', figureTitle, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'middle', ...
        'EdgeColor', 'none', 'FontWeight', 'bold', 'FontSize', 12, ...
        'Interpreter', 'none');
end
footer = text_field(data, 'figure_footer', '');
if strlength(footer) > 0
    annotation(fig, 'textbox', [0 0 1 0.04], 'String', footer, ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'EdgeColor', 'none', 'FontSize', 7.5, 'Color', [0.27 0.27 0.27], ...
        'Interpreter', 'none');
end
set(fig, 'UserData', struct( ...
    'editable_plot_objects', true, ...
    'source_mat', char(matPath), ...
    'source_png', char(pngPath)));
end


function draw_images(ax, data, prefix)
count = object_count(data, prefix, 'image', 'cdata');
for index = 1:count
    item = prefix + "_image" + string(index);
    cdata = numeric_field(data, item + "_cdata", []);
    if isempty(cdata); continue; end
    extent = numeric_field(data, item + "_extent", [0 size(cdata,2) 0 size(cdata,1)]);
    if numel(extent) ~= 4; extent = [0 size(cdata,2) 0 size(cdata,1)]; end
    if ndims(cdata) == 2
        imagesc(ax, extent(1:2), extent(3:4), cdata);
        cmap = text_field(data, item + "_colormap", 'parula');
        try; colormap(ax, char(cmap)); catch; colormap(ax, parula); end
        limits = numeric_field(data, item + "_clim", []);
        if numel(limits) == 2 && all(isfinite(limits)) && limits(1) < limits(2)
            clim(ax, limits(:)');
        end
    else
        image(ax, 'XData', extent(1:2), 'YData', extent(3:4), 'CData', cdata);
    end
    if strcmpi(text_field(data, item + "_origin", 'upper'), 'upper')
        ax.YDir = 'reverse';
    end
end
end


function [handles, labels] = draw_lines(ax, data, prefix)
count = object_count(data, prefix, 'line', 'x');
handles = gobjects(0); labels = strings(0);
for index = 1:count
    item = prefix + "_line" + string(index);
    x = numeric_field(data, item + "_x", []);
    y = numeric_field(data, item + "_y", []);
    if isempty(x) || isempty(y) || numel(x) ~= numel(y); continue; end
    color = numeric_field(data, item + "_color", [0 0.4470 0.7410 1]);
    style = normalize_line_style(text_field(data, item + "_linestyle", '-'));
    marker = normalize_marker(text_field(data, item + "_marker", 'none'));
    lineWidth = scalar_field(data, item + "_linewidth", 0.8);
    markerSize = scalar_field(data, item + "_markersize", 6);
    handle = plot(ax, x(:), y(:), 'Color', color(1:min(3,numel(color))), ...
        'LineStyle', char(style), 'LineWidth', lineWidth, ...
        'Marker', char(marker), 'MarkerSize', markerSize);
    label = text_field(data, item + "_label", '');
    if strlength(label) > 0
        handle.DisplayName = char(label);
        handles(end+1) = handle; %#ok<AGROW>
        labels(end+1) = label; %#ok<AGROW>
    end
end
end


function [handles, labels] = draw_scatters(ax, data, prefix)
count = object_count(data, prefix, 'scatter', 'offsets');
handles = gobjects(0); labels = strings(0);
for index = 1:count
    item = prefix + "_scatter" + string(index);
    offsets = numeric_field(data, item + "_offsets", []);
    if isempty(offsets) || size(offsets,2) < 2; continue; end
    sizes = numeric_field(data, item + "_sizes", 20);
    if isempty(sizes); sizes = 20; end
    if numel(sizes) ~= 1 && numel(sizes) ~= size(offsets,1); sizes = sizes(1); end
    faceColor = numeric_field(data, item + "_facecolor", [0 0.4470 0.7410 1]);
    edgeColor = numeric_field(data, item + "_edgecolor", [0 0 0 0]);
    handle = scatter(ax, offsets(:,1), offsets(:,2), sizes(:), ...
        'MarkerFaceColor', faceColor(1:3));
    if numel(edgeColor) >= 4 && edgeColor(4) == 0
        handle.MarkerEdgeColor = 'none';
    else
        handle.MarkerEdgeColor = edgeColor(1:3);
    end
    label = text_field(data, item + "_label", '');
    if strlength(label) > 0
        handle.DisplayName = char(label);
        handles(end+1) = handle; %#ok<AGROW>
        labels(end+1) = label; %#ok<AGROW>
    end
end
end


function [handles, labels] = draw_rectangles(ax, data, prefix)
count = object_count(data, prefix, 'rectangle', 'position');
handles = gobjects(0); labels = strings(0);
for index = 1:count
    item = prefix + "_rectangle" + string(index);
    position = numeric_field(data, item + "_position", []);
    if numel(position) ~= 4; continue; end
    faceColor = numeric_field(data, item + "_facecolor", [0 0.4470 0.7410 1]);
    edgeColor = numeric_field(data, item + "_edgecolor", [0 0 0 1]);
    handle = rectangle(ax, 'Position', position(:)', ...
        'FaceColor', faceColor(1:3), 'EdgeColor', edgeColor(1:3));
    if numel(faceColor) >= 4 && isprop(handle, 'FaceAlpha')
        handle.FaceAlpha = faceColor(4);
    end
    if numel(edgeColor) >= 4 && edgeColor(4) == 0
        handle.EdgeColor = 'none';
    end
    label = text_field(data, item + "_label", '');
    if strlength(label) > 0
        handle.DisplayName = char(label);
        handles(end+1) = handle; %#ok<AGROW>
        labels(end+1) = label; %#ok<AGROW>
    end
end
end


function draw_text(ax, data, prefix)
count = object_count(data, prefix, 'text', 'string');
for index = 1:count
    item = prefix + "_text" + string(index);
    position = numeric_field(data, item + "_position", []);
    value = text_field(data, item + "_string", '');
    if numel(position) < 2 || strlength(value) == 0; continue; end
    handle = text(ax, position(1), position(2), value, 'Interpreter', 'none');
    if strcmpi(text_field(data, item + "_coordinates", 'data'), 'axes')
        handle.Units = 'normalized';
        handle.Position(1:2) = position(1:2);
    end
end
end


function fig = raster_figure(pngPath, matPath)
if ~isfile(pngPath)
    error('export_python_figure:MissingInputs', ...
        'Neither editable MAT data nor a PNG fallback is available.');
end
imageData = imread(pngPath);
fig = figure('Visible', 'off', 'Color', 'white', 'NumberTitle', 'off');
ax = axes(fig, 'Position', [0 0 1 1]);
image(ax, imageData); axis(ax, 'image'); axis(ax, 'off');
set(fig, 'UserData', struct( ...
    'editable_plot_objects', false, ...
    'source_mat', char(matPath), ...
    'source_png', char(pngPath), ...
    'note', 'Raster fallback: rerun Python after the editable FIG update.'));
end


function count = infer_axes_count(data)
names = fieldnames(data); values = [];
for index = 1:numel(names)
    token = regexp(names{index}, '^ax(\d+)_', 'tokens', 'once');
    if ~isempty(token); values(end+1) = str2double(token{1}); end %#ok<AGROW>
end
if isempty(values); count = 0; else; count = max(values); end
end


function count = object_count(data, prefix, kind, terminal)
countName = prefix + "_" + kind + "_count";
if isfield(data, char(countName))
    count = scalar_field(data, countName, 0);
    return
end
pattern = char("^" + prefix + "_" + kind + "(\d+)_" + terminal + "$");
names = fieldnames(data); values = [];
for index = 1:numel(names)
    token = regexp(names{index}, pattern, 'tokens', 'once');
    if ~isempty(token); values(end+1) = str2double(token{1}); end %#ok<AGROW>
end
if isempty(values); count = 0; else; count = max(values); end
end


function value = numeric_field(data, name, fallback)
name = char(name);
if ~isfield(data, name); value = fallback; return; end
raw = data.(name);
while iscell(raw) && isscalar(raw); raw = raw{1}; end
if isnumeric(raw) || islogical(raw)
    value = double(raw);
else
    value = fallback;
end
end


function value = scalar_field(data, name, fallback)
raw = numeric_field(data, name, fallback);
if isempty(raw); value = fallback; else; value = double(raw(1)); end
end


function value = text_field(data, name, fallback)
name = char(name);
if ~isfield(data, name); value = string(fallback); return; end
raw = data.(name);
while iscell(raw) && isscalar(raw); raw = raw{1}; end
if ischar(raw) || isstring(raw)
    value = string(raw);
else
    value = string(fallback);
end
if numel(value) > 1; value = strjoin(value(:)', newline); end
end


function position = layout_position(index, count)
if count == 9; rows = 3; columns = 3;
elseif count == 6; rows = 2; columns = 3;
elseif count == 4; rows = 2; columns = 2;
elseif count <= 3; rows = 1; columns = max(1,count);
else; columns = ceil(sqrt(count)); rows = ceil(count/columns); end
row = floor((index-1)/columns); column = mod(index-1,columns);
marginX = 0.07; marginBottom = 0.08; marginTop = 0.10; gap = 0.045;
width = (1-2*marginX-(columns-1)*gap)/columns;
height = (1-marginBottom-marginTop-(rows-1)*gap)/rows;
position = [marginX+column*(width+gap), ...
    1-marginTop-(row+1)*height-row*gap, width, height];
end


function set_limits(ax, propertyName, values)
if numel(values) == 2 && all(isfinite(values)) && values(1) < values(2)
    set(ax, propertyName, values(:)');
end
end


function set_scale(ax, propertyName, value)
if any(strcmpi(value, ["linear" "log"]))
    set(ax, propertyName, char(value));
end
end


function set_ticks(ax, tickProperty, labelProperty, data, tickName, labelName)
ticks = numeric_field(data, tickName, []);
if isempty(ticks); return; end
set(ax, tickProperty, ticks(:)');
name = char(labelName);
if ~isfield(data, name); return; end
labels = string(data.(name));
labels = labels(:)';
if numel(labels) == numel(ticks); set(ax, labelProperty, cellstr(labels)); end
end


function value = normalize_line_style(value)
if any(strcmpi(value, ["none" "" " "])); value = "none"; end
if ~any(strcmp(value, ["-" "--" ":" "-." "none"])); value = "-"; end
end


function value = normalize_marker(value)
if any(strcmpi(value, ["none" "" " "])); value = "none"; end
valid = ["none" "o" "+" "*" "." "x" "square" "diamond" "^" "v" ">" "<" "pentagram" "hexagram" "s" "d" "p" "h"];
if ~any(strcmp(value, valid)); value = "none"; end
if value == "s"; value = "square"; elseif value == "d"; value = "diamond";
elseif value == "p"; value = "pentagram"; elseif value == "h"; value = "hexagram"; end
end


function close_if_valid(fig)
if isgraphics(fig); close(fig); end
end
