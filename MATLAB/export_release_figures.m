function count = export_release_figures(results_dir)
%EXPORT_RELEASE_FIGURES Create a native .fig beside every pipeline PNG.
%
% New Python MAT companions contain an editable figure schema. Those files are
% reconstructed as real MATLAB axes, lines, scatter points, rectangles and
% images before SAVEFIG, preserving zoom, pan and data tips. Older results that
% lack the schema retain a raster fallback and should be regenerated when
% editable plot objects are required.

arguments
    results_dir {mustBeTextScalar}
end
results_dir = string(results_dir);

if ~isfolder(results_dir)
    error('export_release_figures:MissingDirectory', ...
        'Results directory does not exist: %s', results_dir);
end

files = dir(fullfile(results_dir, '**', '*.png'));
count = 0;
editable_count = 0;
editable_expected = 0;
for k = 1:numel(files)
    png_path = fullfile(files(k).folder, files(k).name);
    [~, stem] = fileparts(png_path);
    fig_path = fullfile(files(k).folder, stem + ".fig");
    mat_path = fullfile(files(k).folder, stem + ".mat");

    fig_info = dir(fig_path);
    source_time = files(k).datenum;
    mat_info = dir(mat_path);
    if ~isempty(mat_info); source_time = max(source_time, mat_info.datenum); end
    editable_source = mat_source_is_editable(mat_path);
    if editable_source; editable_expected = editable_expected + 1; end
    current_editable = ~isempty(fig_info) && fig_is_data_editable(fig_path);
    if ~isempty(fig_info) && fig_info.datenum >= source_time && ...
            (~editable_source || current_editable)
        if current_editable; editable_count = editable_count + 1; end
        count = count + 1;
        continue
    end

    mode = export_python_figure(mat_path, png_path, fig_path);
    if mode == "editable-data"; editable_count = editable_count + 1; end
    count = count + 1;
    fprintf('[FIG:%s] %s\n', mode, fig_path);
end

if editable_count ~= editable_expected
    error('export_release_figures:EditableAuditFailed', ...
        ['Editable FIG audit failed: %d plot-data MAT source(s) were found, ' ...
        'but only %d data-editable FIG(s) were verified.'], ...
        editable_expected, editable_count);
end
fprintf('Native MATLAB FIG export complete: %d figure(s), %d data-editable.\n', ...
    count, editable_count);
end


function tf = mat_source_is_editable(matPath)
tf = false;
if ~isfile(matPath); return; end
variables = whos('-file', matPath);
names = string({variables.name});
tf = any(names == "figure_schema_version") || ...
    any(~cellfun('isempty', regexp(cellstr(names), '^ax\d+_line\d+_[xy]$', 'once')));
end


function tf = fig_is_data_editable(figPath)
tf = false;
fig = [];
try
    fig = openfig(figPath, 'invisible');
    cleanup = onCleanup(@() close_if_valid(fig));
    metadata = get(fig, 'UserData');
    tf = isstruct(metadata) && isfield(metadata, 'editable_plot_objects') && ...
        logical(metadata.editable_plot_objects);
    clear cleanup
    close_if_valid(fig);
catch
    close_if_valid(fig);
end
end


function close_if_valid(fig)
if ~isempty(fig) && isgraphics(fig); close(fig); end
end
