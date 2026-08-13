function count = export_release_figures(results_dir)
%EXPORT_RELEASE_FIGURES Create a native .fig beside every release PNG.
%
% Called automatically by PYTHON/run_release.py after each full run. Each FIG
% contains the exact rendered PNG in a native MATLAB figure, so it opens by
% double-clicking or uploading to MATLAB without running a redraw script. The
% same-stem MAT companion remains available for numeric analysis.

arguments
    results_dir (1,1) string
end

if ~isfolder(results_dir)
    error('export_release_figures:MissingDirectory', ...
        'Results directory does not exist: %s', results_dir);
end

files = dir(fullfile(results_dir, '**', '*.png'));
count = 0;
for k = 1:numel(files)
    png_path = fullfile(files(k).folder, files(k).name);
    [~, stem] = fileparts(png_path);
    fig_path = fullfile(files(k).folder, stem + ".fig");

    fig_info = dir(fig_path);
    if ~isempty(fig_info) && fig_info.datenum >= files(k).datenum
        count = count + 1;
        continue
    end

    image_data = imread(png_path);
    fig = figure('Visible', 'off', 'Name', stem, 'NumberTitle', 'off', ...
        'Color', 'white');
    cleanup = onCleanup(@() close_if_valid(fig));
    ax = axes(fig, 'Position', [0 0 1 1]);
    image(ax, image_data);
    axis(ax, 'image');
    axis(ax, 'off');
    set(fig, 'UserData', struct( ...
        'source_png', files(k).name, ...
        'mat_companion', stem + ".mat"));
    savefig(fig, fig_path);
    count = count + 1;
    fprintf('[FIG] %s\n', fig_path);
    clear cleanup
    close_if_valid(fig);
end

fprintf('Native MATLAB FIG export complete: %d figure(s).\n', count);
end


function close_if_valid(fig)
if isgraphics(fig)
    close(fig);
end
end
