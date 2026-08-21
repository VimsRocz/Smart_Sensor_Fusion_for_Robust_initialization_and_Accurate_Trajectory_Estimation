classdef TestPipeline < matlab.unittest.TestCase
    % Canonical MATLAB Task 1-7 regression tests.

    properties
        RepoRoot
        OutputRoot
    end

    methods (TestMethodSetup)
        function setup(testCase)
            testFile = mfilename('fullpath');
            testCase.RepoRoot = fileparts(fileparts(fileparts(testFile)));
            addpath(fullfile(testCase.RepoRoot, 'MATLAB'));
            testCase.OutputRoot = tempname;
            mkdir(testCase.OutputRoot);
        end
    end

    methods (TestMethodTeardown)
        function cleanup(testCase)
            if isfolder(testCase.OutputRoot)
                rmdir(testCase.OutputRoot, 's');
            end
        end
    end

    methods (Test)
        function allAttitudeMethodsRecoverKnownRotation(testCase)
            angle = deg2rad(32);
            expected = [cos(angle),-sin(angle),0;sin(angle),cos(angle),0;0,0,1];
            reference = [0,0,-1;.7,0,-.3];
            reference = reference ./ vecnorm(reference,2,2);
            body = (expected' * reference')';
            methods = {'TRIAD','Davenport','SVD'};
            for i = 1:numel(methods)
                result = fusion.solve_attitude(methods{i}, body, reference, [.9999,.0001]);
                testCase.verifyEqual(result.c_body_to_ned, expected, 'AbsTol', 1e-9);
                testCase.verifyEqual(result.quaternion_norm, 1, 'AbsTol', 1e-12);
            end
        end

        function bundledSmallPipelineCompletes(testCase)
            cfg = struct('static_samples',100,'plots',false);
            result = run_pipeline( ...
                'imu',fullfile(testCase.RepoRoot,'DATA','IMU','IMU_X001_small.dat'), ...
                'gnss',fullfile(testCase.RepoRoot,'DATA','GNSS','GNSS_X001_small.csv'), ...
                'truth',fullfile(testCase.RepoRoot,'DATA','Truth','STATE_X001_small.txt'), ...
                'method','TRIAD','tasks','1-7','output',testCase.OutputRoot,'config',cfg);
            testCase.verifyEqual(result.manifest.status,'complete');
            testCase.verifyLessThan(result.tasks{7}.metrics.attitude_rmse_deg,.1);
            testCase.verifyLessThan(result.tasks{7}.metrics.height_rmse_m,.2);
            testCase.verifyEqual(result.tasks{4}.range_screening.accelerometer_samples_interpolated,80);
        end

        function catalogDefinesSevenTasksWithFigurePerSubtask(testCase)
            c = fusion.catalog();
            testCase.verifyEqual(numel(c.tasks), 7);
            required = {4,'4.6';5,'5.10';7,'7.6'};
            for k = 1:7
                task = c.tasks(k);
                testCase.verifyEqual(task.number, k);
                testCase.verifyEqual(task.directory, sprintf('task_%02d_%s', k, task.slug));
                testCase.verifyNotEmpty(task.subtasks);
                covered = cellfun(@(f) string(f.subtask), task.figures);
                for j = 1:numel(task.subtasks)
                    testCase.verifyTrue(any(covered == string(task.subtasks{j}.number)), ...
                        sprintf('Task %d subtask %s has no figure', k, task.subtasks{j}.number));
                end
                match = find(cell2mat(required(:,1)) == k, 1);
                if ~isempty(match)
                    declared = cellfun(@(s) string(s.number), task.subtasks);
                    testCase.verifyTrue(any(declared == string(required{match,2})));
                end
            end
        end

        function figureFilenamesCarryTaskSubtaskFrameAndDataset(testCase)
            ctx = fusion.figures('context', 'X001_small', 'TRIAD', ...
                struct('imu','IMU_X001_small','gnss','GNSS_X001_small','truth','STATE_X001_small'));
            c = fusion.catalog();
            for k = 1:numel(c.tasks)
                task = c.tasks(k);
                for j = 1:numel(task.figures)
                    spec = task.figures{j};
                    name = fusion.figures('filename', ctx, task.number, spec.slug);
                    testCase.verifySubstring(name, sprintf('_task%02d_', task.number));
                    testCase.verifySubstring(name, sprintf('_sub%s_', spec.subtask));
                    testCase.verifySubstring(name, task.slug);
                    testCase.verifySubstring(name, spec.slug);
                    testCase.verifySubstring(name, sprintf('_frame-%s_', spec.frame));
                    testCase.verifyTrue(endsWith(name, '.png'));
                end
            end
        end

        function truthFreeRunDropsTruthFromTheDatasetTag(testCase)
            ctx = fusion.figures('context', 'run', 'SVD', ...
                struct('imu','IMU_X002','gnss','GNSS_X002','truth',''));
            name = fusion.figures('filename', ctx, 1, 'input_time_coverage');
            testCase.verifySubstring(name, 'data-IMU_X002+GNSS_X002');
        end

        function pipelineWithFiguresWritesEveryIndexedFile(testCase)
            cfg = struct('static_samples',100,'plots',true);
            result = run_pipeline( ...
                'imu',fullfile(testCase.RepoRoot,'DATA','IMU','IMU_X001_small.dat'), ...
                'gnss',fullfile(testCase.RepoRoot,'DATA','GNSS','GNSS_X001_small.csv'), ...
                'truth',fullfile(testCase.RepoRoot,'DATA','Truth','STATE_X001_small.txt'), ...
                'method','TRIAD','tasks','1-7','output',testCase.OutputRoot,'config',cfg);

            indexPath = fullfile(result.run_dir,'figures_index.csv');
            testCase.verifyTrue(isfile(indexPath));
            index = readtable(indexPath,'TextType','string');

            % The index must describe the whole catalog, not just what was drawn.
            c = fusion.catalog();
            expected = 0;
            for k = 1:numel(c.tasks); expected = expected + numel(c.tasks(k).figures); end
            testCase.verifyEqual(height(index), expected);

            written = index(index.status == "written", :);
            testCase.verifyGreaterThan(height(written), 0);
            for r = 1:height(written)
                testCase.verifyTrue(isfile(fullfile(result.run_dir, ...
                    char(written.task_directory(r)), char(written.filename(r)))), ...
                    sprintf('Missing figure file: %s', written.filename(r)));
                testCase.verifyTrue(isfile(fullfile(result.run_dir, ...
                    char(written.task_directory(r)), char(written.fig_filename(r)))), ...
                    sprintf('Missing native FIG file: %s', written.fig_filename(r)));
                testCase.verifyTrue(isfile(fullfile(result.run_dir, ...
                    char(written.task_directory(r)), char(written.pdf_filename(r)))), ...
                    sprintf('Missing PDF file: %s', written.pdf_filename(r)));
                testCase.verifyTrue(isfile(fullfile(result.run_dir, ...
                    char(written.task_directory(r)), char(written.mat_filename(r)))), ...
                    sprintf('Missing plotted-data MAT file: %s', written.mat_filename(r)));
                testCase.verifyEqual(string(written.fig_status(r)), "written");
                testCase.verifyEqual(string(written.pdf_status(r)), "written");
                testCase.verifyEqual(string(written.mat_status(r)), "written");
            end
            testCase.verifyEmpty(find(index.status == "not_implemented", 1));

            editableRow = written(written.figure == "propagated_quaternion", :);
            editablePath = fullfile(result.run_dir, ...
                char(editableRow.task_directory(1)), char(editableRow.fig_filename(1)));
            editableFigure = openfig(editablePath, 'invisible');
            editableCleanup = onCleanup(@() close(editableFigure));
            testCase.verifyNotEmpty(findall(editableFigure, 'Type', 'line'));
            zoomObject = zoom(editableFigure); zoomObject.Enable = 'on';
            testCase.verifyEqual(string(zoomObject.Enable), "on");
            clear editableCleanup
        end

        function releaseExporterWritesDirectlyOpenableNativeFig(testCase)
            pngPath = fullfile(testCase.OutputRoot, 'release_plot.png');
            fig = figure('Visible','off');
            plot(0:0.1:1, sin(0:0.1:1));
            title('Release FIG export test');
            exportgraphics(fig, pngPath);
            close(fig);

            matPath = fullfile(testCase.OutputRoot, 'release_plot.mat');
            figure_schema_version = 'sensor-fusion-figure-v1'; %#ok<NASGU>
            figure_title = 'Editable release FIG export test'; %#ok<NASGU>
            figure_footer = 'data-backed MATLAB reconstruction'; %#ok<NASGU>
            figure_size_inches = [8 5]; axes_count = 1; %#ok<NASGU>
            ax1_position = [.12 .14 .82 .72]; %#ok<NASGU>
            ax1_title = 'Sine wave'; ax1_xlabel = 'Time [s]'; ax1_ylabel = 'Value'; %#ok<NASGU>
            ax1_xlim = [0 1]; ax1_ylim = [-1 1]; %#ok<NASGU>
            ax1_xscale = 'linear'; ax1_yscale = 'linear'; ax1_grid = 1; %#ok<NASGU>
            ax1_line_count = 1; ax1_line1_x = 0:.1:1; ax1_line1_y = sin(ax1_line1_x); %#ok<NASGU>
            ax1_line1_label = 'sin(t)'; ax1_line1_color = [0 .447 .741 1]; %#ok<NASGU>
            ax1_line1_linestyle = '-'; ax1_line1_linewidth = 1; %#ok<NASGU>
            ax1_line1_marker = 'none'; ax1_line1_markersize = 6; %#ok<NASGU>
            ax1_scatter_count = 0; ax1_rectangle_count = 0; %#ok<NASGU>
            ax1_image_count = 0; ax1_text_count = 0; %#ok<NASGU>
            save(matPath, 'figure_schema_version', 'figure_title', 'figure_footer', ...
                'figure_size_inches', 'axes_count', 'ax1_position', 'ax1_title', ...
                'ax1_xlabel', 'ax1_ylabel', 'ax1_xlim', 'ax1_ylim', 'ax1_xscale', ...
                'ax1_yscale', 'ax1_grid', 'ax1_line_count', 'ax1_line1_x', ...
                'ax1_line1_y', 'ax1_line1_label', 'ax1_line1_color', ...
                'ax1_line1_linestyle', 'ax1_line1_linewidth', 'ax1_line1_marker', ...
                'ax1_line1_markersize', 'ax1_scatter_count', 'ax1_rectangle_count', ...
                'ax1_image_count', 'ax1_text_count');

            count = export_release_figures(testCase.OutputRoot);
            figPath = fullfile(testCase.OutputRoot, 'release_plot.fig');
            testCase.verifyEqual(count, 1);
            testCase.verifyTrue(isfile(figPath));

            reopened = openfig(figPath, 'invisible');
            closeCleanup = onCleanup(@() close(reopened));
            testCase.verifyTrue(isgraphics(reopened, 'figure'));
            lines = findall(reopened, 'Type', 'line');
            testCase.verifyNotEmpty(lines);
            testCase.verifyEqual(lines(1).YData, sin(0:.1:1), 'AbsTol', 1e-12);
            testCase.verifyTrue(reopened.UserData.editable_plot_objects);
            clear closeCleanup
        end
    end
end
