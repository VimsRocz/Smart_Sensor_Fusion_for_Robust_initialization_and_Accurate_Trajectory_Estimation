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
            end
            testCase.verifyEmpty(find(index.status == "not_implemented", 1));
        end
    end
end
