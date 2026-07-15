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
    end
end
