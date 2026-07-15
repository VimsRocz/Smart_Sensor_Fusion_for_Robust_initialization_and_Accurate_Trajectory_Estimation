function result = task3(method, task1, task2, cfg, outDir)
%TASK3 Solve initial attitude and estimate body-frame IMU biases.

result=fusion.solve_attitude(method,task2.body_vectors_unit,task2.reference_vectors_unit, ...
    [cfg.gravity_weight,cfg.earth_rate_weight]);
C=result.c_body_to_ned;
expectedAccel=(C'*task1.specific_force_reference_ned_mps2')';
expectedGyro=(C'*task1.earth_rate_reference_ned_rps')';
result.task=3; result.name='Initial attitude and IMU biases';
result.subtasks={'3.1 Solve Body-to-NED alignment','3.2 Normalize [w,x,y,z] quaternion','3.3 Estimate IMU biases'};
result.accel_bias_body_mps2=task2.mean_accel_body_mps2-expectedAccel;
result.gyro_bias_body_rps=task2.mean_gyro_body_rps-expectedGyro;
fusion.write_json(fullfile(outDir,'initial_attitude.json'),result); save(fullfile(outDir,'initial_attitude.mat'),'-struct','result');
if cfg.plots
    f=figure('Visible','off'); tiledlayout(1,2); nexttile; bar(result.quaternion_wxyz_body_to_ned); ylim([-1.05,1.05]);
    xticklabels({'w','x','y','z'}); title('Body to NED quaternion'); grid on;
    nexttile; bar([result.gravity_error_deg,result.earth_rate_error_deg]); xticklabels({'Gravity','Earth rate'}); ylabel('deg'); grid on;
    sgtitle(['Task 3 - ' result.method]); exportgraphics(f,fullfile(outDir,'initial_attitude.png'),'Resolution',160); close(f);
end
end
