function result = task2(imu, task1, cfg, outDir)
%TASK2 Select a static interval and form measured body vectors.

window=min(cfg.static_samples,numel(imu.time_s)); if window<20; error('fusion:Task2','Too few IMU samples.'); end
feature=vecnorm(imu.accel_mps2,2,2)+20*vecnorm(imu.gyro_rps,2,2);
sums=conv(feature,ones(window,1),'valid'); sums2=conv(feature.^2,ones(window,1),'valid');
variance=max(0,sums2/window-(sums/window).^2); [minimum,start]=min(variance); stop=start+window-1;
meanAccel=mean(imu.accel_mps2(start:stop,:),1); meanGyro=mean(imu.gyro_rps(start:stop,:),1);
body=[meanAccel/norm(meanAccel);meanGyro/norm(meanGyro)];
reference=[task1.specific_force_reference_ned_mps2/norm(task1.specific_force_reference_ned_mps2); ...
    task1.earth_rate_reference_ned_rps/norm(task1.earth_rate_reference_ned_rps)];
result=struct('task',2,'name','Static interval and measured body vectors', ...
    'subtasks',{{'2.1 Convert increments to SI rates','2.2 Select minimum-variance window','2.3 Average body vectors'}}, ...
    'static_start_index',start,'static_end_index_inclusive',stop, ...
    'static_duration_s',imu.time_s(stop)-imu.time_s(start)+imu.dt_s,'static_feature_variance',minimum, ...
    'mean_accel_body_mps2',meanAccel,'mean_gyro_body_rps',meanGyro, ...
    'body_vectors_unit',body,'reference_vectors_unit',reference);
fusion.write_json(fullfile(outDir,'body_vectors.json'),result); save(fullfile(outDir,'body_vectors.mat'),'-struct','result');
if cfg.plots
    stride=max(1,ceil(numel(imu.time_s)/cfg.max_plot_points)); idx=1:stride:numel(imu.time_s);
    f=figure('Visible','off'); tiledlayout(2,1); nexttile; plot(imu.time_s(idx),imu.accel_mps2(idx,:)); ylabel('m/s^2'); grid on;
    xline(imu.time_s(start),'--'); xline(imu.time_s(stop),'--'); legend('x','y','z','start','end');
    nexttile; plot(imu.time_s(idx),imu.gyro_rps(idx,:)); ylabel('rad/s'); xlabel('Time [s]'); grid on;
    sgtitle('Task 2 - Static IMU body vectors'); exportgraphics(f,fullfile(outDir,'static_interval.png'),'Resolution',160); close(f);
end
end
