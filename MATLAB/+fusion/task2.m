function [result, figCtx] = task2(imu, task1, cfg, outDir, figCtx)
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
    bodyAxes={'Body x','Body y','Body z'}; gyroColor=[0.12 0.47 0.71]; accelColor=[0.84 0.15 0.16];
    stride=max(1,ceil(numel(imu.time_s)/cfg.max_plot_points)); idx=1:stride:numel(imu.time_s);

    f=figure('Visible','off'); tiledlayout(2,3);
    for k=1:3
        ax=nexttile; plot(ax,imu.time_s(idx),imu.gyro_rps(idx,k),'LineWidth',0.8,'Color',gyroColor);
        grid(ax,'on'); title(ax,bodyAxes{k}); if k==1; ylabel(ax,'Angular rate [rad/s]'); end
        if k==1 && isfield(cfg,'imu_measurement_type')
            text(ax,0.03,0.92,sprintf('measurement type: %s',char(string(cfg.imu_measurement_type))), ...
                'Units','normalized','FontSize',8,'Interpreter','none');
        end
    end
    for k=1:3
        ax=nexttile; plot(ax,imu.time_s(idx),imu.accel_mps2(idx,k),'LineWidth',0.8,'Color',accelColor);
        grid(ax,'on'); xlabel(ax,'Time [s]'); if k==1; ylabel(ax,'Specific force [m/s^2]'); end
    end
    figCtx=fusion.figures('save',figCtx,2,'converted_rates',outDir,f);

    % variance(k) is the feature variance of the window STARTING at sample k,
    % so it is one window shorter than the log and shares its leading times.
    scanTime=imu.time_s(1:numel(variance)); scanValue=max(variance,realmin);
    scanStride=max(1,ceil(numel(scanTime)/cfg.max_plot_points)); scanIdx=1:scanStride:numel(scanTime);
    f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
    semilogy(ax,scanTime(scanIdx),scanValue(scanIdx),'LineWidth',0.9,'Color',gyroColor, ...
        'DisplayName','rolling feature variance');
    grid(ax,'on'); hold(ax,'on');
    marker=xline(imu.time_s(start),'--'); marker.Color=accelColor; marker.LineWidth=1.2;
    marker.DisplayName='selected window start';
    scatter(ax,imu.time_s(start),scanValue(start),40,accelColor,'filled','HandleVisibility','off');
    hold(ax,'off'); xlabel(ax,'Window start time [s]'); ylabel(ax,'Rolling feature variance (log)');
    legend(ax,'Location','best','FontSize',8);
    figCtx=fusion.figures('save',figCtx,2,'static_window_variance_scan',outDir,f);

    f=figure('Visible','off'); tiledlayout(2,3);
    for k=1:3
        ax=nexttile; plot(ax,imu.time_s(idx),imu.accel_mps2(idx,k),'LineWidth',0.8,'Color',accelColor);
        grid(ax,'on'); title(ax,bodyAxes{k}); if k==1; ylabel(ax,'Specific force [m/s^2]'); end
        xline(imu.time_s(start),'--'); xline(imu.time_s(stop),'--');
        if k==1
            text(ax,0.03,0.08,sprintf('static window %.3f to %.3f s (%d samples)', ...
                imu.time_s(start),imu.time_s(stop),window), ...
                'Units','normalized','FontSize',8,'Interpreter','none');
        end
    end
    for k=1:3
        ax=nexttile; plot(ax,imu.time_s(idx),imu.gyro_rps(idx,k),'LineWidth',0.8,'Color',gyroColor);
        grid(ax,'on'); xlabel(ax,'Time [s]'); if k==1; ylabel(ax,'Angular rate [rad/s]'); end
        xline(imu.time_s(start),'--'); xline(imu.time_s(stop),'--');
    end
    figCtx=fusion.figures('save',figCtx,2,'static_window_selection',outDir,f);

    pairNames={'Specific force','Earth rate'}; pairLabels={'x / North','y / East','z / Down'};
    f=figure('Visible','off'); tiledlayout(1,2);
    for k=1:2
        ax=nexttile; bar(ax,[result.body_vectors_unit(k,:).',result.reference_vectors_unit(k,:).']);
        grid(ax,'on'); set(ax,'XTick',1:3,'XTickLabel',pairLabels); ylim(ax,[-1.1 1.1]);
        title(ax,pairNames{k}); if k==1; ylabel(ax,'Unit component'); end
        legend(ax,{'measured (body)','reference (NED)'},'Location','best','FontSize',8);
    end
    figCtx=fusion.figures('save',figCtx,2,'mean_body_vs_reference_vectors',outDir,f);
end
end
