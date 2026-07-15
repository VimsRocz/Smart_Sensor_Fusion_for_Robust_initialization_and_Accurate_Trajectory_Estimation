function result = task7(task5, task6, cfg, outDir)
%TASK7 Export truth residuals or innovation-only metrics.

subtasks={'7.1 Position residuals','7.2 Velocity residuals','7.3 Quaternion geodesic error','7.4 Scalar metrics'};
if strcmp(task6.status,'complete')
    o=task6.overlay; pe=o.estimated_position-o.truth_position; ve=o.estimated_velocity-o.truth_velocity;
    pn=vecnorm(pe,2,2); vn=vecnorm(ve,2,2);
    metrics=struct('position_rmse_m',rms_value(pn),'position_final_m',pn(end),'position_max_m',max(pn), ...
        'velocity_rmse_mps',rms_value(vn),'velocity_final_mps',vn(end),'velocity_max_mps',max(vn), ...
        'height_rmse_m',rms_value(-pe(:,3)),'height_final_error_m',-pe(end,3), ...
        'height_final_abs_m',abs(pe(end,3)));
    if ~isempty(o.truth_quaternion)
        dots=min(1,abs(sum(o.estimated_quaternion.*o.truth_quaternion,2))); attitude=2*acosd(dots);
        metrics.attitude_rmse_deg=rms_value(attitude); metrics.attitude_final_deg=attitude(end); metrics.attitude_max_deg=max(attitude);
    end
    time_s=o.time_s; position_error_ned_m=pe; velocity_error_ned_mps=ve;
    artifact=fullfile(outDir,'residuals.mat'); save(artifact,'time_s','position_error_ned_m','velocity_error_ned_mps'); status='complete';
    if cfg.plots; residual_plot(fullfile(outDir,'residuals.png'),time_s,pe,ve,cfg); end
else
    innovations=task5.innovations; metrics=struct('gnss_updates',size(innovations,1));
    if ~isempty(innovations)
        metrics.innovation_position_rms_m=rms_value(vecnorm(innovations(:,1:3),2,2));
        metrics.innovation_velocity_rms_mps=rms_value(vecnorm(innovations(:,4:6),2,2));
    end
    artifact=''; status='complete_without_truth';
end
result=struct('task',7,'name','Residual evaluation and quality metrics','status',status, ...
    'subtasks',{subtasks},'metrics',metrics,'artifact',artifact);
fusion.write_json(fullfile(outDir,'metrics.json'),result);
end

function value=rms_value(x)
value=sqrt(mean(x.^2));
end

function residual_plot(path,time,pe,ve,cfg)
stride=max(1,ceil(numel(time)/cfg.max_plot_points)); idx=1:stride:numel(time); labels={'North','East','Down'};
f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3; nexttile(j); plot(time(idx),pe(idx,j)); yline(0); title(labels{j}); ylabel('Position error [m]'); grid on;
    nexttile(j+3); plot(time(idx),ve(idx,j)); yline(0); ylabel('Velocity error [m/s]'); xlabel('Time [s]'); grid on; end
sgtitle('Task 7 - Fused minus truth residuals'); exportgraphics(f,path,'Resolution',160); close(f);
end
