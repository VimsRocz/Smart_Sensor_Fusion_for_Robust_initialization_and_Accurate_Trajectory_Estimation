function [result, figCtx] = task7(task1, task5, task6, cfg, outDir, figCtx)
%TASK7 Export truth residuals or innovation-only metrics.

subtasks={'7.1 Position residuals','7.2 Velocity residuals','7.3 Quaternion geodesic error', ...
    '7.4 Scalar metrics','7.6 Truth overlays and detailed attitude diagnostics'};
truthFigures={'position_residuals','position_error_distribution','velocity_residuals','attitude_error', ...
    'fused_vs_truth_ned','fused_vs_truth_ecef','fused_vs_truth_body', ...
    'quaternion_truth_vs_estimate','quaternion_error_components', ...
    'euler_error_over_time','attitude_error_angle'};
attitude=[];
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

if ~cfg.plots; return; end

if strcmp(status,'complete')
    stride=max(1,ceil(numel(time_s)/cfg.max_plot_points)); idx=1:stride:numel(time_s);
    figCtx=fusion.figures('save',figCtx,7,'position_residuals',outDir, ...
        residual_plot(time_s,pe,idx,'Position error [m]'));
    figCtx=fusion.figures('save',figCtx,7,'position_error_distribution',outDir, ...
        distribution_plot(pn));
    figCtx=fusion.figures('save',figCtx,7,'velocity_residuals',outDir, ...
        residual_plot(time_s,ve,idx,'Velocity error [m/s]'));
    if isempty(attitude)
        figCtx=fusion.figures('skip',figCtx,7,'attitude_error','truth file has no quaternion columns');
    else
        figCtx=fusion.figures('save',figCtx,7,'attitude_error',outDir, ...
            attitude_plot(time_s,attitude,idx,metrics.attitude_rmse_deg));
    end

    figCtx=fusion.figures('save',figCtx,7,'fused_vs_truth_ned',outDir, ...
        state_overlay_plot(time_s,o.estimated_position,o.truth_position, ...
        o.estimated_velocity,o.truth_velocity,idx,{'North','East','Down'}));

    dcm_ned_to_ecef=task1.c_ecef_to_ned'; origin_ecef=task1.origin_ecef_m;
    estimated_position_ecef=o.estimated_position*dcm_ned_to_ecef'+origin_ecef;
    truth_position_ecef=o.truth_position*dcm_ned_to_ecef'+origin_ecef;
    estimated_velocity_ecef=o.estimated_velocity*dcm_ned_to_ecef';
    truth_velocity_ecef=o.truth_velocity*dcm_ned_to_ecef';
    figCtx=fusion.figures('save',figCtx,7,'fused_vs_truth_ecef',outDir, ...
        state_overlay_plot(time_s,estimated_position_ecef,truth_position_ecef, ...
        estimated_velocity_ecef,truth_velocity_ecef,idx,{'X','Y','Z'}));

    if isempty(o.truth_quaternion); truth_body_quaternion=o.estimated_quaternion;
    else; truth_body_quaternion=o.truth_quaternion; end
    estimated_position_body=rotate_ned_to_body(o.estimated_position,o.estimated_quaternion);
    truth_position_body=rotate_ned_to_body(o.truth_position,truth_body_quaternion);
    estimated_velocity_body=rotate_ned_to_body(o.estimated_velocity,o.estimated_quaternion);
    truth_velocity_body=rotate_ned_to_body(o.truth_velocity,truth_body_quaternion);
    figCtx=fusion.figures('save',figCtx,7,'fused_vs_truth_body',outDir, ...
        state_overlay_plot(time_s,estimated_position_body,truth_position_body, ...
        estimated_velocity_body,truth_velocity_body,idx,{'Body x','Body y','Body z'}));

    if isempty(o.truth_quaternion)
        attitudeFigures={'quaternion_truth_vs_estimate','quaternion_error_components', ...
            'euler_error_over_time','attitude_error_angle'};
        for k=1:numel(attitudeFigures)
            figCtx=fusion.figures('skip',figCtx,7,attitudeFigures{k}, ...
                'truth file has no quaternion columns');
        end
    else
        figCtx=fusion.figures('save',figCtx,7,'quaternion_truth_vs_estimate',outDir, ...
            quaternion_overlay_plot(time_s,o.estimated_quaternion,o.truth_quaternion,idx));
        figCtx=fusion.figures('save',figCtx,7,'quaternion_error_components',outDir, ...
            quaternion_error_plot(time_s,o.estimated_quaternion-o.truth_quaternion,idx));
        figCtx=fusion.figures('save',figCtx,7,'euler_error_over_time',outDir, ...
            euler_error_plot(time_s,o.estimated_quaternion,o.truth_quaternion,idx));
        figCtx=fusion.figures('save',figCtx,7,'attitude_error_angle',outDir, ...
            attitude_plot(time_s,attitude,idx,metrics.attitude_rmse_deg));
    end
else
    for k=1:numel(truthFigures)
        figCtx=fusion.figures('skip',figCtx,7,truthFigures{k},'no truth overlay available');
    end
end

figCtx=fusion.figures('save',figCtx,7,'metric_summary',outDir,metric_plot(metrics));

if isempty(task5.innovations)
    figCtx=fusion.figures('skip',figCtx,7,'innovation_summary','no GNSS updates were applied');
else
    figCtx=fusion.figures('save',figCtx,7,'innovation_summary',outDir, ...
        innovation_plot(task5.innovations));
end
end

function value=rms_value(x)
value=sqrt(mean(x.^2));
end

function f=new_figure(width,height)
% Sized explicitly: the stamped two-line sgtitle and the footer annotation need
% more room than the default figure gives.
f=figure('Visible','off','Position',[100 100 width height]);
end

function f=residual_plot(time,errors,idx,label)
labels={'North','East','Down'};
f=new_figure(1300,470); tl=tiledlayout(f,1,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,time(idx),errors(idx,j),'LineWidth',0.8);
    yline(ax,0,'k-','LineWidth',0.6);
    title(ax,labels{j}); xlabel(ax,'Time [s]'); grid(ax,'on');
    if j==1; ylabel(ax,label); end
end
end

function f=distribution_plot(norms)
f=new_figure(1200,470); tl=tiledlayout(f,1,2);
ax=nexttile(tl,1); histogram(ax,norms,60);
xlabel(ax,'Position error norm [m]'); ylabel(ax,'Samples'); title(ax,'Histogram'); grid(ax,'on');

% Percentiles by indexing the sorted vector: prctile lives in the Statistics
% Toolbox and this pipeline stays on base MATLAB.
ordered=sort(norms(:)); n=numel(ordered);
ax=nexttile(tl,2); plot(ax,ordered,linspace(0,100,n),'LineWidth',1.1); hold(ax,'on');
for p=[50 95]
    value=ordered(max(1,min(n,ceil(p/100*n))));
    xline(ax,value,'--','LineWidth',0.8,'Label',sprintf('p%d = %.3f m',p,value));
end
xlabel(ax,'Position error norm [m]'); ylabel(ax,'Percentile [%]');
title(ax,'Cumulative distribution'); grid(ax,'on');
end

function f=attitude_plot(time,attitude,idx,rmse)
purple=[0.5804 0.4039 0.7412];
f=new_figure(1200,470); tl=tiledlayout(f,1,1);
ax=nexttile(tl); plot(ax,time(idx),attitude(idx),'LineWidth',0.8,'Color',purple); hold(ax,'on');
yline(ax,rmse,'k--','LineWidth',0.8,'Label',sprintf('RMSE = %.4f deg',rmse));
xlabel(ax,'Time [s]'); ylabel(ax,'Attitude error [deg]'); grid(ax,'on');
end

function f=metric_plot(metrics)
names=fieldnames(metrics); keep={}; values=[];
for k=1:numel(names)
    value=metrics.(names{k});
    if isnumeric(value) && isscalar(value) && isfinite(value)
        keep{end+1}=names{k}; %#ok<AGROW>
        values(end+1)=double(value); %#ok<AGROW>
    end
end
f=new_figure(1200,max(420,42*numel(keep)+180)); tl=tiledlayout(f,1,1); ax=nexttile(tl);
if isempty(keep)
    axis(ax,'off');
    text(ax,0.5,0.5,'no scalar metrics available','HorizontalAlignment','center');
    return
end
barh(ax,1:numel(keep),values); grid(ax,'on');
set(ax,'YTick',1:numel(keep),'YTickLabel',strrep(keep,'_','\_'),'YDir','reverse');
xlabel(ax,'Metric value');
for k=1:numel(keep)
    text(ax,values(k),k,sprintf('  %.6g',values(k)),'VerticalAlignment','middle','FontSize',8);
end
end

function f=innovation_plot(innovations)
positionNorm=vecnorm(innovations(:,1:3),2,2); velocityNorm=vecnorm(innovations(:,4:6),2,2);
bins=min(40,max(5,floor(size(innovations,1)/2)));
f=new_figure(1200,470); tl=tiledlayout(f,1,2);
ax=nexttile(tl,1); histogram(ax,positionNorm,bins);
xlabel(ax,'|position innovation| [m]'); ylabel(ax,'Updates');
title(ax,'GNSS position innovations'); grid(ax,'on');
ax=nexttile(tl,2); histogram(ax,velocityNorm,bins);
xlabel(ax,'|velocity innovation| [m/s]'); ylabel(ax,'Updates');
title(ax,'GNSS velocity innovations'); grid(ax,'on');
end

function body=rotate_ned_to_body(values,quaternion)
body=zeros(size(values));
for i=1:size(values,1)
    dcm_body_to_ned=fusion.math3d('quaternion_to_matrix',quaternion(i,:));
    body(i,:)=(dcm_body_to_ned'*values(i,:)')';
end
end

function f=state_overlay_plot(time,estimatedPosition,truthPosition, ...
    estimatedVelocity,truthVelocity,idx,labels)
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,time(idx),estimatedPosition(idx,j),'LineWidth',0.9); hold(ax,'on');
    plot(ax,time(idx),truthPosition(idx,j),'--','LineWidth',0.9); hold(ax,'off');
    title(ax,labels{j}); grid(ax,'on');
    if j==1; ylabel(ax,'Position [m]'); legend(ax,{'Fused','Truth'},'FontSize',8); end
    ax=nexttile(tl,j+3); plot(ax,time(idx),estimatedVelocity(idx,j),'LineWidth',0.9); hold(ax,'on');
    plot(ax,time(idx),truthVelocity(idx,j),'--','LineWidth',0.9); hold(ax,'off');
    xlabel(ax,'Time [s]'); grid(ax,'on');
    if j==1; ylabel(ax,'Velocity [m/s]'); end
end
end

function f=quaternion_overlay_plot(time,estimated,truth,idx)
labels={'w','x','y','z'}; f=new_figure(1200,700); tl=tiledlayout(f,2,2);
for j=1:4
    ax=nexttile(tl,j); plot(ax,time(idx),truth(idx,j),'LineWidth',0.9); hold(ax,'on');
    plot(ax,time(idx),estimated(idx,j),'--','LineWidth',0.9); hold(ax,'off');
    ylabel(ax,['q' labels{j}]); grid(ax,'on'); ax.YAxis.Exponent=0; ytickformat(ax,'%.6f');
    if j>=3; xlabel(ax,'Time [s]'); end
    if j==1; legend(ax,{'Truth','Estimate'},'FontSize',8); end
end
end

function f=quaternion_error_plot(time,error,idx)
labels={'w','x','y','z'}; f=new_figure(1200,700); tl=tiledlayout(f,2,2);
for j=1:4
    ax=nexttile(tl,j); plot(ax,time(idx),error(idx,j),'LineWidth',0.8,'Color',[0.84 0.15 0.16]);
    yline(ax,0,'k-','LineWidth',0.6); ylabel(ax,['Delta q' labels{j}]); grid(ax,'on');
    if j>=3; xlabel(ax,'Time [s]'); end
end
end

function f=euler_error_plot(time,estimatedQuaternion,truthQuaternion,idx)
estimated=euler_zyx_deg(estimatedQuaternion); truth=euler_zyx_deg(truthQuaternion);
error=mod(estimated-truth+180,360)-180; labels={'Yaw','Pitch','Roll'};
f=new_figure(1300,470); tl=tiledlayout(f,1,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,time(idx),error(idx,j),'LineWidth',0.8,'Color',[0.84 0.15 0.16]);
    yline(ax,0,'k-','LineWidth',0.6); title(ax,labels{j}); xlabel(ax,'Time [s]'); grid(ax,'on');
    if j==1; ylabel(ax,'Angle error [deg]'); end
end
end

function angles=euler_zyx_deg(q)
q=q./vecnorm(q,2,2); w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
yaw=atan2d(2*(w.*z+x.*y),1-2*(y.^2+z.^2));
pitch=asind(max(-1,min(1,2*(w.*y-z.*x))));
roll=atan2d(2*(w.*x+y.*z),1-2*(x.^2+y.^2));
angles=[yaw,pitch,roll];
end
