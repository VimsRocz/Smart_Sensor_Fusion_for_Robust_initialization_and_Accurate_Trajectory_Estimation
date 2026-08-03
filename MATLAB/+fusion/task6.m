function [result, figCtx] = task6(truth, task1, task5, cfg, outDir, figCtx)
%TASK6 Align truth in the common NED frame and correct height/quaternion plots.

subtasks={'6.1 Align truth time','6.2 Convert truth ECEF to common NED', ...
    '6.3 Compare height as -NED Down','6.4 Normalize and sign-align quaternions'};
if isempty(truth)
    result=struct('task',6,'name','Truth overlay','status','skipped', ...
        'reason','No truth file supplied; Tasks 1-5 remain valid','subtasks',{subtasks});
    fusion.write_json(fullfile(outDir,'summary.json'),result);
    if cfg.plots
        slugs=catalog_slugs();
        for k=1:numel(slugs); figCtx=fusion.figures('skip',figCtx,6,slugs{k},'no truth file was supplied'); end
    end
    return
end
last=min(truth.time_s(end),task5.time_s(end)); mask=task5.time_s<=last; time=task5.time_s(mask);
if numel(time)<2; error('fusion:Task6','Truth and fused solution have no useful overlap.'); end
C=task1.c_ecef_to_ned; origin=task1.origin_ecef_m;
truthPosAll=(C*(truth.position_ecef_m-origin)')'; truthVelAll=(C*truth.velocity_ecef_mps')';
truthPosition=interp1(truth.time_s,truthPosAll,time,'linear'); truthVelocity=interp1(truth.time_s,truthVelAll,time,'linear');
estimatedPosition=task5.position(mask,:); estimatedVelocity=task5.velocity(mask,:); estimatedQuaternion=task5.quaternion(mask,:);
truthQuaternion=[];
if ~isempty(truth.quaternion_wxyz)
    truthSource=truth.quaternion_wxyz;
    if strcmpi(cfg.truth_quaternion_frame,'body_to_ecef')
        converted=zeros(size(truthSource));
        for i=1:size(truthSource,1)
            [localLat,localLon]=fusion.math3d('ecef_to_geodetic',truth.position_ecef_m(i,:));
            localC=fusion.math3d('ecef_to_ned_matrix',localLat,localLon);
            converted(i,:)=fusion.math3d('matrix_to_quaternion',localC*fusion.math3d('quaternion_to_matrix',truthSource(i,:)));
        end
        truthSource=converted;
    end
    for i=2:size(truthSource,1); if dot(truthSource(i-1,:),truthSource(i,:))<0; truthSource(i,:)=-truthSource(i,:); end; end
    attitudeTime=min(max(time+cfg.truth_attitude_time_offset_s,truth.time_s(1)),truth.time_s(end));
    truthQuaternion=interp1(truth.time_s,truthSource,attitudeTime,'linear'); truthQuaternion=truthQuaternion./vecnorm(truthQuaternion,2,2);
    estimatedQuaternion=estimatedQuaternion./vecnorm(estimatedQuaternion,2,2);
    flip=sum(truthQuaternion.*estimatedQuaternion,2)<0; estimatedQuaternion(flip,:)=-estimatedQuaternion(flip,:);
end
time_s=time; estimated_position_ned_m=estimatedPosition; truth_position_ned_m=truthPosition;
estimated_velocity_ned_mps=estimatedVelocity; truth_velocity_ned_mps=truthVelocity;
estimated_quaternion_wxyz=estimatedQuaternion; truth_quaternion_wxyz=truthQuaternion;
save(fullfile(outDir,'truth_overlay.mat'),'time_s','estimated_position_ned_m','truth_position_ned_m', ...
    'estimated_velocity_ned_mps','truth_velocity_ned_mps','estimated_quaternion_wxyz','truth_quaternion_wxyz');
result=struct('task',6,'name','Truth overlay','status','complete','subtasks',{subtasks}, ...
    'samples',numel(time),'artifact',fullfile(outDir,'truth_overlay.mat'), ...
    'height_definition','height_m = -position_ned_down_m', ...
    'quaternion_convention','scalar-first [w,x,y,z], Body-to-NED, normalized and truth-hemisphere aligned', ...
    'truth_source_quaternion_order',cfg.truth_quaternion_order, ...
    'truth_source_quaternion_frame',cfg.truth_quaternion_frame, ...
    'attitude_reference_frame','time-varying local NED at each truth ECEF position', ...
    'truth_attitude_time_offset_s',cfg.truth_attitude_time_offset_s, ...
    'overlay',struct('time_s',time,'estimated_position',estimatedPosition,'truth_position',truthPosition, ...
    'estimated_velocity',estimatedVelocity,'truth_velocity',truthVelocity, ...
    'estimated_quaternion',estimatedQuaternion,'truth_quaternion',truthQuaternion));
summary=rmfield(result,'overlay'); fusion.write_json(fullfile(outDir,'summary.json'),summary);
if cfg.plots; figCtx=make_plots(figCtx,outDir,result.overlay,cfg,task5.time_s,truth.time_s); end
end

function slugs = catalog_slugs()
slugs={'truth_time_alignment','fused_vs_truth_position_velocity','fused_vs_truth_ground_track', ...
    'height_above_origin','quaternion_comparison','euler_angle_comparison'};
end

function figCtx = make_plots(figCtx,outDir,o,cfg,estimateTime,truthTime)
stride=max(1,ceil(numel(o.time_s)/cfg.max_plot_points)); idx=1:stride:numel(o.time_s); t=o.time_s(idx); labels={'North','East','Down'};

spans={'Estimate (Task 5)',estimateTime;'Truth (as loaded)',truthTime;'Common overlap',o.time_s};
colors=[0 .447 .741;.85 .325 .098;.466 .674 .188];
f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile; hold(ax,'on');
for k=1:size(spans,1)
    s=spans{k,2}; b=barh(ax,k,s(end),0.45);
    % BaseValue shifts the bar start so each bar spans [s(1), s(end)].
    set(b,'BaseValue',s(1),'FaceColor',colors(k,:),'FaceAlpha',0.75,'EdgeColor','none');
end
hold(ax,'off'); set(ax,'YTick',1:size(spans,1),'YTickLabel',spans(:,1)); ylim(ax,[0.4 size(spans,1)+0.6]);
xlabel(ax,'Time [s]'); grid(ax,'on');
text(ax,0.01,0.06,sprintf('attitude-only time offset applied: %+.3f s   |   %d common samples', ...
    cfg.truth_attitude_time_offset_s,numel(o.time_s)),'Units','normalized','FontSize',8, ...
    'BackgroundColor','w','EdgeColor',[.7 .7 .7],'Margin',3,'VerticalAlignment','bottom');
figCtx=fusion.figures('save',figCtx,6,'truth_time_alignment',outDir,f);

f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3
    nexttile(j); plot(t,o.estimated_position(idx,j),t,o.truth_position(idx,j),'--'); title(labels{j}); ylabel('Position [m]'); grid on;
    nexttile(j+3); plot(t,o.estimated_velocity(idx,j),t,o.truth_velocity(idx,j),'--'); ylabel('Velocity [m/s]'); xlabel('Time [s]'); grid on;
end
legend('Fused','Truth');
figCtx=fusion.figures('save',figCtx,6,'fused_vs_truth_position_velocity',outDir,f);

f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
plot(ax,o.estimated_position(idx,2),o.estimated_position(idx,1)); hold(ax,'on');
plot(ax,o.truth_position(idx,2),o.truth_position(idx,1),'--'); hold(ax,'off');
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'East [m]'); ylabel(ax,'North [m]'); legend(ax,{'fused','truth'});
figCtx=fusion.figures('save',figCtx,6,'fused_vs_truth_ground_track',outDir,f);

f=figure('Visible','off'); tiledlayout(2,1);
ax=nexttile; plot(ax,t,-o.estimated_position(idx,3),t,-o.truth_position(idx,3),'--'); grid(ax,'on');
ylabel(ax,'Height above origin [m]'); legend(ax,{'fused','truth'});
ax=nexttile; plot(ax,t,-(o.estimated_position(idx,3)-o.truth_position(idx,3))); yline(ax,0); grid(ax,'on');
xlabel(ax,'Time [s]'); ylabel(ax,'Fused - truth [m]');
figCtx=fusion.figures('save',figCtx,6,'height_above_origin',outDir,f);

if isempty(o.truth_quaternion)
    figCtx=fusion.figures('skip',figCtx,6,'quaternion_comparison','truth file has no quaternion columns');
    figCtx=fusion.figures('skip',figCtx,6,'euler_angle_comparison','truth file has no quaternion columns');
    return
end
f=figure('Visible','off'); tiledlayout(2,2); names={'w','x','y','z'};
for j=1:4
    nexttile; plot(t,o.estimated_quaternion(idx,j),t,o.truth_quaternion(idx,j),'--'); title(['q' names{j}]); grid on;
    if j>=3; xlabel('Time [s]'); end
end
legend('Fused','Truth');
figCtx=fusion.figures('save',figCtx,6,'quaternion_comparison',outDir,f);

estimatedEuler=euler_zyx_deg(o.estimated_quaternion(idx,:)); truthEuler=euler_zyx_deg(o.truth_quaternion(idx,:));
angleLabels={'Yaw','Pitch','Roll'};
f=figure('Visible','off'); tiledlayout(1,3);
for j=1:3
    nexttile; plot(t,estimatedEuler(:,j),t,truthEuler(:,j),'--'); title(angleLabels{j});
    xlabel('Time [s]'); ylabel('Angle [deg]'); grid on;
end
legend('Fused','Truth');
figCtx=fusion.figures('save',figCtx,6,'euler_angle_comparison',outDir,f);
end

function angles = euler_zyx_deg(q)
%EULER_ZYX_DEG Yaw/pitch/roll in degrees from scalar-first quaternion rows.
w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
yaw=atan2d(2*(w.*z+x.*y),1-2*(y.^2+z.^2));
pitch=asind(max(-1,min(1,2*(w.*y-z.*x))));
roll=atan2d(2*(w.*x+y.*z),1-2*(x.^2+y.^2));
angles=[yaw,pitch,roll];
end
