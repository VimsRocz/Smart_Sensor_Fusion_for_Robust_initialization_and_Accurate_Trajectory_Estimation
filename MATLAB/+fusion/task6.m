function result = task6(truth, task1, task5, cfg, outDir)
%TASK6 Align truth in the common NED frame and correct height/quaternion plots.

subtasks={'6.1 Align truth time','6.2 Convert truth ECEF to common NED', ...
    '6.3 Compare height as -NED Down','6.4 Normalize and sign-align quaternions'};
if isempty(truth)
    result=struct('task',6,'name','Truth overlay','status','skipped', ...
        'reason','No truth file supplied; Tasks 1-5 remain valid','subtasks',{subtasks});
    fusion.write_json(fullfile(outDir,'summary.json'),result); return
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
if cfg.plots; make_plots(outDir,result.overlay,cfg); end
end

function make_plots(outDir,o,cfg)
stride=max(1,ceil(numel(o.time_s)/cfg.max_plot_points)); idx=1:stride:numel(o.time_s); t=o.time_s(idx); labels={'North','East','Down'};
f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3
    nexttile(j); plot(t,o.estimated_position(idx,j),t,o.truth_position(idx,j),'--'); title(labels{j}); ylabel('Position [m]'); grid on;
    nexttile(j+3); plot(t,o.estimated_velocity(idx,j),t,o.truth_velocity(idx,j),'--'); ylabel('Velocity [m/s]'); xlabel('Time [s]'); grid on;
end
legend('Fused','Truth'); sgtitle('Task 6 - Common-frame NED truth overlay'); exportgraphics(f,fullfile(outDir,'truth_overlay_ned.png'),'Resolution',160); close(f);
f=figure('Visible','off'); plot(t,-o.estimated_position(idx,3),t,-o.truth_position(idx,3),'--'); grid on; legend('Fused height','Truth height');
xlabel('Time [s]'); ylabel('Relative height [m]'); title('Task 6.3 - Height = -NED Down'); exportgraphics(f,fullfile(outDir,'height_comparison.png'),'Resolution',160); close(f);
if ~isempty(o.truth_quaternion)
    f=figure('Visible','off'); tiledlayout(2,2); names={'w','x','y','z'};
    for j=1:4; nexttile; plot(t,o.estimated_quaternion(idx,j),t,o.truth_quaternion(idx,j),'--'); title(['q' names{j}]); grid on; end
    legend('Fused','Truth'); sgtitle('Task 6.4 - Normalized Body-to-NED quaternions'); exportgraphics(f,fullfile(outDir,'quaternion_comparison.png'),'Resolution',160); close(f);
end
end
