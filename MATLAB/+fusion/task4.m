function [result, figCtx] = task4(imu, gnss, task1, task3, cfg, outDir, figCtx)
%TASK4 Propagate normalized attitude and IMU-only NED trajectory.

n=numel(imu.time_s); q=zeros(n,4); acceleration=zeros(n,3); current=task3.quaternion_wxyz_body_to_ned;
[accel,accelRejected]=screen_ranges(imu.time_s,imu.accel_mps2,cfg.max_specific_force_mps2);
[gyro,gyroRejected]=screen_ranges(imu.time_s,imu.gyro_rps,cfg.max_angular_rate_rps);
gravity=[0;0;task1.gravity_mps2]; position=zeros(n,3); velocity=zeros(n,3);
velocity(1,:)=(task1.c_ecef_to_ned*gnss.velocity_ecef_mps(1,:)')'; e2=6.69437999014e-3; a=6378137.0;
for i=1:n
    if i==1; dt=imu.dt_s; else; dt=imu.time_s(i)-imu.time_s(i-1); end
    stateIndex=max(1,i-1); latitude=task1.lat_rad+position(stateIndex,1)/6.35e6;
    altitude=task1.altitude_m-position(stateIndex,3); denom=sqrt(1-e2*sin(latitude)^2);
    radiusEast=a/denom; radiusNorth=a*(1-e2)/denom^3; previousVelocity=velocity(stateIndex,:)';
    omegaIE=cfg.earth_rate_rps*[cos(latitude);0;-sin(latitude)];
    omegaEN=[previousVelocity(2)/(radiusEast+altitude); ...
        -previousVelocity(1)/(radiusNorth+altitude); ...
        -previousVelocity(2)*tan(latitude)/(radiusEast+altitude)];
    C=fusion.math3d('quaternion_to_matrix',current);
    bodyRate=gyro(i,:)'-task3.gyro_bias_body_rps'-C'*(omegaIE+omegaEN);
    current=fusion.math3d('quaternion_multiply',current,fusion.math3d('quaternion_from_rotvec',bodyRate*dt));
    current=current/norm(current); if i>1 && dot(current,q(i-1,:))<0; current=-current; end
    C=fusion.math3d('quaternion_to_matrix',current);
    coriolis=-cross(2*omegaIE+omegaEN,previousVelocity);
    acceleration(i,:)=(C*(accel(i,:)'-task3.accel_bias_body_mps2')+gravity+coriolis)'; q(i,:)=current;
    if i>1
        velocity(i,:)=velocity(i-1,:)+.5*(acceleration(i-1,:)+acceleration(i,:))*dt;
        position(i,:)=position(i-1,:)+.5*(velocity(i-1,:)+velocity(i,:))*dt;
    end
end
time_s=imu.time_s; position_ned_m=position; velocity_ned_mps=velocity; acceleration_ned_mps2=acceleration; quaternion_wxyz=q;
save(fullfile(outDir,'inertial_solution.mat'),'time_s','position_ned_m','velocity_ned_mps','acceleration_ned_mps2','quaternion_wxyz');
result=struct('task',4,'name','IMU-only strapdown propagation', ...
    'subtasks',{{'4.1 Screen outliers and correct IMU','4.2 Propagate with Earth/transport rates', ...
    '4.3 Coriolis-compensated NED integration','4.6 Compare GNSS-derived and IMU-derived kinematics in NED/ECEF/Body'}}, ...
    'samples',n,'duration_s',time_s(end),'artifact',fullfile(outDir,'inertial_solution.mat'), ...
    'range_screening',struct('accelerometer_samples_interpolated',accelRejected, ...
    'gyroscope_samples_interpolated',gyroRejected), ...
    'final_position_ned_m',position(end,:),'final_velocity_ned_mps',velocity(end,:), ...
    'time_s',time_s,'position',position,'velocity',velocity,'acceleration',acceleration,'quaternion',q);
summary=rmfield(result,{'time_s','position','velocity','acceleration','quaternion'}); fusion.write_json(fullfile(outDir,'summary.json'),summary);
if cfg.plots
    stride=max(1,ceil(numel(time_s)/cfg.max_plot_points)); idx=1:stride:numel(time_s);
    figCtx=fusion.figures('save',figCtx,4,'range_screening',outDir,screening_plot(imu,cfg,result.range_screening,idx));
    figCtx=fusion.figures('save',figCtx,4,'bias_corrected_imu',outDir,bias_plot(time_s,accel,gyro,task3,idx));
    figCtx=fusion.figures('save',figCtx,4,'propagated_quaternion',outDir,quaternion_plot(time_s,q,idx));
    figCtx=fusion.figures('save',figCtx,4,'propagated_euler_angles',outDir,euler_plot(time_s,q,idx));
    figCtx=fusion.figures('save',figCtx,4,'imu_only_position_velocity',outDir,state_plot(time_s,position,velocity,idx));
    figCtx=fusion.figures('save',figCtx,4,'imu_only_acceleration',outDir,acceleration_plot(time_s,acceleration,idx));
    figCtx=fusion.figures('save',figCtx,4,'imu_only_ground_track',outDir,ground_track_plot(position,idx));

    c_ecef_to_ned=task1.c_ecef_to_ned; c_ned_to_ecef=c_ecef_to_ned';
    origin_ecef=task1.origin_ecef_m;
    gnss_position_ned=(c_ecef_to_ned*(gnss.position_ecef_m-origin_ecef)')';
    gnss_velocity_ned=(c_ecef_to_ned*gnss.velocity_ecef_mps')';
    gnss_acceleration_ned=differentiate_series(gnss.velocity_ecef_mps*c_ecef_to_ned',gnss.time_s);
    figCtx=fusion.figures('save',figCtx,4,'gnss_vs_imu_ned',outDir, ...
        kinematic_comparison_plot(gnss.time_s,gnss_position_ned,gnss_velocity_ned, ...
        gnss_acceleration_ned,time_s(idx),position(idx,:),velocity(idx,:),acceleration(idx,:), ...
        {'North','East','Down'}));

    imu_position_ecef=position(idx,:)*c_ned_to_ecef'+origin_ecef;
    imu_velocity_ecef=velocity(idx,:)*c_ned_to_ecef';
    imu_acceleration_ecef=acceleration(idx,:)*c_ned_to_ecef';
    gnss_acceleration_ecef=differentiate_series(gnss.velocity_ecef_mps,gnss.time_s);
    figCtx=fusion.figures('save',figCtx,4,'gnss_vs_imu_ecef',outDir, ...
        kinematic_comparison_plot(gnss.time_s,gnss.position_ecef_m,gnss.velocity_ecef_mps, ...
        gnss_acceleration_ecef,time_s(idx),imu_position_ecef,imu_velocity_ecef, ...
        imu_acceleration_ecef,{'X','Y','Z'}));

    gnss_quaternion=interp1(time_s,q,gnss.time_s,'linear','extrap');
    gnss_quaternion=gnss_quaternion./vecnorm(gnss_quaternion,2,2);
    gnss_position_body=rotate_ned_to_body(gnss_position_ned,gnss_quaternion);
    gnss_velocity_body=rotate_ned_to_body(gnss_velocity_ned,gnss_quaternion);
    gnss_acceleration_body=rotate_ned_to_body(gnss_acceleration_ned,gnss_quaternion);
    imu_position_body=rotate_ned_to_body(position(idx,:),q(idx,:));
    imu_velocity_body=rotate_ned_to_body(velocity(idx,:),q(idx,:));
    imu_acceleration_body=rotate_ned_to_body(acceleration(idx,:),q(idx,:));
    figCtx=fusion.figures('save',figCtx,4,'gnss_vs_imu_body',outDir, ...
        kinematic_comparison_plot(gnss.time_s,gnss_position_body,gnss_velocity_body, ...
        gnss_acceleration_body,time_s(idx),imu_position_body,imu_velocity_body, ...
        imu_acceleration_body,{'Body x','Body y','Body z'}));
end
end

function [repaired,rejected]=screen_ranges(time,values,limit)
valid=all(isfinite(values),2) & vecnorm(values,2,2)<=limit; rejected=sum(~valid);
if rejected==0; repaired=values; return; end
if sum(valid)<2; error('fusion:Task4','Fewer than two IMU samples remain below limit %g.',limit); end
repaired=zeros(size(values));
for j=1:size(values,2); repaired(:,j)=interp1(time(valid),values(valid,j),time,'linear','extrap'); end
end

function f=screening_plot(imu,cfg,counts,idx)
t=imu.time_s(idx);
f=figure('Visible','off'); tiledlayout(2,1);
ax=nexttile; plot(t,vecnorm(imu.accel_mps2(idx,:),2,2)); yline(cfg.max_specific_force_mps2,'--','limit');
ylabel(ax,'|specific force| [m/s^2]'); grid(ax,'on');
text(ax,0.02,0.92,sprintf('interpolated: %d accel, %d gyro', ...
    counts.accelerometer_samples_interpolated,counts.gyroscope_samples_interpolated), ...
    'Units','normalized','FontSize',8,'BackgroundColor','w');
ax=nexttile; plot(t,vecnorm(imu.gyro_rps(idx,:),2,2)); yline(cfg.max_angular_rate_rps,'--','limit');
ylabel(ax,'|angular rate| [rad/s]'); xlabel(ax,'Time [s]'); grid(ax,'on');
end

function f=bias_plot(time,accel,gyro,task3,idx)
t=time(idx); labels={'Body x','Body y','Body z'};
f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3
    ax=nexttile(j); plot(ax,t,accel(idx,j)); hold(ax,'on');
    plot(ax,t,accel(idx,j)-task3.accel_bias_body_mps2(j)); hold(ax,'off');
    title(ax,labels{j}); grid(ax,'on');
    ax2=nexttile(j+3); plot(ax2,t,gyro(idx,j)); hold(ax2,'on');
    plot(ax2,t,gyro(idx,j)-task3.gyro_bias_body_rps(j)); hold(ax2,'off');
    xlabel(ax2,'Time [s]'); grid(ax2,'on');
    if j==1
        ylabel(ax,'Specific force [m/s^2]'); ylabel(ax2,'Angular rate [rad/s]');
        legend(ax,{'screened','bias removed'},'FontSize',8,'Location','best');
    end
end
end

function f=quaternion_plot(time,q,idx)
t=time(idx); labels={'qw','qx','qy','qz'};
f=figure('Visible','off'); tiledlayout(2,2);
for k=1:4
    ax=nexttile; plot(ax,t,q(idx,k)); title(ax,labels{k}); grid(ax,'on');
    % Components sit near +-1 with tiny variation; a fixed tick format keeps the
    % full value on every tick instead of an offset/exponent header.
    ax.YAxis.Exponent=0; ytickformat(ax,'%.6f');
    if k>=3; xlabel(ax,'Time [s]'); end
end
end

function f=euler_plot(time,q,idx)
t=time(idx); angles=quaternion_to_euler_zyx_deg(q(idx,:)); labels={'Yaw','Pitch','Roll'};
f=figure('Visible','off'); tiledlayout(1,3);
for j=1:3
    ax=nexttile; plot(ax,t,angles(:,j)); title(ax,labels{j}); xlabel(ax,'Time [s]'); grid(ax,'on');
    if j==1; ylabel(ax,'Angle [deg]'); end
end
end

function f=state_plot(time,position,velocity,idx)
labels={'North','East','Down'};
f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3; nexttile(j); plot(time(idx),position(idx,j)); title(labels{j}); ylabel('Position [m]'); grid on;
    nexttile(j+3); plot(time(idx),velocity(idx,j)); ylabel('Velocity [m/s]'); xlabel('Time [s]'); grid on; end
end

function f=acceleration_plot(time,acceleration,idx)
t=time(idx); labels={'North','East','Down'};
f=figure('Visible','off'); tiledlayout(1,3);
for j=1:3
    ax=nexttile; plot(ax,t,acceleration(idx,j)); title(ax,labels{j}); xlabel(ax,'Time [s]'); grid(ax,'on');
    if j==1; ylabel(ax,'Acceleration [m/s^2]'); end
end
end

function f=ground_track_plot(position,idx)
east=position(idx,2); north=position(idx,1);
f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
plot(ax,east,north); hold(ax,'on');
hStart=scatter(ax,east(1),north(1),36,[0 0.5 0],'filled');
hEnd=scatter(ax,east(end),north(end),36,[0.86 0.08 0.24],'filled'); hold(ax,'off');
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'East [m]'); ylabel(ax,'North [m]');
legend([hStart hEnd],{'start','end'},'FontSize',8,'Location','best');
end

function derivative=differentiate_series(values,time)
derivative=zeros(size(values));
for j=1:size(values,2)
    derivative(:,j)=gradient(values(:,j),time);
end
end

function body=rotate_ned_to_body(values,quaternion)
body=zeros(size(values));
for i=1:size(values,1)
    dcm_body_to_ned=fusion.math3d('quaternion_to_matrix',quaternion(i,:));
    body(i,:)=(dcm_body_to_ned'*values(i,:)')';
end
end

function f=kinematic_comparison_plot(gnssTime,gnssPosition,gnssVelocity,gnssAcceleration, ...
    imuTime,imuPosition,imuVelocity,imuAcceleration,labels)
f=figure('Visible','off','Position',[100 100 1450 920]); tl=tiledlayout(f,3,3);
gnssValues={gnssPosition,gnssVelocity,gnssAcceleration};
imuValues={imuPosition,imuVelocity,imuAcceleration};
yLabels={'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
for row=1:3
    for column=1:3
        ax=nexttile(tl,(row-1)*3+column);
        plot(ax,gnssTime,gnssValues{row}(:,column),'k--','LineWidth',0.9); hold(ax,'on');
        plot(ax,imuTime,imuValues{row}(:,column),'LineWidth',0.8); hold(ax,'off');
        grid(ax,'on');
        if row==1; title(ax,labels{column}); end
        if row==3; xlabel(ax,'Time [s]'); end
        if column==1; ylabel(ax,yLabels{row}); end
        if row==1 && column==1
            legend(ax,{'GNSS derived','IMU derived'},'FontSize',8,'Location','best');
        end
    end
end
end

function angles=quaternion_to_euler_zyx_deg(q)
q=q./vecnorm(q,2,2); w=q(:,1); x=q(:,2); y=q(:,3); z=q(:,4);
yaw=atan2d(2*(w.*z+x.*y),1-2*(y.^2+z.^2));
pitch=asind(max(-1,min(1,2*(w.*y-z.*x))));
roll=atan2d(2*(w.*x+y.*z),1-2*(x.^2+y.^2));
angles=[yaw,pitch,roll];
end
