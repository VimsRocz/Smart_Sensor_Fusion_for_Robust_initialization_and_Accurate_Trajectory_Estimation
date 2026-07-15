function result = task4(imu, gnss, task1, task3, cfg, outDir)
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
    'subtasks',{{'4.1 Screen outliers and correct IMU','4.2 Propagate with Earth/transport rates','4.3 Coriolis-compensated NED integration'}}, ...
    'samples',n,'duration_s',time_s(end),'artifact',fullfile(outDir,'inertial_solution.mat'), ...
    'range_screening',struct('accelerometer_samples_interpolated',accelRejected, ...
    'gyroscope_samples_interpolated',gyroRejected), ...
    'final_position_ned_m',position(end,:),'final_velocity_ned_mps',velocity(end,:), ...
    'time_s',time_s,'position',position,'velocity',velocity,'acceleration',acceleration,'quaternion',q);
summary=rmfield(result,{'time_s','position','velocity','acceleration','quaternion'}); fusion.write_json(fullfile(outDir,'summary.json'),summary);
if cfg.plots; state_plot(fullfile(outDir,'inertial_solution.png'),'Task 4 - IMU-only solution',time_s,position,velocity,cfg); end
end

function [repaired,rejected]=screen_ranges(time,values,limit)
valid=all(isfinite(values),2) & vecnorm(values,2,2)<=limit; rejected=sum(~valid);
if rejected==0; repaired=values; return; end
if sum(valid)<2; error('fusion:Task4','Fewer than two IMU samples remain below limit %g.',limit); end
repaired=zeros(size(values));
for j=1:size(values,2); repaired(:,j)=interp1(time(valid),values(valid,j),time,'linear','extrap'); end
end

function state_plot(path,titleText,time,position,velocity,cfg)
stride=max(1,ceil(numel(time)/cfg.max_plot_points)); idx=1:stride:numel(time); labels={'North','East','Down'};
f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3; nexttile(j); plot(time(idx),position(idx,j)); title(labels{j}); ylabel('Position [m]'); grid on;
    nexttile(j+3); plot(time(idx),velocity(idx,j)); ylabel('Velocity [m/s]'); xlabel('Time [s]'); grid on; end
sgtitle(titleText); exportgraphics(f,path,'Resolution',160); close(f);
end
