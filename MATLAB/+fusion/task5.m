function result = task5(imu, gnss, task1, task4, cfg, outDir)
%TASK5 Fuse IMU prediction with asynchronous GNSS measurements.

C=task1.c_ecef_to_ned; origin=task1.origin_ecef_m;
gnssPosition=(C*(gnss.position_ecef_m-origin)')'; gnssVelocity=(C*gnss.velocity_ecef_mps')';
state=[gnssPosition(1,:),gnssVelocity(1,:)]';
P=diag([repmat(cfg.gnss_position_std_m^2,1,3),repmat(cfg.gnss_velocity_std_mps^2,1,3)]); R=P; I=eye(6);
n=numel(imu.time_s); position=zeros(n,3); velocity=zeros(n,3); innovations=zeros(gnss.rows,6); updateTimes=zeros(gnss.rows,1); g=1;
for i=1:n
    if i==1; dt=imu.dt_s; else; dt=imu.time_s(i)-imu.time_s(i-1); end
    F=[eye(3),dt*eye(3);zeros(3),eye(3)]; G=[.5*dt^2*eye(3);dt*eye(3)];
    state=F*state+G*task4.acceleration(i,:)'; P=F*P*F'+cfg.process_accel_std_mps2^2*(G*G');
    while g<=gnss.rows && gnss.time_s(g)<=imu.time_s(i)+dt/2
        measurement=[gnssPosition(g,:),gnssVelocity(g,:)]'; innovation=measurement-state; S=P+R; K=P/S;
        state=state+K*innovation; P=(I-K)*P*(I-K)'+K*R*K'; innovations(g,:)=innovation'; updateTimes(g)=imu.time_s(i); g=g+1;
    end
    position(i,:)=state(1:3)'; velocity(i,:)=state(4:6)';
end
innovations=innovations(1:g-1,:); updateTimes=updateTimes(1:g-1);
time_s=imu.time_s; position_ned_m=position; velocity_ned_mps=velocity; acceleration_ned_mps2=task4.acceleration; quaternion_wxyz=task4.quaternion;
save(fullfile(outDir,'fused_solution.mat'),'time_s','position_ned_m','velocity_ned_mps','acceleration_ned_mps2','quaternion_wxyz','innovations','updateTimes');
result=struct('task',5,'name','GNSS/IMU Kalman fusion','subtasks',{{'5.1 Predict NED state','5.2 Update with GNSS','5.3 Save innovations'}}, ...
    'samples',n,'gnss_updates',size(innovations,1),'artifact',fullfile(outDir,'fused_solution.mat'), ...
    'final_position_ned_m',position(end,:),'final_velocity_ned_mps',velocity(end,:), ...
    'time_s',time_s,'position',position,'velocity',velocity,'acceleration',task4.acceleration, ...
    'quaternion',task4.quaternion,'innovations',innovations,'innovation_times',updateTimes);
summary=rmfield(result,{'time_s','position','velocity','acceleration','quaternion','innovations','innovation_times'}); fusion.write_json(fullfile(outDir,'summary.json'),summary);
if cfg.plots; state_plot(fullfile(outDir,'fused_solution.png'),time_s,position,velocity,cfg); end
end

function state_plot(path,time,position,velocity,cfg)
stride=max(1,ceil(numel(time)/cfg.max_plot_points)); idx=1:stride:numel(time); labels={'North','East','Down'};
f=figure('Visible','off'); tiledlayout(2,3);
for j=1:3; nexttile(j); plot(time(idx),position(idx,j)); title(labels{j}); ylabel('Position [m]'); grid on;
    nexttile(j+3); plot(time(idx),velocity(idx,j)); ylabel('Velocity [m/s]'); xlabel('Time [s]'); grid on; end
sgtitle('Task 5 - GNSS/IMU fused solution'); exportgraphics(f,path,'Resolution',160); close(f);
end
