function [result, figCtx] = task5(imu, gnss, task1, task4, cfg, outDir, figCtx)
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
result=struct('task',5,'name','GNSS/IMU Kalman fusion','subtasks',{{'5.1 Predict NED state', ...
    '5.2 Update with GNSS','5.3 Save innovations', ...
    '5.10 Present fused state in NED/ECEF/Body'}}, ...
    'samples',n,'gnss_updates',size(innovations,1),'artifact',fullfile(outDir,'fused_solution.mat'), ...
    'final_position_ned_m',position(end,:),'final_velocity_ned_mps',velocity(end,:), ...
    'time_s',time_s,'position',position,'velocity',velocity,'acceleration',task4.acceleration, ...
    'quaternion',task4.quaternion,'innovations',innovations,'innovation_times',updateTimes);
summary=rmfield(result,{'time_s','position','velocity','acceleration','quaternion','innovations','innovation_times'}); fusion.write_json(fullfile(outDir,'summary.json'),summary);
if cfg.plots
    stride=max(1,ceil(numel(time_s)/cfg.max_plot_points)); idx=1:stride:numel(time_s);
    figCtx=fusion.figures('save',figCtx,5,'prediction_vs_gnss',outDir, ...
        prediction_plot(time_s,position,velocity,gnss.time_s,gnssPosition,gnssVelocity,idx));
    figCtx=fusion.figures('save',figCtx,5,'kalman_innovations',outDir, ...
        innovation_plot(updateTimes,innovations));
    figCtx=fusion.figures('save',figCtx,5,'fused_position_velocity',outDir, ...
        state_plot(time_s,position,velocity,idx));
    figCtx=fusion.figures('save',figCtx,5,'fused_ground_track',outDir, ...
        ground_track_plot(position,gnssPosition,idx));
    figCtx=fusion.figures('save',figCtx,5,'fused_vs_imu_only',outDir, ...
        comparison_plot(time_s,position,velocity,task4.position,task4.velocity,idx));
    figCtx=fusion.figures('save',figCtx,5,'fused_state_ned',outDir, ...
        fused_kinematic_plot(time_s(idx),position(idx,:),velocity(idx,:), ...
        task4.acceleration(idx,:),{'North','East','Down'}));

    c_ned_to_ecef=task1.c_ecef_to_ned'; origin_ecef=task1.origin_ecef_m;
    position_ecef=position(idx,:)*c_ned_to_ecef'+origin_ecef;
    velocity_ecef=velocity(idx,:)*c_ned_to_ecef';
    acceleration_ecef=task4.acceleration(idx,:)*c_ned_to_ecef';
    figCtx=fusion.figures('save',figCtx,5,'fused_state_ecef',outDir, ...
        fused_kinematic_plot(time_s(idx),position_ecef,velocity_ecef,acceleration_ecef, ...
        {'X','Y','Z'}));

    position_body=rotate_ned_to_body(position(idx,:),task4.quaternion(idx,:));
    velocity_body=rotate_ned_to_body(velocity(idx,:),task4.quaternion(idx,:));
    acceleration_body=rotate_ned_to_body(task4.acceleration(idx,:),task4.quaternion(idx,:));
    figCtx=fusion.figures('save',figCtx,5,'fused_state_body',outDir, ...
        fused_kinematic_plot(time_s(idx),position_body,velocity_body,acceleration_body, ...
        {'Body x','Body y','Body z'}));
end
end

function f=new_figure(width,height)
% Sized explicitly: the stamped two-line sgtitle and the footer annotation need
% more room than the default figure gives a 2x3 grid.
f=figure('Visible','off','Position',[100 100 width height]);
end

function f=prediction_plot(time,position,velocity,gnssTime,gnssPosition,gnssVelocity,idx)
labels={'North','East','Down'}; crimson=[0.8627 0.0784 0.2353];
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,time(idx),position(idx,j),'LineWidth',0.9); hold(ax,'on');
    scatter(ax,gnssTime,gnssPosition(:,j),9,crimson,'filled');
    title(ax,labels{j}); ylabel(ax,'Position [m]'); grid(ax,'on');
    if j==1; legend(ax,{'filter state','GNSS'},'FontSize',8); end
    ax=nexttile(tl,j+3); plot(ax,time(idx),velocity(idx,j),'LineWidth',0.9); hold(ax,'on');
    scatter(ax,gnssTime,gnssVelocity(:,j),9,crimson,'filled');
    ylabel(ax,'Velocity [m/s]'); xlabel(ax,'Time [s]'); grid(ax,'on');
end
end

function f=innovation_plot(updateTimes,innovations)
labels={'North','East','Down'}; orange=[1 0.4980 0.0549];
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
if isempty(innovations)
    for k=1:6
        ax=nexttile(tl,k); xlim(ax,[0 1]); ylim(ax,[0 1]);
        text(ax,0.5,0.5,'no GNSS updates','HorizontalAlignment','center','VerticalAlignment','middle');
    end
    return
end
for j=1:3
    ax=nexttile(tl,j); plot(ax,updateTimes,innovations(:,j),'-o','MarkerSize',2.5,'LineWidth',0.7);
    yline(ax,0,'k-','LineWidth',0.6);
    title(ax,labels{j}); ylabel(ax,'Position innovation [m]'); grid(ax,'on');
    ax=nexttile(tl,j+3); plot(ax,updateTimes,innovations(:,j+3),'-o','MarkerSize',2.5,'LineWidth',0.7,'Color',orange);
    yline(ax,0,'k-','LineWidth',0.6);
    ylabel(ax,'Velocity innovation [m/s]'); xlabel(ax,'Time [s]'); grid(ax,'on');
end
end

function f=state_plot(time,position,velocity,idx)
labels={'North','East','Down'};
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,time(idx),position(idx,j),'LineWidth',0.9);
    title(ax,labels{j}); ylabel(ax,'Position [m]'); grid(ax,'on');
    ax=nexttile(tl,j+3); plot(ax,time(idx),velocity(idx,j),'LineWidth',0.9);
    ylabel(ax,'Velocity [m/s]'); xlabel(ax,'Time [s]'); grid(ax,'on');
end
end

function f=ground_track_plot(position,gnssPosition,idx)
crimson=[0.8627 0.0784 0.2353];
f=new_figure(800,700); tl=tiledlayout(f,1,1); ax=nexttile(tl,1);
plot(ax,position(idx,2),position(idx,1),'LineWidth',1.0); hold(ax,'on');
scatter(ax,gnssPosition(:,2),gnssPosition(:,1),14,crimson,'filled');
axis(ax,'equal'); xlabel(ax,'East [m]'); ylabel(ax,'North [m]'); grid(ax,'on');
legend(ax,{'fused','GNSS fixes'},'FontSize',8);
end

function f=comparison_plot(time,position,velocity,imuPosition,imuVelocity,idx)
labels={'North','East','Down'};
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,time(idx),position(idx,j),'LineWidth',0.9); hold(ax,'on');
    plot(ax,time(idx),imuPosition(idx,j),'--','LineWidth',0.8);
    title(ax,labels{j}); ylabel(ax,'Position [m]'); grid(ax,'on');
    if j==1; legend(ax,{'Fused','IMU-only'},'FontSize',8); end
    ax=nexttile(tl,j+3); plot(ax,time(idx),velocity(idx,j),'LineWidth',0.9); hold(ax,'on');
    plot(ax,time(idx),imuVelocity(idx,j),'--','LineWidth',0.8);
    ylabel(ax,'Velocity [m/s]'); xlabel(ax,'Time [s]'); grid(ax,'on');
end
end

function body=rotate_ned_to_body(values,quaternion)
body=zeros(size(values));
for i=1:size(values,1)
    dcm_body_to_ned=fusion.math3d('quaternion_to_matrix',quaternion(i,:));
    body(i,:)=(dcm_body_to_ned'*values(i,:)')';
end
end

function f=fused_kinematic_plot(time,position,velocity,acceleration,labels)
f=new_figure(1450,920); tl=tiledlayout(f,3,3);
values={position,velocity,acceleration};
yLabels={'Position [m]','Velocity [m/s]','Acceleration [m/s^2]'};
for row=1:3
    for column=1:3
        ax=nexttile(tl,(row-1)*3+column);
        plot(ax,time,values{row}(:,column),'LineWidth',0.85); grid(ax,'on');
        if row==1; title(ax,labels{column}); end
        if row==3; xlabel(ax,'Time [s]'); end
        if column==1; ylabel(ax,yLabels{row}); end
        if row==1 && column==1
            legend(ax,{'Fused GNSS + IMU'},'FontSize',8,'Location','best');
        end
    end
end
end
