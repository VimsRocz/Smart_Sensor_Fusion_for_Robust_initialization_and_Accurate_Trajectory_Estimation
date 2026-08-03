function [result, figCtx] = task1(data, cfg, outDir, figCtx)
%TASK1 Validate inputs and compute WGS-84 navigation reference vectors.

origin=data.gnss.position_ecef_m(1,:); [lat,lon,alt]=fusion.math3d('ecef_to_geodetic',origin);
C=fusion.math3d('ecef_to_ned_matrix',lat,lon);
sin2=sin(lat)^2; normalGravity=9.7803253359*(1+0.00193185265241*sin2)/sqrt(1-0.00669437999013*sin2)-3.086e-6*alt;
gravity=normalGravity; if ~isempty(cfg.gravity_override_mps2); gravity=cfg.gravity_override_mps2; end
omega=cfg.earth_rate_rps*[cos(lat),0,-sin(lat)];
result=struct('task',1,'name','Input validation and reference navigation vectors', ...
    'subtasks',{{'1.1 Validate inputs','1.2 Derive WGS-84 origin','1.3 Compute gravity and Earth rate'}}, ...
    'origin_ecef_m',origin,'latitude_deg',rad2deg(lat),'longitude_deg',rad2deg(lon), ...
    'altitude_m',alt,'gravity_mps2',gravity,'normal_gravity_mps2',normalGravity, ...
    'gravity_was_overridden',~isempty(cfg.gravity_override_mps2), ...
    'specific_force_reference_ned_mps2',[0,0,-gravity], ...
    'earth_rate_reference_ned_rps',omega,'c_ecef_to_ned',C,'lat_rad',lat,'lon_rad',lon, ...
    'validation',struct('status','valid','imu_rows',data.imu.rows,'gnss_rows',data.gnss.rows, ...
    'truth_rows',truth_rows(data.truth),'imu_clock_wraps_repaired',data.imu.clock_wraps));
fusion.write_json(fullfile(outDir,'reference.json'),result); save(fullfile(outDir,'reference.mat'),'-struct','result');
if cfg.plots
    figCtx=coverage_figure(figCtx,data,outDir);
    figCtx=gnss_raw_figure(figCtx,data.gnss,outDir);
    figCtx=imu_raw_figure(figCtx,data.imu,cfg,outDir);
    figCtx=origin_map_figure(figCtx,result,outDir);
    figCtx=reference_vector_figure(figCtx,result,outDir);
end
end

function n=truth_rows(truth)
if isempty(truth); n=0; else; n=truth.rows; end
end

function figCtx=coverage_figure(figCtx,data,outDir)
names={'IMU','GNSS'};
spans=[data.imu.time_s(end)-data.imu.time_s(1),data.gnss.time_s(end)-data.gnss.time_s(1)];
notes={sprintf('%.1f Hz, %d rows',1/data.imu.dt_s,data.imu.rows),sprintf('%d epochs',data.gnss.rows)};
if ~isempty(data.truth)
    names{end+1}='Truth'; spans(end+1)=data.truth.time_s(end)-data.truth.time_s(1);
    notes{end+1}=sprintf('%d states',data.truth.rows);
end
f=new_figure(1100,380); tl=tiledlayout(f,1,1); ax=nexttile(tl); hold(ax,'on');
colors=get(ax,'ColorOrder');
for k=1:numel(names)
    % read_inputs zero-bases every stream, so each bar starts at t = 0.
    barh(ax,k,spans(k),0.45,'FaceColor',colors(1+mod(k-1,size(colors,1)),:), ...
        'FaceAlpha',0.75,'EdgeColor','none');
    text(ax,0,k+0.32,sprintf('%s: %s',names{k},notes{k}),'FontSize',8.5);
end
hold(ax,'off'); grid(ax,'on'); ylim(ax,[0.4,numel(names)+0.7]);
yticks(ax,1:numel(names)); yticklabels(ax,names); xlabel(ax,'Time since first sample [s]');
figCtx=fusion.figures('save',figCtx,1,'input_time_coverage',outDir,f);
end

function figCtx=gnss_raw_figure(figCtx,gnss,outDir)
t=gnss.time_s; titles={'ECEF X','ECEF Y','ECEF Z'}; velocityColor=[0.850,0.325,0.098];
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,t,gnss.position_ecef_m(:,j),'-o','MarkerSize',2.5,'LineWidth',0.9);
    % ECEF components are ~7e6 m, so the automatic exponent would hide the variation.
    ax.YAxis.Exponent=0; grid(ax,'on'); title(ax,titles{j});
    if j==1; ylabel(ax,'ECEF position [m]'); end
    ax=nexttile(tl,j+3); plot(ax,t,gnss.velocity_ecef_mps(:,j),'-o','MarkerSize',2.5, ...
        'LineWidth',0.9,'Color',velocityColor);
    grid(ax,'on'); xlabel(ax,'Time [s]');
    if j==1; ylabel(ax,'ECEF velocity [m/s]'); end
end
figCtx=fusion.figures('save',figCtx,1,'gnss_raw_position_velocity',outDir,f);
end

function figCtx=imu_raw_figure(figCtx,imu,cfg,outDir)
t=imu.time_s; stride=max(1,ceil(numel(t)/cfg.max_plot_points)); idx=1:stride:numel(t);
titles={'Body x','Body y','Body z'}; gyroColor=[0,0.447,0.741]; accelColor=[0.635,0.078,0.184];
f=new_figure(1300,760); tl=tiledlayout(f,2,3);
for j=1:3
    ax=nexttile(tl,j); plot(ax,t(idx),imu.gyro_rps(idx,j),'LineWidth',0.8,'Color',gyroColor);
    grid(ax,'on'); title(ax,titles{j});
    if j==1; ylabel(ax,'Angular rate [rad/s]'); end
    ax=nexttile(tl,j+3); plot(ax,t(idx),imu.accel_mps2(idx,j),'LineWidth',0.8,'Color',accelColor);
    grid(ax,'on'); xlabel(ax,'Time [s]');
    if j==1; ylabel(ax,'Specific force [m/s^2]'); end
end
figCtx=fusion.figures('save',figCtx,1,'imu_raw_rates',outDir,f);
end

function figCtx=origin_map_figure(figCtx,result,outDir)
latitude=result.latitude_deg; longitude=result.longitude_deg; span=0.05; marker=[0.863,0.078,0.235];
f=new_figure(1200,460); tl=tiledlayout(f,1,2);
ax=nexttile(tl); hold(ax,'on'); graticule=[0.53,0.53,0.53];
plot(ax,[-180,180],[0,0],'Color',graticule,'LineWidth',0.6);
plot(ax,[0,0],[-90,90],'Color',graticule,'LineWidth',0.6);
scatter(ax,longitude,latitude,55,marker,'filled'); hold(ax,'off'); grid(ax,'on');
xlim(ax,[-180,180]); ylim(ax,[-90,90]); xticks(ax,-180:60:180); yticks(ax,-90:30:90);
xlabel(ax,'Longitude [deg]'); ylabel(ax,'Latitude [deg]'); title(ax,'Global position');
ax=nexttile(tl); scatter(ax,longitude,latitude,70,marker,'filled'); grid(ax,'on');
xlim(ax,longitude+[-span,span]); ylim(ax,latitude+[-span,span]);
ax.XAxis.Exponent=0; ax.YAxis.Exponent=0;
xlabel(ax,'Longitude [deg]'); ylabel(ax,'Latitude [deg]'); title(ax,'Local detail (+/-0.05 deg)');
legend(ax,{'Task 1 origin'},'FontSize',8,'Location','northeast');
text(ax,0.02,0.02,sprintf('lat %.6f deg   lon %.6f deg   alt %.2f m',latitude,longitude,result.altitude_m), ...
    'Units','normalized','VerticalAlignment','bottom','FontSize',8,'BackgroundColor','w','Margin',2);
figCtx=fusion.figures('save',figCtx,1,'reference_origin_map',outDir,f);
end

function figCtx=reference_vector_figure(figCtx,result,outDir)
labels={'North','East','Down'};
f=new_figure(1000,420); tl=tiledlayout(f,1,2);
ax=nexttile(tl); bar(ax,result.specific_force_reference_ned_mps2(:),'FaceColor',[0.466,0.674,0.188]);
grid(ax,'on'); xticks(ax,1:3); xticklabels(ax,labels); ylabel(ax,'Specific force [m/s^2]');
title(ax,'Gravity reference in NED');
ax=nexttile(tl); bar(ax,result.earth_rate_reference_ned_rps(:)*1e6,'FaceColor',[0.850,0.325,0.098]);
grid(ax,'on'); xticks(ax,1:3); xticklabels(ax,labels); ylabel(ax,'Earth rate [urad/s]');
title(ax,'Earth-rotation reference in NED');
figCtx=fusion.figures('save',figCtx,1,'gravity_earth_rate_vectors',outDir,f);
end

function f=new_figure(width,height)
f=figure('Visible','off'); f.Position(3:4)=[width,height];
end
