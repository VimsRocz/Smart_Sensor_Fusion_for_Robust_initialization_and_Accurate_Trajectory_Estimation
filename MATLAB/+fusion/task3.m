function [result, figCtx] = task3(method, task1, task2, cfg, outDir, figCtx)
%TASK3 Solve initial attitude and estimate body-frame IMU biases.

result=fusion.solve_attitude(method,task2.body_vectors_unit,task2.reference_vectors_unit, ...
    [cfg.gravity_weight,cfg.earth_rate_weight]);
C=result.c_body_to_ned;
expectedAccel=(C'*task1.specific_force_reference_ned_mps2')';
expectedGyro=(C'*task1.earth_rate_reference_ned_rps')';
result.task=3; result.name='Initial attitude and IMU biases';
result.subtasks={'3.1 Solve Body-to-NED alignment','3.2 Normalize [w,x,y,z] quaternion','3.3 Estimate IMU biases'};
result.accel_bias_body_mps2=task2.mean_accel_body_mps2-expectedAccel;
result.gyro_bias_body_rps=task2.mean_gyro_body_rps-expectedGyro;
fusion.write_json(fullfile(outDir,'initial_attitude.json'),result); save(fullfile(outDir,'initial_attitude.mat'),'-struct','result');
if cfg.plots
    q=result.quaternion_wxyz_body_to_ned;

    f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
    imagesc(ax,C,[-1,1]); axis(ax,'image'); colormap(ax,diverging_map());
    for row=1:3
        for column=1:3
            text(ax,column,row,sprintf('%.4f',C(row,column)),'HorizontalAlignment','center', ...
                'VerticalAlignment','middle','FontSize',10);
        end
    end
    set(ax,'XTick',1:3,'XTickLabel',{'body x','body y','body z'}, ...
        'YTick',1:3,'YTickLabel',{'North','East','Down'});
    title(ax,['C body-to-NED (' result.method ')']);
    cb=colorbar(ax); cb.Label.String='Matrix element';
    figCtx=fusion.figures('save',figCtx,3,'rotation_matrix',outDir,f);

    f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
    residuals=[result.gravity_error_deg,result.earth_rate_error_deg];
    b=bar(ax,residuals); b.FaceColor='flat'; b.CData=[0.173,0.627,0.173;1.000,0.498,0.055];
    set(ax,'XTick',1:2,'XTickLabel',{'Gravity','Earth rate'}); grid(ax,'on');
    ylabel(ax,'Residual alignment error [deg]'); ylim(ax,[0,max(residuals)*1.35+1e-9]);
    for k=1:2
        text(ax,k,residuals(k),sprintf('%.4g',residuals(k)),'HorizontalAlignment','center', ...
            'VerticalAlignment','bottom','FontSize',9);
    end
    figCtx=fusion.figures('save',figCtx,3,'vector_alignment_error',outDir,f);

    f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
    bar(ax,q,'FaceColor',[0.122,0.467,0.706]); ylim(ax,[-1.15,1.15]); grid(ax,'on');
    set(ax,'XTick',1:4,'XTickLabel',{'qw','qx','qy','qz'}); ylabel(ax,'Component value');
    yline(ax,0,'k-');
    for k=1:4
        text(ax,k,q(k),sprintf('%.5f',q(k)),'HorizontalAlignment','center', ...
            'VerticalAlignment',above_or_below(q(k)),'FontSize',9);
    end
    text(ax,0.02,0.02,sprintf('norm(q) = %.12f',result.quaternion_norm),'Units','normalized', ...
        'HorizontalAlignment','left','VerticalAlignment','bottom','FontSize',8);
    figCtx=fusion.figures('save',figCtx,3,'initial_quaternion',outDir,f);

    f=figure('Visible','off'); tiledlayout(1,1); ax=nexttile;
    angles=quaternion_to_euler_zyx_deg(q);
    b=bar(ax,angles); b.FaceColor='flat';
    b.CData=[0.580,0.404,0.741;0.090,0.745,0.812;0.737,0.741,0.133];
    set(ax,'XTick',1:3,'XTickLabel',{'Yaw','Pitch','Roll'}); grid(ax,'on');
    ylabel(ax,'Angle [deg]'); yline(ax,0,'k-');
    pad=0.15*max(1,max(abs(angles)));
    ylim(ax,[min(0,min(angles))-pad,max(0,max(angles))+pad]);
    for k=1:3
        text(ax,k,angles(k),sprintf('%.4f',angles(k)),'HorizontalAlignment','center', ...
            'VerticalAlignment',above_or_below(angles(k)),'FontSize',9);
    end
    figCtx=fusion.figures('save',figCtx,3,'initial_euler_angles',outDir,f);

    f=figure('Visible','off'); tiledlayout(1,2);
    ax=nexttile; bar(ax,result.accel_bias_body_mps2,'FaceColor',[0.839,0.153,0.157]);
    set(ax,'XTick',1:3,'XTickLabel',{'x','y','z'}); grid(ax,'on'); yline(ax,0,'k-');
    xlabel(ax,'Body axis'); ylabel(ax,'Accelerometer bias [m/s^2]'); title(ax,'Accelerometer bias');
    ax=nexttile; bar(ax,result.gyro_bias_body_rps*1e6,'FaceColor',[0.122,0.467,0.706]);
    set(ax,'XTick',1:3,'XTickLabel',{'x','y','z'}); grid(ax,'on'); yline(ax,0,'k-');
    xlabel(ax,'Body axis'); ylabel(ax,'Gyroscope bias [urad/s]'); title(ax,'Gyroscope bias');
    figCtx=fusion.figures('save',figCtx,3,'imu_bias_estimates',outDir,f);
end
end

function angles=quaternion_to_euler_zyx_deg(q)
%QUATERNION_TO_EULER_ZYX_DEG Scalar-first unit quaternion to [yaw,pitch,roll].
q=q(:)'/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);
yaw=atan2d(2*(w*z+x*y),1-2*(y*y+z*z));
pitch=asind(max(-1,min(1,2*(w*y-z*x))));
roll=atan2d(2*(w*x+y*z),1-2*(x*x+y*y));
angles=[yaw,pitch,roll];
end

function alignment=above_or_below(value)
if value>=0; alignment='bottom'; else; alignment='top'; end
end

function map=diverging_map()
%DIVERGING_MAP Blue-grey-red map matching the Python "coolwarm" colouring.
anchors=[0.230,0.299,0.754;0.865,0.865,0.865;0.706,0.016,0.150];
map=interp1([-1;0;1],anchors,linspace(-1,1,256)');
end
