function result = task1(data, cfg, outDir)
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
    f=figure('Visible','off'); scatter(rad2deg(lon),rad2deg(lat),50,'r','filled'); grid on; xlim([-180,180]); ylim([-90,90]);
    xlabel('Longitude [deg]'); ylabel('Latitude [deg]'); title('Task 1.2 - Reference location');
    exportgraphics(f,fullfile(outDir,'reference_location.png'),'Resolution',160); close(f);
end
end

function n=truth_rows(truth)
if isempty(truth); n=0; else; n=truth.rows; end
end
