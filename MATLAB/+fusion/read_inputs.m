function data = read_inputs(imuPath, gnssPath, truthPath, cfg)
%READ_INPUTS Validate and normalize the documented IMU/GNSS/truth contracts.

must_file(imuPath, 'IMU'); must_file(gnssPath, 'GNSS');
raw = readmatrix(imuPath, 'FileType', 'text');
raw = raw(~all(isnan(raw),2),:);
if size(raw,1) < 3 || size(raw,2) < 8 || any(~isfinite(raw(:,1:8)), 'all')
    error('fusion:Input', 'IMU must contain >=3 finite rows and >=8 numeric columns.');
end
rawTime = raw(:,2); positive = diff(rawTime); positive = positive(positive > 0);
if isempty(positive); error('fusion:Input', 'IMU time has no positive increments.'); end
dt = median(positive); time = zeros(size(rawTime)); wraps = 0;
for i=2:numel(rawTime)
    step = rawTime(i)-rawTime(i-1);
    if step < -max(.25,20*dt); wraps=wraps+1; step=dt; end
    if step <= 0; error('fusion:Input', 'Unsupported IMU duplicate/reversal at row %d.',i); end
    time(i)=time(i-1)+step;
end
gyro=raw(:,3:5); accel=raw(:,6:8);
if strcmpi(cfg.imu_measurement_type,'delta')
    rowDt=[dt;diff(time)]; gyro=gyro./rowDt; accel=accel./rowDt;
end
data.imu = struct('path',char(imuPath),'time_s',time,'gyro_rps',gyro, ...
    'accel_mps2',accel,'dt_s',dt,'rows',size(raw,1),'columns',size(raw,2), ...
    'clock_wraps',wraps);

tableData = readtable(gnssPath, 'VariableNamingRule', 'preserve');
aliases = struct( ...
    'time',{{'Posix_Time','POSIX_Time','time','Time','timestamp'}}, ...
    'x',{{'X_ECEF_m','X_ECEF','ecef_x_m','x'}}, ...
    'y',{{'Y_ECEF_m','Y_ECEF','ecef_y_m','y'}}, ...
    'z',{{'Z_ECEF_m','Z_ECEF','ecef_z_m','z'}}, ...
    'vx',{{'VX_ECEF_mps','VX_ECEF','ecef_vx_mps','vx'}}, ...
    'vy',{{'VY_ECEF_mps','VY_ECEF','ecef_vy_mps','vy'}}, ...
    'vz',{{'VZ_ECEF_mps','VZ_ECEF','ecef_vz_mps','vz'}});
keys={'time','x','y','z','vx','vy','vz'}; matrix=zeros(height(tableData),7); mapping=struct();
names=tableData.Properties.VariableNames;
for k=1:numel(keys)
    candidates=aliases.(keys{k}); index=find(ismember(names,candidates),1);
    if isempty(index); error('fusion:Input','GNSS missing required %s column.',keys{k}); end
    matrix(:,k)=double(tableData.(names{index})); mapping.(keys{k})=names{index};
end
if size(matrix,1)<2 || any(~isfinite(matrix),'all'); error('fusion:Input','GNSS requires >=2 finite rows.'); end
gtime=matrix(:,1)-matrix(1,1);
if any(diff(gtime)<=0); error('fusion:Input','GNSS timestamps must be strictly increasing.'); end
position=matrix(:,2:4); velocity=matrix(:,5:7);
if any(vecnorm(position,2,2)<6e6); error('fusion:Input','GNSS ECEF norms must exceed 6,000 km.'); end
data.gnss=struct('path',char(gnssPath),'time_s',gtime,'position_ecef_m',position, ...
    'velocity_ecef_mps',velocity,'rows',size(matrix,1),'column_map',mapping);

data.truth=[];
if nargin>=3 && strlength(string(truthPath))>0
    must_file(truthPath,'Truth'); truth=readmatrix(truthPath,'FileType','text');
    truth=truth(~all(isnan(truth),2),:);
    if size(truth,1)<2 || size(truth,2)<8 || any(~isfinite(truth(:,1:min(12,end))),'all')
        error('fusion:Input','Truth requires >=2 finite rows and >=8 columns.');
    end
    ttime=truth(:,2)-truth(1,2); if any(diff(ttime)<=0); error('fusion:Input','Truth time must increase.'); end
    quat=[]; if size(truth,2)>=12
        quat=truth(:,9:12);
        if strcmpi(cfg.truth_quaternion_order,'xyzw'); quat=quat(:,[4,1,2,3]); end
        quat=quat./vecnorm(quat,2,2);
    end
    data.truth=struct('path',char(truthPath),'time_s',ttime,'position_ecef_m',truth(:,3:5), ...
        'velocity_ecef_mps',truth(:,6:8),'quaternion_wxyz',quat, ...
        'source_quaternion_order',cfg.truth_quaternion_order,'rows',size(truth,1));
end
end

function must_file(path,label)
if ~isfile(path); error('fusion:Input','%s file does not exist: %s',label,path); end
info=dir(path); if info.bytes==0; error('fusion:Input','%s file is empty: %s',label,path); end
end
