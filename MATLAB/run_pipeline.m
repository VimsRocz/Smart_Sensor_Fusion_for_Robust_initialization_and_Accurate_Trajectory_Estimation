function result = run_pipeline(varargin)
%RUN_PIPELINE Canonical MATLAB entry point for Tasks 1-7.
%
% result = run_pipeline('imu', path, 'gnss', path, 'truth', path, ...
%   'method', 'TRIAD', 'tasks', '1-7', 'output', 'results', 'config', struct())

matlabRoot=fileparts(mfilename('fullpath')); repoRoot=fileparts(matlabRoot); addpath(matlabRoot);
p=inputParser; p.FunctionName='run_pipeline';
addParameter(p,'imu',fullfile(repoRoot,'DATA','IMU','IMU_X001_small.dat'));
addParameter(p,'gnss',fullfile(repoRoot,'DATA','GNSS','GNSS_X001_small.csv'));
addParameter(p,'truth',fullfile(repoRoot,'DATA','Truth','STATE_X001_small.txt'));
addParameter(p,'method','TRIAD'); addParameter(p,'tasks','1-7');
addParameter(p,'output',fullfile(repoRoot,'results','matlab')); addParameter(p,'run_id',''); addParameter(p,'config',struct());
parse(p,varargin{:}); args=p.Results; cfg=fusion.default_config(args.config); [requested,expanded]=fusion.parse_tasks(args.tasks);
method=char(string(args.method)); valid={'TRIAD','Davenport','SVD'}; index=find(strcmpi(method,valid),1);
if isempty(index); error('fusion:Method','Method must be TRIAD, Davenport or SVD.'); end; method=valid{index};
data=fusion.read_inputs(args.imu,args.gnss,args.truth,cfg);
if strlength(string(args.run_id))==0
    [~,imuStem]=fileparts(args.imu); [~,gnssStem]=fileparts(args.gnss); runId=[imuStem '__' gnssStem];
else; runId=regexprep(char(args.run_id),'[^A-Za-z0-9_.-]','_'); end
runDir=fullfile(args.output,runId,lower(method)); if ~exist(runDir,'dir'); mkdir(runDir); end
started=datetime('now','TimeZone','UTC'); manifest=struct('schema_version','2.0','status','running', ...
    'run_id',runId,'method',method,'requested_tasks',requested,'executed_tasks',expanded, ...
    'auto_dependencies',setdiff(expanded,requested),'started_utc',char(started), ...
    'inputs',struct('imu',char(args.imu),'gnss',char(args.gnss),'truth',char(args.truth)),'config',cfg);
fusion.write_json(fullfile(runDir,'manifest.json'),manifest); tasks=cell(1,max(expanded));
tasks{1}=fusion.task1(data,cfg,make_task_dir(runDir,1,'inputs_reference'));
if max(expanded)>=2; tasks{2}=fusion.task2(data.imu,tasks{1},cfg,make_task_dir(runDir,2,'static_imu')); end
if max(expanded)>=3; tasks{3}=fusion.task3(method,tasks{1},tasks{2},cfg,make_task_dir(runDir,3,'attitude')); end
if max(expanded)>=4; tasks{4}=fusion.task4(data.imu,data.gnss,tasks{1},tasks{3},cfg,make_task_dir(runDir,4,'inertial')); end
if max(expanded)>=5; tasks{5}=fusion.task5(data.imu,data.gnss,tasks{1},tasks{4},cfg,make_task_dir(runDir,5,'fusion')); end
if max(expanded)>=6; tasks{6}=fusion.task6(data.truth,tasks{1},tasks{5},cfg,make_task_dir(runDir,6,'truth')); end
if max(expanded)>=7; tasks{7}=fusion.task7(tasks{5},tasks{6},cfg,make_task_dir(runDir,7,'evaluation')); end
finished=datetime('now','TimeZone','UTC'); manifest.status='complete'; manifest.finished_utc=char(finished);
manifest.elapsed_s=seconds(finished-started); if numel(tasks)>=7; manifest.metrics=tasks{7}.metrics; end
fusion.write_json(fullfile(runDir,'manifest.json'),manifest);
result=struct('run_dir',runDir,'manifest',manifest,'tasks',{tasks});
fprintf('%s complete: %s\n',method,runDir);
end

function path=make_task_dir(runDir,number,slug)
path=fullfile(runDir,sprintf('task_%02d_%s',number,slug)); if ~exist(path,'dir'); mkdir(path); end
end
