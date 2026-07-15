function comparison = run_all_methods(varargin)
%RUN_ALL_METHODS Run TRIAD, Davenport and SVD and compare Task 7 metrics.

methods={'TRIAD','Davenport','SVD'}; results=struct();
for i=1:numel(methods)
    results.(methods{i})=run_pipeline(varargin{:},'method',methods{i});
end
root=fileparts(results.TRIAD.run_dir); comparisonDir=fullfile(root,'comparison'); if ~exist(comparisonDir,'dir'); mkdir(comparisonDir); end
metrics=struct();
for i=1:numel(methods)
    taskList=results.(methods{i}).tasks;
    if numel(taskList)>=7; metrics.(methods{i})=taskList{7}.metrics; else; metrics.(methods{i})=struct(); end
end
comparison=struct('schema_version','2.0','methods',{methods},'metrics',metrics, ...
    'note','Task 1-2 inputs are common; Task 3 attitude differs; Tasks 4-7 consume that method-specific attitude.');
fusion.write_json(fullfile(comparisonDir,'method_comparison.json'),comparison);

positionRMSE=nan(3,1); velocityRMSE=nan(3,1); attitudeRMSE=nan(3,1); heightRMSE=nan(3,1);
for i=1:3
    m=metrics.(methods{i});
    if isfield(m,'position_rmse_m'); positionRMSE(i)=m.position_rmse_m; end
    if isfield(m,'velocity_rmse_mps'); velocityRMSE(i)=m.velocity_rmse_mps; end
    if isfield(m,'attitude_rmse_deg'); attitudeRMSE(i)=m.attitude_rmse_deg; end
    if isfield(m,'height_rmse_m'); heightRMSE(i)=m.height_rmse_m; end
end
tableOut=table(string(methods(:)),positionRMSE,velocityRMSE,heightRMSE,attitudeRMSE, ...
    'VariableNames',{'method','position_rmse_m','velocity_rmse_mps','height_rmse_m','attitude_rmse_deg'});
writetable(tableOut,fullfile(comparisonDir,'method_comparison.csv'));
fprintf('All-method comparison complete: %s\n',comparisonDir);
end
