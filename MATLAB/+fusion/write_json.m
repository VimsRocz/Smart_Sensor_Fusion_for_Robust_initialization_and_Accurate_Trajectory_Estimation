function write_json(path, payload)
%WRITE_JSON Atomically write a pretty JSON artifact.

folder = fileparts(path);
if ~exist(folder, 'dir'); mkdir(folder); end
temporary = [path '.tmp'];
text = jsonencode(payload, 'PrettyPrint', true);
fid = fopen(temporary, 'w');
if fid < 0; error('fusion:Output', 'Cannot write %s.', temporary); end
cleaner = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid, '%s\n', text);
clear cleaner
movefile(temporary, path, 'f');
end
