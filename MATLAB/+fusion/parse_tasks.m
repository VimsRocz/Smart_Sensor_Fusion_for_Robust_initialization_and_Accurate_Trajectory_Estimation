function [requested, expanded] = parse_tasks(specification)
%PARSE_TASKS Parse "3", "1-5" or "1,3,5" and include dependencies.

if isnumeric(specification)
    requested = unique(specification(:)');
else
    requested = [];
    items = split(erase(string(specification), ' '), ',');
    for i = 1:numel(items)
        item = items(i);
        if contains(item, '-')
            bounds = str2double(split(item, '-'));
            requested = [requested, min(bounds):max(bounds)]; %#ok<AGROW>
        elseif strlength(item) > 0
            requested(end + 1) = str2double(item); %#ok<AGROW>
        end
    end
    requested = unique(requested);
end
if isempty(requested) || any(~isfinite(requested)) || min(requested) < 1 || max(requested) > 7
    error('fusion:Tasks', 'Tasks must select one or more integers from 1 through 7.');
end
expanded = 1:max(requested);
end
