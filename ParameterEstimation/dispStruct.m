function dispStruct(s, indent)
    if nargin < 2, indent = ''; end
    if isstruct(s) || isobject(s)
        fn = fieldnames(s);
        for i = 1:numel(fn)
            val = s.(fn{i});
            fprintf('%s%s:\n', indent, fn{i});
            dispStruct(val, [indent '  ']);
        end
    elseif iscell(s)
        for i = 1:numel(s)
            fprintf('%s{%d}:\n', indent, i);
            dispStruct(s{i}, [indent '  ']);
        end
    else
        sz = size(s);
        if numel(s) == 1
            fprintf('%s%s\n', indent, mat2str(s));
        else
            fprintf('%s[%s %s]\n', indent, num2str(sz), class(s));
        end
    end
end
