function sstr = struct2single(dstr)
%Convert all double fields to single
names = fields(dstr);
data = struct2cell(dstr);
for i = 1:length(data)
    if(isa(data{i},'double'))
        data{i} = single(data{i});
    end
end
sstr = cell2struct(data,names);