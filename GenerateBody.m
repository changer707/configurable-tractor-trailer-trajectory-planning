function GenerateBody(gamma,shape_param)
% Input: scalar called \gamma, retunr void, but a new file is written as
% the output. By default, the obstacles are all assumed to be quadrilateral.
% If other types of polygonal obstacles are to be added, the .mod file also
% needs to be modified accordingly. This is not difficult.
%
% This function is used to generate the new vertexes according to the
% current setting of \gamma. When \gamma = 1, the new vertexes are idential
% to the ones on the original polygon(s); when \gamma = 0, all the new vertexes
% gather at the geometric center of the original polygon; when 0< \gamma <1,
% the new polygon appears to be a shrinked version of the original one.

if ((gamma > 1) || (gamma < 0))
    if gamma>1
        gamma =1;
    else
        error 'Invalid \gamma';
    end
end

warning off



shape_change = gamma*shape_param;
delete('shape_config');
fid = fopen('shape_config', 'w');
for ii = 1 : length(shape_param)
    fprintf(fid,'%g  %f \r\n', ii, shape_change(ii));
end
fclose(fid);