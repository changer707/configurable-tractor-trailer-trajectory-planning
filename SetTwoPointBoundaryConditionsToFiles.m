function SetTwoPointBoundaryConditionsToFiles(vector,NC,NE,box_l,box_w,box_vertex)
% Input:
% --- vector: two-point boundary_conditions
% Output:
% --- Initial_config
% --- Terminal_config
% This function is used to set the boundary conditions for NLP solution in
% AMPL.

warning off

 X1 = vector(1 : NC+6);
 X2 = vector(NC+7 : NC+12);
 box = [box_l,box_w];
 
delete('Initial_config');
fid = fopen('Initial_config', 'w');
for ii = 1 : length(X1)
    fprintf(fid,'%g  %f \r\n', ii, X1(ii));
end
fclose(fid);

delete('Terminal_config');
fid = fopen('Terminal_config', 'w');
for ii = 1 : length(X2)
    fprintf(fid,'%g  %f \r\n', ii, X2(ii));
end
fclose(fid);

delete('NC_config');
fid = fopen('NC_config', 'w');
fprintf(fid,'%d', NC);
fclose(fid);

delete('NE_config');
fid = fopen('NE_config', 'w');
fprintf(fid,'%d', NE);
fclose(fid);

delete('Box_config');
fid = fopen('Box_config', 'w');
for ii = 1 : length(box)
    fprintf(fid,'%g  %f \r\n', ii, box(ii));
end
fclose(fid);

delete('Boxvertex_config');
fid = fopen('Boxvertex_config', 'w');
for ii = 1 : length(box_vertex)
    fprintf(fid,'%g  %f \r\n', ii, box_vertex(ii));
end
fclose(fid);