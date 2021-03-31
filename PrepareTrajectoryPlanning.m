% This script is used to prepare for trajectory planning and do
% post-processings after the solutions are derived.

if (mod(length(polygon_obstacle_vertex), 8) ~= 0)
    error obstacle vertex size is set inappropriately
end

if (length(boundary_constraints) ~= NC+12)
    error two-point boundary conditions set wrongly
end

SetTwoPointBoundaryConditionsToFiles(boundary_constraints,NC,NE,box_l,box_w);