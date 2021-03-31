function [box_vertex] = GetBoxVertex(x0,y0,W,L,alfa)
box_vertex = zeros(8,1);   %点的顺序：右上——左上（顺时针）
R = sqrt((W/2)^2+(L/2)^2); %四点共圆半径
delta_b = atan(W/L);
box_vertex(1) = x0 + R*cos(alfa+delta_b);  
box_vertex(2) = y0 + R*sin(alfa+delta_b);  
box_vertex(3) = x0 + R*cos(alfa-delta_b);  
box_vertex(4) = y0 + R*sin(alfa-delta_b);
box_vertex(5) = x0 + R*cos(alfa+delta_b-pi);
box_vertex(6) = y0 + R*sin(alfa+delta_b-pi);
box_vertex(7) = x0 + R*cos(alfa-delta_b+pi);
box_vertex(8) = y0 + R*sin(alfa-delta_b+pi);

end