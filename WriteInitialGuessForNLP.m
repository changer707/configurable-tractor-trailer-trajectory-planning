function WriteInitialGuessForNLP(NE,NC,Bazierpath,shape_param,theta0_all)
global dynamic_constraints
phy_max = dynamic_constraints(1);  v_max = dynamic_constraints(3);
Lw = shape_param(5); TN = shape_param(1); TL = shape_param(2); TM = shape_param(3); TB = shape_param(4); TT1 = shape_param(6);  TT2 = shape_param(7);
x = Bazierpath(:,1); y = Bazierpath(:,2);
%% tractor
%差分，得航向角
theta = zeros(NE,1); v1 = zeros(NE,1); dt = zeros(NE,1); phy = zeros(NE,1); a = zeros(NE,1); w =zeros(NE,1);
theta(1) = theta0_all;
for i = 2:length(theta)-1
    theta_path = atan((Bazierpath(i+1,2)-Bazierpath(i,2))./(Bazierpath(i+1,1)-Bazierpath(i,1)));%轨迹的航向角而非车辆的航向角（只有正走时满足）
    if abs(theta_path-theta(i-1))>0.5*pi
        theta(i) = theta_path - pi;
    else
        theta(i) = theta_path;
    end
end
theta(NE) = theta(NE-1);
for ii = 1 : (NE-1)
    % Specify v(t)
    addtion = (x(ii+1) - x(ii)) * cos(theta(ii)) + (y(ii+1) - y(ii)) * sin(theta(ii));
    if (addtion > 0)
        v1(ii) = v_max;
    else
        v1(ii) = -v_max;
    end
    % dt
    ds = hypot(x(ii+1) - x(ii), y(ii+1) - y(ii));
    dt(ii) = ds / abs(v1(ii));
    % a(t)
    a(ii) = (v1(ii+1)-v1(ii))./dt(ii);
    % phy(t)
    dtheta = (theta(ii+1) - theta(ii)) / dt(ii);
    phy(ii) = atan(dtheta * shape_param(2) / v1(ii));
    if (phy(ii) > phy_max)
        phy(ii) = phy_max;
    elseif (phy(ii) < -phy_max)
        phy(ii) = -phy_max;
    end
    % w(t)
    w(ii) = (phy(ii+1)-phy(ii))./dt(ii);
end
%% trailers
theta_2 = zeros(NE,1); theta_2(1) = theta0_all;
theta_3 = zeros(NE,1); theta_3(1) = theta0_all;
theta_4 = zeros(NE,1); theta_4(1) = theta0_all;
for i = 2:NE
    theta_2(i) = theta_2(i-1)+dt(i-1)*v1(i-1)*sin( theta(i-1)-theta_2(i-1) )./Lw; %theta for first trailer
    theta_3(i) = theta_3(i-1)+dt(i-1)*v1(i-1)*sin( theta_2(i-1)-theta_3(i-1) )*cos( theta(i-1)-theta_2(i-1) )./Lw;
    theta_4(i) = theta_4(i-1)+dt(i-1)*v1(i-1)*sin( theta_3(i-1)-theta_4(i-1) )*cos( theta(i-1)-theta_2(i-1) )*cos( theta_2(i-1)-theta_3(i-1) )./Lw;
end
x_2 = zeros(NE,1);y_2 = zeros(NE,1);x_3 = zeros(NE,1);y_3 = zeros(NE,1);x_4 = zeros(NE,1);y_4 = zeros(NE,1);
AX = zeros(NE,1);AY = zeros(NE,1);BX=zeros(NE,1);BY=zeros(NE,1);CX=zeros(NE,1);CY=zeros(NE,1);DX=zeros(NE,1);DY=zeros(NE,1);
AX_2 = zeros(NE,1);AY_2 = zeros(NE,1);BX_2=zeros(NE,1);BY_2=zeros(NE,1);CX_2=zeros(NE,1);CY_2=zeros(NE,1);DX_2=zeros(NE,1);DY_2=zeros(NE,1);
AX_3 = zeros(NE,1);AY_3 = zeros(NE,1);BX_3=zeros(NE,1);BY_3=zeros(NE,1);CX_3=zeros(NE,1);CY_3=zeros(NE,1);DX_3=zeros(NE,1);DY_3=zeros(NE,1);
AX_4 = zeros(NE,1);AY_4 = zeros(NE,1);BX_4=zeros(NE,1);BY_4=zeros(NE,1);CX_4=zeros(NE,1);CY_4=zeros(NE,1);DX_4=zeros(NE,1);DY_4=zeros(NE,1);
for i =1:NE
% trailer (x,y)
    x_2(i) = x(i) - Lw*cos(theta_2(i));    y_2(i) = y(i) - Lw*sin(theta_2(i));
    x_3(i) = x(i) - Lw*( cos(theta_2(i))+cos(theta_3(i)) );   y_3(i) = y(i) - Lw*( sin(theta_2(i))+sin(theta_3(i)) );
    x_4(i) = x(i) - Lw*( cos(theta_2(i))+cos(theta_3(i))+cos(theta_4(i)) );   y_4(i) = y(i) - Lw*( sin(theta_2(i))+sin(theta_3(i))+sin(theta_4(i)) );
% tractor ABCD
    AX(i) = x(i) + (TL+TN)*cos( theta(i) ) - TB*sin( theta(i) );   AY(i) = y(i) + (TL+TN)*sin( theta(i) ) + TB*cos( theta(i) );
    BX(i) = x(i) + (TL+TN)*cos( theta(i) ) + TB*sin( theta(i) );   BY(i) = y(i) + (TL+TN)*cos( theta(i) ) - TB*sin( theta(i) );
    CX(i) = x(i) - TM*cos( theta(i) ) + TB*sin( theta(i) );        CY(i) = y(i) - TM*cos( theta(i) ) - TB*sin( theta(i) );
    DX(i) = x(i) - TM*cos( theta(i) ) - TB*sin( theta(i) );        DY(i) = y(i) - TM*cos( theta(i) ) + TB*sin( theta(i) );
% trailer ABCD
    AX_2(i) = x_2(i) + TT1*cos( theta_2(i) ) - TB*sin( theta_2(i) );   AY_2(i) = y_2(i) + TT1*sin( theta_2(i) ) + TB*cos( theta_2(i) );
    BX_2(i) = x_2(i) + TT1*cos( theta_2(i) ) + TB*sin( theta_2(i) );   BY_2(i) = y_2(i) + TT1*cos( theta_2(i) ) - TB*sin( theta_2(i) );
    CX_2(i) = x_2(i) - TT2*cos( theta_2(i) ) + TB*sin( theta_2(i) );   CY_2(i) = y_2(i) - TT2*cos( theta_2(i) ) - TB*sin( theta_2(i) );
    DX_2(i) = x_2(i) - TT2*cos( theta_2(i) ) - TB*sin( theta_2(i) );   DY_2(i) = y_2(i) - TT2*cos( theta_2(i) ) + TB*sin( theta_2(i) );
    
    AX_3(i) = x_3(i) + TT1*cos( theta_3(i) ) - TB*sin( theta_3(i) );   AY_3(i) = y_3(i) + TT1*sin( theta_3(i) ) + TB*cos( theta_3(i) );
    BX_3(i) = x_3(i) + TT1*cos( theta_3(i) ) + TB*sin( theta_3(i) );   BY_3(i) = y_3(i) + TT1*cos( theta_3(i) ) - TB*sin( theta_3(i) );
    CX_3(i) = x_3(i) - TT2*cos( theta_3(i) ) + TB*sin( theta_3(i) );   CY_3(i) = y_3(i) - TT2*cos( theta_3(i) ) - TB*sin( theta_3(i) );
    DX_3(i) = x_3(i) - TT2*cos( theta_3(i) ) - TB*sin( theta_3(i) );   DY_3(i) = y_3(i) - TT2*cos( theta_3(i) ) + TB*sin( theta_3(i) );
    
    AX_4(i) = x_4(i) + TT1*cos( theta_4(i) ) - TB*sin( theta_4(i) );   AY_4(i) = y_4(i) + TT1*sin( theta_4(i) ) + TB*cos( theta_4(i) );
    BX_4(i) = x_4(i) + TT1*cos( theta_4(i) ) + TB*sin( theta_4(i) );   BY_4(i) = y_4(i) + TT1*cos( theta_4(i) ) - TB*sin( theta_4(i) );
    CX_4(i) = x_4(i) - TT2*cos( theta_4(i) ) + TB*sin( theta_4(i) );   CY_4(i) = y_4(i) - TT2*cos( theta_4(i) ) - TB*sin( theta_4(i) );
    DX_4(i) = x_4(i) - TT2*cos( theta_4(i) ) - TB*sin( theta_4(i) );   DY_4(i) = y_4(i) - TT2*cos( theta_4(i) ) + TB*sin( theta_4(i) );    
end
x_all = {x,x_2,x_3,x_4}; y_all = {y,y_2,y_3,y_4}; theta_all = {theta,theta_2,theta_3,theta_4};
AX_all = {AX,AX_2,AX_3,AX_4}; BX_all = {BX,BX_2,BX_3,BX_4};  CX_all = {CX,CX_2,CX_3,CX_4}; DX_all = {DX,DX_2,DX_3,DX_4};
AY_all = {AY,AY_2,AY_3,AY_4}; BY_all = {BY,BY_2,BY_3,BY_4};  CY_all = {CY,CY_2,CY_3,CY_4}; DY_all = {DY,DY_2,DY_3,DY_4};
%% 写入initial_guess.INIVAL
delete('initial_guess.INIVAL');
fid = fopen('initial_guess.INIVAL', 'w');
for ii = 1 : NC
    for jj = 1 : NE
        fprintf(fid, 'let x[%g,%g] := %f;\r\n', jj, ii, x_all{ii}(jj));
        fprintf(fid, 'let y[%g,%g] := %f;\r\n', jj, ii, y_all{ii}(jj));
        fprintf(fid, 'let theta[%g,%g] := %f;\r\n', jj, ii, theta_all{ii}(jj));
        fprintf(fid, 'let AX[%g,%g] := %f;\r\n', jj, ii, AX_all{ii}(jj));
        fprintf(fid, 'let BX[%g,%g] := %f;\r\n', jj, ii, BX_all{ii}(jj));
        fprintf(fid, 'let CX[%g,%g] := %f;\r\n', jj, ii, CX_all{ii}(jj));
        fprintf(fid, 'let DX[%g,%g] := %f;\r\n', jj, ii, DX_all{ii}(jj));
        fprintf(fid, 'let AY[%g,%g] := %f;\r\n', jj, ii, AY_all{ii}(jj));
        fprintf(fid, 'let BY[%g,%g] := %f;\r\n', jj, ii, BY_all{ii}(jj));
        fprintf(fid, 'let CY[%g,%g] := %f;\r\n', jj, ii, CY_all{ii}(jj));
        fprintf(fid, 'let DY[%g,%g] := %f;\r\n', jj, ii, DY_all{ii}(jj));
    end
end
for ii = 1:NE
    fprintf(fid, 'let v[%g] := %f;\r\n', ii, v1(ii));
    fprintf(fid, 'let phy[%g] := %f;\r\n', ii, phy(ii));
    fprintf(fid, 'let a[%g] := %f;\r\n', ii, a(ii));
    fprintf(fid, 'let w[%g] := %f;\r\n', ii, w(ii));
end
fprintf(fid, 'let tf:= %f;\r\n', sum(dt));
fclose(fid);
disp(['Astar 初始解写入完成！'])
end