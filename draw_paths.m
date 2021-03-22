function draw_paths(NCNC,box_vertex,offaxle)
% Plot the obtained optimal trajectories for the tractor/trailers.

colorpool = [237,28,36; 0,162,232; 34,177,76; 255,127,39]./255;

% Load the optimization results.
load phy.txt
load x.txt
load y.txt
load t.txt
load tf.txt;
t = tf(1,1);

load AX.txt
load BX.txt
load CX.txt
load DX.txt

load AY.txt
load BY.txt
load CY.txt
load DY.txt

% NCNC = 2; % Specify the number of tractor + trailers at a gross.

phy = reshape(phy,length(phy),1); % Get the steering angle profile
x = reshape(x,length(x)./NCNC,NCNC); % Get the x coordination profile for each of the NCNC parts of the tractor-trailer vehicle
y = reshape(y,length(y)./NCNC,NCNC); % Get the y coordination profile

number_of_frame = 100; % Determin how dense the trajectories are to be plotted.

% load NE.txt
% NE = NE(1,1);
NE = 20;
phy = smoother(phy,number_of_frame,NE);
x = smoother(x,number_of_frame,NE);
y = smoother(y,number_of_frame,NE);

oot = AX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); AX = oot;
oot = BX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); BX = oot;
oot = CX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); CX = oot;
oot = DX; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); DX = oot;

oot = AY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); AY = oot;
oot = BY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); BY = oot;
oot = CY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); CY = oot;
oot = DY; oot = reshape(oot,length(oot)./NCNC,NCNC); oot = smoother(oot,number_of_frame,NE); DY = oot;

hold on
axis equal
grid on
box on
%  axis([-15,20,-20,10]);
xlabel('x axis / m');
ylabel('y axis / m');
title('Optimized Trajectories of the Tractor-Trailer Vehicle');
%% ¹ì¼£
set(0,'DefaultLineLineWidth',2);
for ii = 1:NCNC
    xt = x(:,ii);
    yt = y(:,ii);
    plot(xt,yt,'Color',colorpool(ii,:));
end
%% ÕÏ°­
load Current_vertex;
for ii = 1 :(length(Current_vertex)/8)
    APP = [Current_vertex((ii-1)*8+1,4),Current_vertex((ii-1)*8+2,4)];
    CPP = [Current_vertex((ii-1)*8+5,4),Current_vertex((ii-1)*8+6,4)];
    BPP = [Current_vertex((ii-1)*8+3,4),Current_vertex((ii-1)*8+4,4)];
    DPP = [Current_vertex((ii-1)*8+7,4),Current_vertex((ii-1)*8+8,4)];
    PPX = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
    fill(PPX(1,:),PPX(2,:),[0.5,0.5,0.5]);
end
%% ¿âÎ»
APP = box_vertex(1:2);%[-1,5.7];
CPP = box_vertex(5:6);%[15,8.3];
BPP = box_vertex(3:4);%[15,5.7];
DPP = box_vertex(7:8);%[-1,8.3];
set(0,'DefaultLineLineWidth',1.5);
PPX = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
plot(PPX(1,:),PPX(2,:),'k--');
%% ³µÌå
set(0,'DefaultLineLineWidth',1);
% xpp=zeros(NCNC,5);ypp=zeros(NCNC,5);
% for jjx = round(linspace(1,1)) 
%     for ii = 1:NCNC
%         APP = [AX(jjx,ii), AY(jjx,ii)];
%         CPP = [CX(jjx,ii), CY(jjx,ii)];
%         BPP = [BX(jjx,ii), BY(jjx,ii)];
%         DPP = [DX(jjx,ii), DY(jjx,ii)];
%         PP_ = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
%         xpp(ii,:)=PP_(1,:);ypp(ii,:)=PP_(2,:);
%         h(ii)=plot(xpp(ii,:),ypp(ii,:),'Color',colorpool(ii,:));  
%         set(h(ii),'YDataSource','xpp(ii)')
%         set(h(ii),'YDataSource','ypp(ii)')
%     end
% end
 theAxes = axis;
fmat = moviein(200);
for jjx = round(linspace(1,number_of_frame)) 
    for ii = 1:NCNC
        APP = [AX(jjx,ii), AY(jjx,ii)];
        CPP = [CX(jjx,ii), CY(jjx,ii)];
        BPP = [BX(jjx,ii), BY(jjx,ii)];
        DPP = [DX(jjx,ii), DY(jjx,ii)];
        PP_ = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
        h(ii)= plot(PP_(1,:),PP_(2,:),'Color',colorpool(ii,:));  
        if ii>1
            if ~offaxle
                x_zhou=[x(jjx,ii-1),x(jjx,ii)];
                y_zhou=[y(jjx,ii-1),y(jjx,ii)];
            else
%                 x_zhou=0.5*[(APP(1)+BPP(1)),CPP(1)+DPP(1)];
%                 y_zhou=0.5*[(APP(2)+BPP(2)),CPP(2)+DPP(2)];
                x_zhou=[0.5*(CX(jjx,ii-1)+DX(jjx,ii-1)),x(jjx,ii)];
                y_zhou=[0.5*(CY(jjx,ii-1)+DY(jjx,ii-1)),y(jjx,ii)];
            end
            m(ii)=plot(x_zhou,y_zhou,'k','LineWidth',2);
        end
        axis(theAxes)
        fmat(:,jjx)=getframe;
    end

     delete(h)
     if NCNC>1
       delete(m)
     end

end
movie(fmat,5);




% for ii = 1:NCNC
%     for jjx = round(linspace(1,number_of_frame))
%         APP = [AX(jjx,ii), AY(jjx,ii)];
%         CPP = [CX(jjx,ii), CY(jjx,ii)];
%         BPP = [BX(jjx,ii), BY(jjx,ii)];
%         DPP = [DX(jjx,ii), DY(jjx,ii)];
%         PP_ = [APP(1),BPP(1),CPP(1),DPP(1),APP(1);APP(2),BPP(2),CPP(2),DPP(2),APP(2)];
%         plot(PP_(1,:),PP_(2,:),'Color',colorpool(ii,:));
%     end
% end
