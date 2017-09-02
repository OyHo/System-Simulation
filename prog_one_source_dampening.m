clear
clc

x = 0; % origo pos
y = 0; % origo pos
r=36.6; % roundabout radius
c=1; %counter

for pos = 0:pi/50:2*pi
    xunit(c)= x + r*cos(pos);
    yunit(c)= y + r*sin(pos);
    c=c+1;
    %plot(xunit,yunit)
    hold on
    %drawnow
end

figure('Position',get(0,'ScreenSize'))
axis([-40 40 -40 40]); axis square, axis off
axis equal
hold on

line(xunit,yunit,'LineWidth',30,'Color',[.4 .4 .4]);
line(xunit,yunit,'LineWidth',25,'Color',[1 1 1]);
line(xunit,yunit,'LineWidth',23,'Color',[.4 .4 .4]);
%line(xunit,yunit,'LineWidth',1,'LineStyle','--','Color',[1 1 0]);

l=2*pi*r;
N=22; %Number of cars
v0=8.3; % initial speed 30 km'h
%space=4; %Space between cars
%carlength=3; %Car length
%totspace=l-N*(space+carlength);
carcolor1 = 'b';
carcolor2 = 'y';

%if totspace <0
%    disp('Error');
%end

%if totspace>=0
%    theta = [0:2*pi/N:2*pi-2*pi/N];
%end
theta = [0:2*pi/N:2*pi-2*pi/N];

%%                                                      KALIBRERING
k_verdi = 100;
C=ones(1,N)/k_verdi; % 0.007
I=ones(1,N)*1000;


m=ones(1,N);
V_NULL = ones(1,N+1);
dteta=2*pi/N;
%x=cos((i-1)*dteta);
%y=sin((i-1)*dteta);
a_const = -5; %constant
e = zeros(1,N);
aks = -2;
R = zeros(1,N);

n = 0;
flux = [0];
flux_string = 0;
time_control = zeros(1,1);
ID = 1; %car identification

for i=1:N
    xval(i)=r*cos(theta(i));
    yval(i)=r*sin(theta(i));
    q(i)=0; % eller dteta,  position
    s(i)=i*dteta; % car position
    p(i)=v0*I(i); %car f
    %p(N+1)=v0*I(i); % first car and last car are the same
    if i==1
         h(i) =  plot(xval(i),yval(i),'s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor',carcolor1,'MarkerSize',10);
    else
        h(i) =  plot(xval(i),yval(i),'s','LineWidth',2,'MarkerEdgeColor','r','MarkerFaceColor',carcolor2,'MarkerSize',10);
    end
    hold on;
end

%p(1)=v0; %first car speed
dt=0.01;
nq=q; % pos
np=p; % car speed
ne=ones(1,N); % source
%I(N+1) = C(1);
j=0;
time = 100;
breakdist = 3;
crashdist = 0;
dangerzone = 2;
xnextcarforward = zeros(1,N);
ynextcarforward = zeros(1,N);
x = zeros(1,N+1);
y_f = zeros(1,N+1);
y = zeros(1,N+1);
theta = 0:2*pi/N:2*pi-2*pi/(N+1);
%R = -0.1;

ht=text(0,0,'time [s] = 0.0'); %Counts time in figure
%hSpeed = text(-65,35,'Speed [km/t] = 0.0'); %Displays speed of cars in ascending order
hFlux = text(-65,33, 'Flux [cars/sec] = 0');%Displays flux of cars (take some time)

%% Må fikse at biler reduserer fart dersom de kommer innenfor breakdist til bil forran
%% Biler kan øke fart dersom de kommer innenfor breakdist til bil bak
for t=0:dt:time % sekund
    j=j+1;
    
    set(ht, 'String',['Time = ',int2str(t)]) %num2str
    set(hFlux, 'String',['Flux [cars/sec] = ',num2str(flux_string)]);
    
    titlestr=['    Velocities:  '];
    Tid=['Time:  '];
    starttime=tic;
    if t == 1
        %p(1:5:N) = 1.5; % kvar femte bil fucker opp
        p(2) = 0.1*p(2); % første bil punkterer/bremsar kraftig ned
    end
    for i=1:N
          if p(i)/I(i)>v0+1
         	  p(i)= v0*I(i);
          end
    end
    for i=2:N
         xprecar(1) = x(N) - x(1); % xdistanse til bil bak
         yprecar(1) = y(N) - y(1); % ydistanse til bil bak
         xprecar(i) = x(i) - x(i-1); % xdistanse til bil bak
         yprecar(i) = y(i) - y(i-1); % ydistanse til bil bak
         distback(1) = sqrt(xprecar(1).^2+yprecar(1).^2);
         distback(i) = sqrt(xprecar(i).^2+yprecar(i).^2);
    end
    for k = 1:N-1
        danger(k) = ((p(k+1)/I(k+1))^2 -((p(k)/I(k))^2))/(2*aks);
        danger(N) = ((p(1)/I(1))^2 -((p(N)/I(N))^2))/(2*aks);
    end
    for i=1:N % bilnummer
        s(i)=s(i)+dt/r*p(i)/I(i);
        %s(N+1)= s(1);
        xnextcarforward(i) = x(i) - x(i+1); % xdistanse til neste bil framover
        ynextcarforward(i) = y(i) - y(i+1); % ydistanse til neste bil framover
        xnextcarforward(N) = x(N) - x(1); % xdistanse til neste bil framover
        ynextcarforward(N) = y(N) - y(1); % ydistanse til neste bil framover
        distforward(i) = sqrt(xnextcarforward(i).^2+ynextcarforward(i).^2);
        if t > 0.5
            if distforward(1) < 1.5
                p(1) = 0;
            end
            if distforward(i) < 1.5
                p(i) = 0;
            end
        end
        
        %% Justering mtp. bilen forran //REGULERING?
        %if distforward < breakdist
             %s(i)=s(i) -dt*((p(i)/I(i))^2-v0^2)/(2*distforward);
             %s(i)=s(i) -dt*sqrt(2*distforward*1+p(i)/I(i)^2);
             %if distforward < dangerzone
                 %s(i)=s(i)-dt*(p(i)/I(i))*0.4;
             %end
            % for z = 1:dangerzone
             %    delta = z * 0.5;
              %   s(i)=s(i) -dt*p(i)*delta;
             %end
        %end
        %if distforward < breakdist
            % s(i)=s(i) -dt*p(i)*0.2;
           %  if distforward < dangerzone
          %       s(i)=s(i)-dt*p(i)*0.4;
         %    end
            % for z = 1:dangerzone
             %    delta = z * 0.5;
              %   s(i)=s(i) -dt*p(i)*delta;
             %end
        %end
        %% Justering mtp. bilen bak
        %if distback < breakdist
        %     s(i)=s(i) +dt*p(i)*0.2;
         %    if distback < dangerzone
         %        s(i)=s(i) +dt*p(i)*0.4;
          %   end
             %for z = 1:dangerzone
               %  delta = z * 0.5;
              %   s(i)=s(i) -dt*p(i)*delta;
             %end
       % end
        %% DEMPER KOBLES INN 
        if distforward(i) < danger(i)
            R(i) =1000;
        else
            R(i) = 0;
        end
        if distback(i) < 1.5
            fprintf('Car %d is crashing with car %d \n', i, i+1);
        end
        if p(i) < 0
            disp('Negative speed, car is reversing!');
            break
        end
        
        %% posisjon sendes/settes til plottet h()
        %s(i)=sum(q(1:i)); % position
        x(i)=r*cos(mod(s(i),2*pi));
        %x(N+1)= x(1);
        y_f(i) = y(i);
        y(i)=r*sin(mod(s(i),2*pi));
        %y(N+1) = y(1);
        set(h(i),'XData',x(i),'YData',y(i));
        titlestr=[titlestr ' ' int2str(p(i)/I(i))];
        a(i,j)=s(i)*r;
        
        %% FLUX
        if x(i) > 0 && y(i) > 0 && y_f(i) <= 0
            if n == 0
                n = 1;
                time_control(1,n) = t;
            else
                n = n + 1;
                time_control(1,n) = t;                
            end
            if n > N
                flux(1,n-N) = N/(time_control(1,n)-time_control(1,n-N));
                flux_string = flux(1,n-N);
            end
        end
        
        
    end
    %% TITTEL
    %titlestr=[Tid ' ' int2str(t) ' [s]' titlestr ' [km/h] ' ];
    title(titlestr);
    drawnow; 
   
    %% Euler 2 difflign.
    nq(1)=p(N)/I(N)-p(1)/I(1);
    np(1)=e(1)+q(1)/C(1)-q(2)/C(2)-(p(1)/I(1))*R(1);
    ne(1)=a_const*(p(1)/I(1)-v0);
    for i=2:N-1
        nq(i)=p(i-1)/I(i-1)-p(i)/I(i); %pos
        np(i)=e(i)+q(i)/C(i)-q(i+1)/C(i+1)-(p(i)/I(i))*R(i); %speed
        %np(i-1)=e(i-1)-q(i)/C(i);
        ne(i)=a_const*(p(i)/I(i)-v0);
        %ne(i-1)=a_const*(p(i-1)/I(i-1)-v0);
        %Re = ((p(i)/I(i))^2-(p(i-1)/I(i-1))^2)./(2*distforward(i));
    end
    nq(N)=p(N-1)/I(N-1)-p(N)/I(N);
    np(N)=q(N)/C(N)-q(1)/C(1)-(p(N)/I(N))*R(N);
    ne(N)=a_const*(p(N)/I(N)-v0);
    q=q+dt*nq;
    p=p+dt*np;
    e=e+dt*ne;
    %R =Re;
    %    pause
%
end
hold off
figure(2)
mesh(a')
hidden off
figure(3)
plot(a')
axis([0 10000 0 30])
title(['Time vs Distance  |  Number of cars =  ',num2str(N,2), ...
    '  | Speed limit = ', num2str(v0,2), '[m/s] | Total time = '...
    , int2str(time), ' [s] | dt = ', num2str(dt,3)])
xlabel('Iterations')
ylabel('Distance [m]')