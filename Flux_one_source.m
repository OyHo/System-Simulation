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
end

% k_verdi = [1 10 30 60 100 140 180 240 300 400 500 650 750 850 950 1000 1150 1250 1400 1500 1650 1850 2000 2300 2550 2800 3000 3150 3300 3500];
% R = [];

l=2*pi*r;
ii = 0;
NN = 30;
VV = 15;
FLUX = zeros(VV,NN-1);

for N = 2:NN
    ii = ii + 1;
    jj = 0;
    for v0 = 1:VV
        jj = jj + 1;


theta = [0:2*pi/N:2*pi-2*pi/N];
k_verdi = 100;
C=ones(1,N)/k_verdi; % 0.007
I=ones(1,N)*1000;
m=ones(1,N);
dteta=2*pi/N;
a_const = -5; %constant
e = zeros(1,N);
aks = -2;


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
end

dt=0.01;
nq=q; % pos
np=p; % car speed
ne=ones(1,N); % source
j=0;
time = 200;
breakdist = 3;
crashdist = 0;
dangerzone = 2;
xnextcarforward = zeros(1,N);
ynextcarforward = zeros(1,N);
x = zeros(1,N+1);
y_f = zeros(1,N+1);
y = zeros(1,N+1);
theta = 0:2*pi/N:2*pi-2*pi/(N+1);
Rn = zeros(1,N);

%% Må fikse at biler reduserer fart dersom de kommer innenfor breakdist til bil forran
%% Biler kan øke fart dersom de kommer innenfor breakdist til bil bak
for t=0:dt:time % sekund
    j=j+1;
    
    if t == 1
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
        
        %% DEMPER KOBLES INN 
        if distforward(i) < danger(i)
            R(i) = 1000;
        else
            R(i) = 0;
        end

        x(i)=r*cos(mod(s(i),2*pi));
        y_f(i) = y(i);
        y(i)=r*sin(mod(s(i),2*pi));
        
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
   
    %% Euler 2 difflign.
    nq(1)=p(N)/I(N)-p(1)/I(1);
    np(1)=e(1)+q(1)/C(1)-q(2)/C(2)-(p(1)/I(1))*Rn(1);
    ne(1)=a_const*(p(1)/I(1)-v0);
    for i=2:N-1
        nq(i)=p(i-1)/I(i-1)-p(i)/I(i); %pos
        np(i)=e(i)+q(i)/C(i)-q(i+1)/C(i+1)-(p(i)/I(i))*Rn(i); %speed
        %np(i-1)=e(i-1)-q(i)/C(i);
        ne(i)=a_const*(p(i)/I(i)-v0);
        %ne(i-1)=a_const*(p(i-1)/I(i-1)-v0);
        %Re = ((p(i)/I(i))^2-(p(i-1)/I(i-1))^2)./(2*distforward(i));
    end
    nq(N)=p(N-1)/I(N-1)-p(N)/I(N);
    np(N)=q(N)/C(N)-q(1)/C(1)-(p(N)/I(N))*Rn(N);
    ne(N)=a_const*(p(N)/I(N)-v0);
    q=q+dt*nq;
    p=p+dt*np;
    e=e+dt*ne;

end

FLUX(jj,ii) = max(flux);
v0
N


    end
end
