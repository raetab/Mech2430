close all;
clear;
 
% Constant variables
r = 0.045; % radius (m)
d_eta = 0.13; % length of rod(m) eta
d_2i_M3 = 0.135; %length of rod(m) 2.5i and M3 are the same
CoG_eta_M3 = 0.043; %centre of gravity location for eta & M3
CoG_2i = 0.045; %center of gravity location for 2.5i
Wc = 733.038; % constant angular velocity of crank
theta = 0:0.01:360; % from 0-360 degrees
Mp_eta = 0.489; % mass of piston (kg)for eta 
Mp_M3_2i = 0.468; %mass of the piston (kg) for 2.5i & M3
Mcr_eta = 0.62; % mass of rod (kg) eta
Mcr_2i = 0.64; % mass of rod (kg) 2.5i 
Mcr_M3 = 0.595; % mass of rod (kg) M3
I_eta = 0.020; % moment of inertia of rod(kg*m^2) eta
I_2i = 0.021; % moment of inertia of rod(kg*m^2) 2.5i
I_M3 = 0.018; % moment of inertia of rod(kg*m^2) M3
 
% Relationship between theta and phi
phi_eta = asind((r/d_eta)*sind(theta)); %eta
phi_2i_M3 = asind((r/d_2i_M3)*sind(theta)); %2.5i & M3
 
%Kinematics
 
%Angluar velocity of con-rod
 
Wr_eta = (-Wc.*r.*cosd(theta))./(d_eta.*cosd(phi_eta));%eta
Wr_2i = (-Wc.*r.*cosd(theta))./(d_2i_M3.*cosd(phi_2i_M3));%2.5i
Wr_M3 = (-Wc.*r.*cosd(theta))./(d_2i_M3.*cosd(phi_2i_M3)); %M3
 
figure(1)
hold on;
plot(theta,Wr_eta, 'r')
plot(theta,Wr_2i, '-.b')
plot(theta,Wr_M3, '--g')
title('Angular Velocity of Connecting Rod')
xlabel('Theta (degress)')
ylabel('Connecting Rod Angular Velocity (rad/s)')
legend('eta', '2.5i', 'M3');
 
 
%Velocity of piston (P) (only in the j-direction)
 
Vp_eta = Wc.*r.*sind(theta) - Wr_eta.*d_eta.*sind(phi_eta); %eta
Vp_2i = Wc.*r.*sind(theta) - Wr_2i.*d_2i_M3.*sind(phi_2i_M3); %2.5i 
Vp_M3 = Wc.*r.*sind(theta) - Wr_M3.*d_2i_M3.*sind(phi_2i_M3); %M3
 
Vp_max_eta = max(Vp_eta)
Vp_max_2i = max(Vp_2i)
Vp_max_M3 = max(Vp_M3)
 
figure(2)
hold on;
plot(theta,Vp_eta, 'r')
plot(theta,Vp_2i, '-.b')
plot(theta,Vp_M3, '--g')
title('Velocity of Piston(only in the j-direction) ')
xlabel('Theta (degress)')
ylabel(' Vp (m/s) ')
legend('eta', '2.5i', 'M3');
 
%Acceleration of point C (Con-rod connection on crank)
 
Ac = -((Wc.^2).*r.*sind(theta)) + (-(Wc.^2).*r.*cosd(theta));
 
 
 
%Angular Acceleration of con-rod
 
Acr_eta = (-(Wc.^2).*r.*sind(theta) + (Wr_eta.^2).*d_eta.*sind(phi_eta))./(d_eta.*cosd(phi_eta)); %eta
Acr_2i = (-(Wc.^2).*r.*sind(theta) + (Wr_2i.^2).*d_2i_M3.*sind(phi_2i_M3))./(d_2i_M3.*cosd(phi_2i_M3)); %2.5i
Acr_M3 = (-(Wc.^2).*r.*sind(theta) + (Wr_M3.^2).*d_2i_M3.*sind(phi_2i_M3))./(d_2i_M3.*cosd(phi_2i_M3)); %M3
 
figure(3)
hold on;
plot(theta,Acr_eta, 'r')
plot(theta,Acr_2i, '-.b')
plot(theta,Acr_M3, '--g')
title('Angular Acceleration of Connecting Rod')
xlabel('Theta (degress)')
ylabel('Connecting Rod Angular Acceleration (rad/s^2)')
legend('eta', '2.5i', 'M3');
 
%Acceleration of piston (P) (only in the j-direction)
 
Ap_eta = -((Wc.^2).*r.*cosd(theta)) - (Acr_eta.*d_eta.*sind(phi_eta)) - ((Wr_eta.^2).*d_eta.*cosd(phi_eta));%eta
Ap_2i = -((Wc.^2).*r.*cosd(theta)) - (Acr_2i.*d_2i_M3.*sind(phi_2i_M3)) - ((Wr_2i.^2).*d_2i_M3.*cosd(phi_2i_M3));%2.5i
Ap_M3 = -((Wc.^2).*r.*cosd(theta)) - (Acr_M3.*d_2i_M3.*sind(phi_2i_M3)) - ((Wr_M3.^2).*d_2i_M3.*cosd(phi_2i_M3));%M3
 
Ap_max_eta = max(Ap_eta)
Ap_max_2i = max(Ap_2i)
Ap_max_M3 = max(Ap_M3)
 
 
figure(4)
hold on;
plot(theta,Ap_eta, 'r')
plot(theta,Ap_2i, '-.b')
plot(theta,Ap_M3, '--g')
title('Acceleration of piston(only in the j-direction)')
xlabel('Theta (degress)')
ylabel(' Ap (m/s^2) ')
legend('eta', '2.5i', 'M3');
 
%Acceleration of con-rod CoM in the x direction (i)
AGx_eta = Acr_eta.*(d_eta-CoG_eta_M3).*cosd(phi_eta) - (Wr_eta.^2).*(d_eta-CoG_eta_M3).*sind(phi_eta); %eta
AGx_2i = Acr_2i.*(d_2i_M3-CoG_2i).*cosd(phi_2i_M3) - (Wr_2i.^2).*(d_2i_M3-CoG_2i).*sind(phi_2i_M3); %2.5i
AGx_M3 = Acr_M3.*(d_2i_M3-CoG_eta_M3).*cosd(phi_2i_M3) - (Wr_M3.^2).*(d_2i_M3-CoG_eta_M3).*sind(phi_2i_M3); %M3
 
%Acceleration of con-rod CoM in the y direction (j)
AGy_eta = Ap_eta + Acr_eta.*(d_eta-CoG_eta_M3).*sind(phi_eta) + (Wr_eta.^2).*(d_eta-CoG_eta_M3).*cosd(phi_eta); %eta
AGy_2i = Ap_2i + Acr_2i.*(d_2i_M3-CoG_2i).*sind(phi_2i_M3) + (Wr_2i.^2).*(d_2i_M3-CoG_2i).*cosd(phi_2i_M3); %2.5i
AGy_M3 = Ap_M3 + Acr_M3.*(d_2i_M3-CoG_eta_M3).*sind(phi_2i_M3) + (Wr_M3.^2).*(d_2i_M3-CoG_eta_M3).*cosd(phi_2i_M3); %M3
 
%Kinetics
%Force of the piston acting in the y direction
 
Py_eta = Mp_eta.*9.81 + Mp_eta.*Ap_eta; %eta
Py_2i = Mp_M3_2i.*9.81 + Mp_M3_2i.*Ap_2i; %2i
Py_M3 = Mp_M3_2i.*9.81 + Mp_M3_2i.*Ap_M3; %M3
 
Py_max_eta = max(Py_eta)
Py_max_2i = max(Py_2i)
Py_max_M3 = max(Py_M3)
 
 
figure(5)
hold on;
plot(theta,Py_eta, 'r')
plot(theta,Py_2i, '-.b')
plot(theta,Py_M3, '--g')
title('Force of the Piston acting in the y-direction')
xlabel('Theta (degress)')
ylabel(' Force of Py  (N)')
legend('eta', '2.5i', 'M3');
 
%Force of Crank in Y direction 
 
Cy_eta = Mcr_eta.*AGy_eta + Py_eta + Mcr_eta.*9.81; %eta
Cy_2i = Mcr_2i.*AGy_2i + Py_2i + Mcr_2i.*9.81; %2i
Cy_M3 = Mcr_M3.*AGy_M3 + Py_M3 + Mcr_M3.*9.81; %M3
 
Cy_max_eta = max(Cy_eta)
Cy_max_2i = max(Cy_2i)
Cy_max_M3 = max(Cy_M3)
 
 
figure(6)
hold on;
plot(theta,Cy_eta,'r')
plot(theta,Cy_2i,'-.b')
plot(theta,Cy_M3,'--g')
title('Force of the Crank in y-direction')
xlabel('Theta (degress)')
ylabel('Force of Cy (N)')
legend('eta', '2.5i', 'M3');
 
%Force of the Piston in the x direction - using moment of inertia
 
Px_eta = ((CoG_eta_M3.*cosd(phi_eta).*Mcr_eta.*AGx_eta - I_eta.*Acr_eta + CoG_eta_M3.*sind(phi_eta).*Cy_eta + (d_eta-CoG_eta_M3).*sind(phi_eta).*Py_eta))./(d_eta.*cosd(phi_eta)); %eta
Px_2i = ((CoG_2i.*cosd(phi_2i_M3).*Mcr_2i.*AGx_2i - I_2i.*Acr_2i + CoG_2i.*sind(phi_2i_M3).*Cy_2i + (d_2i_M3-CoG_2i).*sind(phi_2i_M3).*Py_2i))./(d_2i_M3.*cosd(phi_2i_M3)); %2i
Px_M3 = ((CoG_eta_M3.*cosd(phi_2i_M3).*Mcr_M3.*AGx_M3 - I_M3.*Acr_M3 + CoG_eta_M3.*sind(phi_2i_M3).*Cy_M3 + (d_2i_M3-CoG_eta_M3).*sind(phi_2i_M3).*Py_M3))./(d_2i_M3.*cosd(phi_2i_M3)); %M3
 
Px_max_eta = max(Px_eta)
Px_max_2i = max(Px_2i)
Px_max_M3 = max(Px_M3)
 
figure(7)
hold on;
plot(theta,Px_eta, 'r')
plot(theta,Px_2i, '-.b')
plot(theta,Px_M3, '--g')
title('Force of the Piston in the x-direction')
xlabel('Theta (degress)')
ylabel('Force of Px (N)')
legend('eta', '2.5i', 'M3');
 
%Force of the Crank in the x direction
 
Cx_eta = Px_eta-(Mcr_eta*AGx_eta);
Cx_2i = Px_2i-(Mcr_2i*AGx_2i);
Cx_M3 = Px_M3-(Mcr_M3*AGx_M3);
 
Cx_max_eta = max(Cx_eta)
Cx_max_2i = max(Cx_2i)
Cx_max_M3 = max(Cx_M3)
 
figure(8)
hold on;
plot(theta, Cx_eta, 'r')
plot(theta, Cx_2i, '-.b')
plot(theta, Cx_M3, '--g')
title('Force of the Crank in the x-direction')
xlabel('Theta (degress)')
ylabel('Force of Cx (N)')
legend('eta', '2.5i', 'M3');
 
%Force of the wall on the piston
figure(9)
FL_eta = Cx_eta;
FL_2i = Cx_2i;
FL_M3 = Cx_M3;
 
hold on;
plot(theta, FL_eta, 'r')
plot(theta, FL_2i, '-.b')
plot(theta, FL_M3, '--g')
title('Force of the wall on the piston')
xlabel('Theta (degress)')
ylabel('Force of FL (N)')
legend('eta', '2.5i', 'M3');
 
% Magnitude of the gudgeon pin
 
P_mag_eta = sqrt(Px_eta.*Px_eta + Py_eta.*Py_eta);
P_mag_2i = sqrt(Px_2i.*Py_2i + Py_2i.*Py_2i);
P_mag_M3 = sqrt(Px_M3.*Px_M3 + Py_M3.*Py_M3);
 
% Magnitude of the journal bearing
C_mag_eta = sqrt(Cx_eta.*Cx_eta + Cy_eta.*Cy_eta);
C_mag_2i = sqrt(Cx_2i.*Cx_2i + Cy_2i.*Cy_2i);
C_mag_M3 = sqrt(Cx_M3.*Cx_M3 + Cy_M3.*Cy_M3);
 
% Maximum load on the gudgeon pin
P_max_eta = max(P_mag_eta * 10^-3);
P_max_2i = max(P_mag_2i * 10^-3)
P_max_M3 = max(P_mag_M3 * 10^-3)
 
% Maximum load on the journal bearing
C_max_eta = max(C_mag_eta * 10^-3);
C_max_2i = max(C_mag_2i * 10^-3)
C_max_M3 = max(C_mag_M3 * 10^-3)
 
 
% Determine maximum angular velocity of the 2.5i connecting rod
%need to comment out while 2 to evaluate the max 2.5i angular velocity 
while 1
    Wc = Wc + 0.1
    %using the above equations for 2.5i:
    phi_2i_M3 = asind((r/d_2i_M3)*sind(theta)); %2.5i & M3
    Wr_2i = (-Wc.*r.*cosd(theta))./(d_2i_M3.*cosd(phi_2i_M3));%2.5i
    Vp_2i = Wc.*r.*sind(theta) - Wr_2i.*d_2i_M3.*sind(phi_2i_M3); %2.5i 
    Ac = -((Wc.^2).*r.*sind(theta)) + (-(Wc.^2).*r.*cosd(theta));
    Acr_2i = (-(Wc.^2).*r.*sind(theta) + (Wr_2i.^2).*d_2i_M3.*sind(phi_2i_M3))./(d_2i_M3.*cosd(phi_2i_M3)); %2.5i
    Ap_2i = -((Wc.^2).*r.*cosd(theta)) - (Acr_2i.*d_2i_M3.*sind(phi_2i_M3)) - ((Wr_2i.^2).*d_2i_M3.*cosd(phi_2i_M3));%2.5i
    AGx_2i = Acr_2i.*(d_2i_M3-CoG_2i).*cosd(phi_2i_M3) - (Wr_2i.^2).*(d_2i_M3-CoG_2i).*sind(phi_2i_M3); %2.5i
    AGy_2i = Ap_2i + Acr_2i.*(d_2i_M3-CoG_2i).*sind(phi_2i_M3) + (Wr_2i.^2).*(d_2i_M3-CoG_2i).*cosd(phi_2i_M3); %2.5i
    Py_2i = Mp_M3_2i.*9.81 + Mp_M3_2i.*Ap_2i; %2i
    Cy_2i = Mcr_2i.*AGy_2i + Py_2i + Mcr_2i.*9.81; %2i
    Px_2i = ((CoG_2i.*cosd(phi_2i_M3).*Mcr_2i.*AGx_2i - I_2i.*Acr_2i + CoG_2i.*sind(phi_2i_M3).*Cy_2i + (d_2i_M3-CoG_2i).*sind(phi_2i_M3).*Py_2i))./(d_2i_M3.*cosd(phi_2i_M3)); %2i
    Cx_2i = Px_2i-(Mcr_2i*AGx_2i);
    P_mag_2i = sqrt(Px_2i.^2 + Py_2i.^2);
    C_mag_2i = sqrt(Cx_2i.*Cx_2i + Cy_2i.*Cy_2i);
    P_max_2i = max(P_mag_2i * 10^-3);
    C_max_2i = max(C_mag_2i * 10^-3);
    
    if P_max_2i > P_max_eta
        break
    end
end
 
 
%determining the maximum angular velocity for the M3 connecting rod
 
while 2
    Wc = Wc + 0.1
    %using the above equations for M3:
    phi_2i_M3 = asind((r/d_2i_M3)*sind(theta)); %2.5i & M3
    Wr_M3 = (-Wc.*r.*cosd(theta))./(d_2i_M3.*cosd(phi_2i_M3)); %M3
    Vp_M3 = Wc.*r.*sind(theta) - Wr_M3.*d_2i_M3.*sind(phi_2i_M3); %M3
    Ac = -((Wc.^2).*r.*sind(theta)) + (-(Wc.^2).*r.*cosd(theta));
    Acr_M3 = (-(Wc.^2).*r.*sind(theta) + (Wr_M3.^2).*d_2i_M3.*sind(phi_2i_M3))./(d_2i_M3.*cosd(phi_2i_M3)); %M3
    Ap_M3 = -((Wc.^2).*r.*cosd(theta)) - (Acr_M3.*d_2i_M3.*sind(phi_2i_M3)) - ((Wr_M3.^2).*d_2i_M3.*cosd(phi_2i_M3));%M3
    AGx_M3 = Acr_M3.*(d_2i_M3-CoG_eta_M3).*cosd(phi_2i_M3) - (Wr_M3.^2).*(d_2i_M3-CoG_eta_M3).*sind(phi_2i_M3); %M3
    AGy_M3 = Ap_M3 + Acr_M3.*(d_2i_M3-CoG_eta_M3).*sind(phi_2i_M3) + (Wr_M3.^2).*(d_2i_M3-CoG_eta_M3).*cosd(phi_2i_M3); %M3
    Py_M3 = Mp_M3_2i.*9.81 + Mp_M3_2i.*Ap_M3; %M3
    Cy_M3 = Mcr_M3.*AGy_M3 + Py_M3 + Mcr_M3.*9.81; %M3
    Px_M3 = ((CoG_eta_M3.*cosd(phi_2i_M3).*Mcr_M3.*AGx_M3 - I_M3.*Acr_M3 + CoG_eta_M3.*sind(phi_2i_M3).*Cy_M3 + (d_2i_M3-CoG_eta_M3).*sind(phi_2i_M3).*Py_M3))./(d_2i_M3.*cosd(phi_2i_M3)); %M3
    Cx_M3 = Px_M3-(Mcr_M3*AGx_M3);
    P_mag_M3 = sqrt(Px_M3.*Px_M3 + Py_M3.*Py_M3);
    C_mag_M3 = sqrt(Cx_M3.*Cx_M3 + Cy_M3.*Cy_M3);
    P_max_M3 = max(P_mag_M3 * 10^-3);
    C_max_M3 = max(C_mag_M3 * 10^-3);
 
    if P_max_M3 > P_max_eta
        break
    end
end
 
