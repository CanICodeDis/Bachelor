1;

twoRs=39e-3;

t_el1=5.80e-3;
dPhase1=5e-4;
V_pp=0.076;
I_pp=1.74;

freq=1/t_el1;
omega=2*pi*freq;

V_amp=V_pp/2;
V_eff=V_amp/sqrt(2);
I_amp=I_pp/2;
I_eff=I_amp/sqrt(2);

Z = V_eff/ I_eff;
twoL =abs( ( Z - twoRs) / (omega * i));
disp("2*L aus Impedanz: "),disp(twoL);

phi_deg=(dPhase1/t_el1)*360;
phi_rad=(phi_deg/180) * pi;
%tan(phi)=(omega*L)/R -> L = (tan(phi) * R)/ omega
twoL =( tan(phi_rad) * twoRs ) / omega;
disp ("Phasenverschiebung :") , disp(phi_deg),disp(phi_rad);
disp ("2*L: ") , disp (twoL);

x= -10:0.1:10;
plot (x,sin(x));
xlabel("x");
ylabel("sin(x)");
title ("Plot-Test");

a = input ("IsWaiting");
