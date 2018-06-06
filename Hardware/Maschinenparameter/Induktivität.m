1;

p=7;

twoRs=39e-3;

t_el=[3.880e-3,5.8e-3,2.95e-3];
dPhase=[4.4e-4,5.0e-4,3.9e-4];
V_pp=[9.0e-2,0.076,0.1];
I_pp=[1.74,1.74,1.76];

freq=1./t_el
freq_mech = freq/p
omega=2*pi.*freq

V_amp = V_pp/2;
V_eff = V_amp/sqrt(2);
I_amp=I_pp/2;
I_eff=I_amp/sqrt(2);


phi_deg=(dPhase./t_el)*360
phi_rad=(phi_deg./180) * pi
%tan(phi)=(omega*L)/R -> L = (tan(phi) * R)/omega
twoL =( tan(phi_rad) * twoRs ) ./ omega;
disp ("Phasenverschiebung :") , disp(phi_deg),disp(phi_rad);
disp ("L: ") , disp (twoL);

Vs = V_eff .* e.^(i*0);
Is = I_eff .* e.^(i*phi_rad);
Z = Vs./Is;
% Z = R + jwL
wL = imag(Z);
L= wL ./ omega
%tau_el = twoL/ twoRs
