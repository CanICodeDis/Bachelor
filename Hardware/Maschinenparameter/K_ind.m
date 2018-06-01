1;

Upm = [500,1000,2000];
Uind= [0.635,1.25,2.4];
omega_mech = (Upm * pi) / 30;
k_ind = Uind ./ omega_mech;
disp("K_ind aus Drehzahl und EMK"),disp (k_ind);

