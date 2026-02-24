function R = Rz(psi)
    R = [cosd(psi) -sind(psi) 0;
         sind(psi)  cosd(psi) 0;
         0          0         1];
end