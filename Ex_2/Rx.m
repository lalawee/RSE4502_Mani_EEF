function R = Rx(phi)
    R = [1  0          0;
         0  cosd(phi) -sind(phi);
         0  sind(phi)  cosd(phi)];
end