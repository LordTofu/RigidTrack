psi = 45
theta = 45
phi = 45

spsi = sind(psi)
cpsi = cosd(psi)

st = sind(theta)
ct = cosd(theta)

sphi = sind(phi)
cphi = cosd(phi)

R_z = [cpsi, -spsi, 0; spsi, cpsi, 0; 0,0,1]
R_y = [ct, 0, st; 0, 1, 0; -st, 0, ct]
R_x = [1, 0, 0; 0, cphi, -sphi; 0, sphi,cphi]

R = R_x*R_y*R_z