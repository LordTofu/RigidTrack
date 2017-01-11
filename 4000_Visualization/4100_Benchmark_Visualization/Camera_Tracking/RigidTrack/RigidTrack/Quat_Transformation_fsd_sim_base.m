vecx =-164.062500466971;
vecy =162.006758231012;
vecz =745.9743077102357;
vec =[vecx;vecy;vecz];
angle = norm(vec,2);
vec =[vecx;vecy;vecz]/angle;
q =[cos(angle/2);sin(angle/2)*vec];
M_BO = transpose(M_fr_quat(q))