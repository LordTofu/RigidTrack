function [ mu, theta, height ] = flat2LLA( flatearth_pos )
xsi = 0
R =  6378137.0
f = 1/298.257223563
mu_0 = 47*pi/180
theta_0 = 45*pi/180

Flat2NED = [cos(xsi), -sin(xsi); sin(xsi), cos(xsi)]
NEDpos = Flat2NED*flatearth_pos(1:2)

R_N = R/sqrt(1-(2*f-f^2)*(sin(mu_0))^2)
R_M = R_N*(1-(2*f-f^2))/(1-(2*f-f^2)*(sin(mu_0))^2)
%[lat, lon, height] = flat2lla(flatearth_pos, [0 45], 0, -10);

d_mu = atan(1/R_M)*flatearth_pos(1)
d_theta = atan(1/(R_N*cos(mu_0)))*flatearth_pos(2)

mu = (mu_0+d_mu)*180/pi
theta = (theta_0 +d_theta)*180/pi
height = flatearth_pos(3)
end

