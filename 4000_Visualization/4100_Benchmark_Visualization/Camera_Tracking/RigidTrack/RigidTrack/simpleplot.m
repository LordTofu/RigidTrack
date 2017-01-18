clear all
ImportLog
samples = 15*100;

% x = x(1:samples)
% y = y(1:samples)
% z = z(1:samples)
% velx = velx(1:samples)
% vely = vely(1:samples)
% velz = velz(1:samples)
% euler1 = euler1(1:samples)
% euler2 = euler2(1:samples)
% euler3 = euler3(1:samples)
% t = t(1:samples)
figure(1)
subplot(3,1,1)
plot(t, [x, y, z])
xlabel('Time[s]')
legend('x', 'y', 'z')
grid on;
ylim([-3000, 3000])
grid minor;

subplot(3,1,2)
plot(t, [velx, vely, velz])
xlabel('Time[s]')
legend('V_x', 'V_y', 'V_z')
grid on;
ylim([-3000, 3000])
grid minor;

subplot(3,1,3)
plot(t, [euler1, euler2, euler3])
ylabel('Euler Angle [deg]')
xlabel('Time[s]')
legend('Roll', 'Pitch', 'Heading')
ylim([-180, 180])
grid on;
grid minor;

sprintf('Standard Deviation of Position X is %f mm', std(x))
sprintf('Standard Deviation of Position Y is %f mm', std(y))
sprintf('Standard Deviation of Position Z is %f mm', std(z))
sprintf('Standard Deviation of Velocity X is %f mm/s', std(velx))
sprintf('Standard Deviation of Velocity Y is %f mm/s', std(vely))
sprintf('Standard Deviation of Velocity Z is %f mm/s', std(velz))
sprintf('Standard Deviation of Euler 1 is %f �', std(euler1))
sprintf('Standard Deviation of Euler 2 is %f �', std(euler2))
sprintf('Standard Deviation of Euler 3 is %f �', std(euler3))

t=t-t(1);
posz_sim=timeseries(z,t);

velz_filtered = doFilter(velz);
plot(t, velz, t, velz_filtered);
ylim([-3000, 3000])

velx_filtered = doFilter(velx);
plot(t, velx, t, velx_filtered);
ylim([-3000, 3000])


