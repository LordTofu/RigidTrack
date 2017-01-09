clear all
ImportLog

figure(1)
subplot(3,1,1)
plot(t, [x, y, z])
xlabel('Time[s]')
legend('x', 'y', 'z')
grid on;
grid minor;

subplot(3,1,2)
plot(t, [velx, vely, velz])
xlabel('Time[s]')
legend('V_x', 'V_y', 'V_z')
grid on;
grid minor;

subplot(3,1,3)
plot(t, [euler1, euler2, euler3])
ylabel('Euler Angle [deg]')
xlabel('Time[s]')
legend('Euler Angles')
grid on;
grid minor;

sprintf('Standard Deviation of Position X is %f mm', std(x))
sprintf('Standard Deviation of Position Y is %f mm', std(y))
sprintf('Standard Deviation of Position Z is %f mm', std(z))
sprintf('Standard Deviation of Velocity X is %f mm/s^2', std(velx))
sprintf('Standard Deviation of Velocity Y is %f mm/s^2', std(vely))
sprintf('Standard Deviation of Velocity Z is %f mm/s^2', std(velz))
sprintf('Standard Deviation of Euler 1 is %f �', std(euler1))
sprintf('Standard Deviation of Euler 2 is %f �', std(euler2))
sprintf('Standard Deviation of Euler 3 is %f �', std(euler3))
