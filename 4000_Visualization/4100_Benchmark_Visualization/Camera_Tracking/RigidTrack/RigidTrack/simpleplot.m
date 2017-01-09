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

