clear all
ImportLog
samples_end = 43*100;
samples_start =40*100;
x = x(samples_start:samples_end)
y = y(samples_start:samples_end)
z = z(samples_start:samples_end)
velx = velx(samples_start:samples_end)
vely = vely(samples_start:samples_end)
velz = velz(samples_start:samples_end)
euler1 = euler1(samples_start:samples_end)
euler2 = euler2(samples_start:samples_end)
euler3 = euler3(samples_start:samples_end)
t = t(samples_start:samples_end)
figure(1)
subplot(3,1,1)
plot(t, [x, y, z])
xlabel('Time[s]')
legend('x', 'y', 'z')
grid on;
ylim([-2000, 2000])
grid minor;

subplot(3,1,2)
plot(t, [velx, vely, velz])
xlabel('Time[s]')
legend('V_x', 'V_y', 'V_z')
grid on;
ylim([-2, 2])
grid minor;

subplot(3,1,3)
plot(t, [euler1, euler2, euler3])
ylabel('Euler Angle [deg]')
xlabel('Time[s]')
legend('Roll', 'Pitch', 'Heading')
ylim([-180, 180])
grid on;
grid minor;

t=t-t(1);
posz_sim=timeseries(z,t);

% compute velocity only with every 10th timestep
t_downSamp = zeros(idivide(numel(t), int32(10)),1);
posz_downSamp = zeros(numel(t_downSamp),1);
velz_downSamp = zeros(numel(t_downSamp),1);

for i=1:numel(t_downSamp)
   t_downSamp(i) = t(i*10);
   posz_downSamp(i) = z(i*10);
end

for i=2:numel(t_downSamp)
   velz_downSamp(i) = (posz_downSamp(i) - posz_downSamp((i-1)))/(t_downSamp(i) - t_downSamp((i-1)));
end

% velz_filtered = doFilter(velz);
% plot(t, velz, t_downSamp, velz_downSamp);
% ylim([-3000, 3000])

sprintf('Standard Deviation of Position X is %f mm', std(x))
sprintf('Standard Deviation of Position Y is %f mm', std(y))
sprintf('Standard Deviation of Position Z is %f mm', std(z))
sprintf('Standard Deviation of Velocity X is %f m/s', std(velx))
sprintf('Standard Deviation of Velocity Y is %f m/s', std(vely))
sprintf('Standard Deviation of Velocity Z is %f m/s', std(velz))
sprintf('Standard Deviation of Velocity Z DS is %f mm/s', std(velz_downSamp))
sprintf('Standard Deviation of Euler 1 is %f °', std(euler1))
sprintf('Standard Deviation of Euler 2 is %f °', std(euler2))
sprintf('Standard Deviation of Euler 3 is %f °', std(euler3))

