figure(1);
plot(xx(1,:),xx(2,:),'--','Linewidth',1.2); hold on; plot(x_tra,y_tra,'-');
legend('Actual trajectory','Reference Trajectory')
figure(2);
plot(xx(3,:)); hold on; plot(theta_tra);
legend('Actual trajectory','Reference Trajectory')
figure(3);
plot(xx(2,:)); hold on; plot(y_tra);
legend('Actual trajectory','Reference Trajectory')
figure(4);
plot(xx(1,:)); hold on; plot(x_tra);
legend('Actual trajectory','Reference Trajectory')
figure(5)
subplot(211)
stairs(t,u_cl(:,1),'k','linewidth',1.5); axis([0 t(end) -0.2 0.8])
ylabel('v (m/s)')
grid on
subplot(212)
stairs(t,u_cl(:,2),'r','linewidth',1.5); %axis([0 t(end) -0.85 0.85])
xlabel('time (seconds)')
ylabel('\omega (rad/s)')
grid on

