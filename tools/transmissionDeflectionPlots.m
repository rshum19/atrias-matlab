figure

subplot(2,2,1)
plot(a.time,(a.right.motorAngleA-a.controllerData(:,end-4)))
title('Right A Transmission Deflection')
xlabel('Time (s)')
ylabel('Motor Side Angle (rad)')
ylim([-0.5 0.5]*10^-2)

subplot(2,2,2)
plot(a.time,(a.right.motorAngleB-a.controllerData(:,end-5)))
title('Right B Transmission Deflection')
xlabel('Time (s)')
ylabel('Motor Side Angle (rad)')
ylim([-0.5 0.5]*10^-2)

subplot(2,2,3)
plot(a.time,(a.left.motorAngleA-a.controllerData(:,end-2)))
title('Left A Transmission Deflection')
xlabel('Time (s)')
ylabel('Motor Side Angle (rad)')
ylim([-0.5 0.5]*10^-2)

subplot(2,2,4)
plot(a.time,(a.left.motorAngleB-a.controllerData(:,end-3)))
title('Left B Transmission Deflection')
xlabel('Time (s)')
ylabel('Motor Side Angle (rad)')
ylim([-0.5 0.5]*10^-2)