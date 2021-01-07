function generatePlots()
results = load('results.mat');
% True trajectory vs measurements
figure(1);
sps = [2 2];
position();
velocity();
positionErrors();
velocityErrors();

    function position()
        subplot(sps(1), sps(2), 1);
        hold off;
        % True position
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.trueTrajectory(:, 1));
        hold on;
        plot(results.tVec, results.trueTrajectory(:, 2));
        % IMU open-loop position:
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.estIMURaw.pos(:, 1), '--');
        plot(results.tVec, results.estIMURaw.pos(:, 2), '--');
        % GNSS position
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.pGNSS(:, 1), '.');
        plot(results.tVec, results.pGNSS(:, 2), '.');
        
        title('Position')
        xlabel('Time [s]');
        ylabel('Position [m]');
        legend({'North true', 'East true', 'North INS', 'East INS', 'North GNSS', 'East GNSS'});
    end

    function velocity()
        subplot(sps(1), sps(2), 2);
        hold off;
        % True:
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.trueTrajectory(:, 3));
        hold on;
        % IMU open-loop:
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.estIMURaw.vel(:, 1), '--');
        
        title('Velocity')
        xlabel('Time [s]');
        ylabel('Velocity[mps]');
        legend({'true', 'INS'});
    end

    function positionErrors()
        subplot(sps(1), sps(2), 3);
        hold off;
        % INS open-loop position:
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.errPosIMU, '--');
        hold on;
        % GNSS position
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.errPosGNSS, '.');        
        % INS corrected by EKF:
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.errPosEKF, '-');
        hold on;
        
        title('Position errors')
        xlabel('Time [s]');
        ylabel('Position Error[m]');
        legend({'North INS open loop', 'East INS open loop', 'North GNSS', 'East GNSS', 'North INS corrected by EKF', 'East INS corrected by EKF'});
    end

function velocityErrors()
        subplot(sps(1), sps(2), 4);
        hold off;
        % INS open-loop position:
        set(gca,'ColorOrderIndex',1);
        plot(results.tVec, results.errVelIMU, '--');
        hold on;
        % INS corrected by EKF:
        plot(results.tVec, results.errVelEKF, '-');
        hold on;
        
        title('Velocity errors')
        xlabel('Time [s]');
        ylabel('Velocity Error [mps]');
        legend({'INS open loop', 'INS corrected by EKF'});
    end
end

