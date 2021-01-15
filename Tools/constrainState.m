function [xCosntrained] = constrainState(x, tNow, tDelay)


    % Quadratic programming
    H = [2 0 0 0; 0 2 0 0; 0 0 2 0; 0 0 0 2];
%     H = 2*pinv(PUpdatedAtDelay); % 2 is to compensate 1/2 inside quadprog
    f = (x'*H)';  % Linear term is 2*x'*H
    A = [0 0 0 -1; 0 0 0 1];    % Constraints: (1) -dTd <= Td equivalent to dT >= -Td
                                %              (2) dTd <= tspan - Td            
    b = [tDelay; tNow-tDelay];
    lb = [-100 -100 -100 -100];
    ub = [100 100 100 100];
    x0 = [x(1:3); 0];
%     options = optimoptions('quadprog', 'Algorithm', 'active-set');
    [xCosntrained, ~] = quadprog(H, f, A, b, [], [], lb, ub, x0);%, options);
end