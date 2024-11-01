function [EOM, lambda_eqs] = derive_EOM(L, q, Q, constraints)
    % DERIVE_EOM Derives the equations of motion using Lagrangian mechanics
    %
    % Inputs:
    %   L: Lagrangian (T - V), where T is kinetic energy and V is potential energy
    %   q: Vector of generalized coordinates (symbolic)
    %   q_dot: Vector of generalized velocities (symbolic)
    %   q_ddot: Vector of generalized accelerations (symbolic)
    %   Q: Vector of generalized forces/torques (symbolic)
    %   constraints: Constraint equations (symbolic), optional
    %
    % Outputs:
    %   EOM: Equations of motion (symbolic)
    %   lambda_eqs: Equations for Lagrange multipliers if constraints exist
    
    % Number of generalized coordinates
    n = length(q);

    syms t real
    assume(q, 'real')
    assume(L,'real')
    
    % Time derivatives of generalized coordinates
    q_dot = diff(q, t); % q_dot is the time derivative of q
    q_ddot = diff(q_dot, t); % q_ddot is the second time derivative of q

    assume(q_dot, 'real')
    assume(q_ddot, 'real')
    
    % Initialize equations of motion
    EOM = sym(zeros(n, 1));
    
    % Loop over each generalized coordinate q_i
    for i = 1:n
        % Partial derivative of L with respect to q_i
        dL_dq = diff(L, q(i));  % ∂L/∂q_i
        
        % Partial derivative of L with respect to q_dot_i
        dL_dq_dot = diff(L, q_dot(i));  % ∂L/∂(q_dot_i)
        
        % Time derivative of ∂L/∂(q_dot_i) using chain rule
        d_dt_dL_dq_dot = diff(dL_dq_dot, t);  % Total time derivative
        
        % Generalized force (external forces, control inputs, etc.)
        if isempty(Q) || all(Q==0)
            Q_i = sym(0);
        else
            Q_i = Q(i);
        end
        
        % Lagrange's equation for the i-th coordinate
        EOM(i) = simplify(d_dt_dL_dq_dot - dL_dq - Q_i);
    end
    
    % If constraints are provided, incorporate constraint forces (Lagrange multipliers)
    if nargin > 3 && ~isempty(constraints)
        m = length(constraints);  % Number of constraints
        lambda = sym('lambda', [m, 1]);  % Lagrange multipliers
        
        % Compute Jacobian of the constraint equations (constraint matrix)
        J_c = jacobian(constraints, q);
        
        % Lagrange multipliers terms for the equations of motion
        lambda_eqs = simplify(J_c' * lambda); 
        
        % Modify the equations of motion to account for constraints
        EOM = EOM + lambda_eqs;
        
        % Constraints for the accelerations: J_c * q_ddot = 0
        constraint_eqs = simplify(J_c * q_ddot);
    else
        lambda_eqs = [];
        constraint_eqs = [];
    end
    
    % Output the final equations of motion (including constraints if applicable)
    EOM = simplify(EOM);
    if nargin > 3 && ~isempty(constraints)
        lambda_eqs = simplify(lambda_eqs);  % Constraint force equations, if any
    end

end



