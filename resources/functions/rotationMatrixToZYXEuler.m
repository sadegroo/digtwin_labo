function [yaw, pitch, roll] = rotationMatrixToZYXEuler(R)
    % Function to extract ZYX Euler angles (Yaw, Pitch, Roll) from a 3x3 rotation matrix.
    %
    % Inputs:
    %   R: 3x3 symbolic or numeric rotation matrix
    %
    % Outputs:
    %   yaw: Rotation about the Z-axis (psi)
    %   pitch: Rotation about the Y-axis (theta)
    %   roll: Rotation about the X-axis (phi)
    
    % Validate that input is a 3x3 matrix
    if ~isequal(size(R), [3, 3])
        error('Input matrix must be 3x3.');
    end
    
    % Yaw (psi)
    yaw = atan2(R(2,1), R(1,1));
    
    % Pitch (theta)
    pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
    
    % Roll (phi)
    roll = atan2(R(3,2), R(3,3));
end

