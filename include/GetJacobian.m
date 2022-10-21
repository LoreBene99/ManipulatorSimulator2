%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei : transformation matrix of link <i> w.r.t. link <i-1> for the input qi.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

function J = GetJacobian(biTei, bTe, jointType)

    numLinks = length(jointType);
    % initialize J
    J = zeros(6, numLinks);

    for i = 1 : numLinks
        % Compute the trasmormation matrix with respect to the base,
        % it is necessary as argument of GetJacobianColumn  
        bTi = GetTransformationWrtBase(biTei, i);
        % Here we call GetJacobianColumn for each joint
        J(:, i) = GetJacobianColumn(bTi, bTe, jointType(i));
    end


end
