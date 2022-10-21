%% Inverse Kinematic Function
% Function computing the end effector jacobian column for the input
% parameters.
%
% Inputs
% - biTei : transformation matrix of link <i> w.r.t. link <i-1> for the input qi.
% - bTe: tranformation matrix from the base to the end effector
% - jointType: 0 if the joint is revolute, 1 if the joint is prismatic (is
%   referred to the joint corresponding to bTe).
%
% Output
% - h: Jacobian column h(1:3) angular part, h(4:6) linear

function [h] = GetJacobianColumn(biTei, bTe, jointType)
    % this is the versor of translation or rotation
    ki = biTei( (1 : 3) , 3);
    rei = bTe(1:3, 4) - biTei(1:3, 4);

    % Prismatic joint
    if(jointType == 1)
       Jai = [0; 0; 0];
       % The traslation of the end effector is along ki
       Jli = ki;
    end
    
    % Revolute joint 
    if(jointType == 0)
       % The rotation is around ki
       Jai = ki;
       % The traslation of the end effector is along a versor
       % wich is given by the cross (vector) product of ki and rei 
       Jli = cross(ki, rei);
    end
    
    h = [Jai; Jli];
end
