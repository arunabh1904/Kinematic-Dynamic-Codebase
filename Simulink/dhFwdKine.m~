% dhFwdKine: Returns the forward kinematics of a manipulator with the provided DH parameter
%
% H = dhFwdKine(linkList, paramList) Returns the forward kinematics of a manipulator 
% with the provided DH parameter set

% H ( homogenous transformation matrix 4-by-4)
%
% linkList = linkList is to be an array of links each created by createLink
% function 
% paramList = paramList is to be an array containing the current state of their joint variables
%
% Vineet Pandey
% 10826588

function H = dhFwdKine(linkList, paramList)
N = length(linkList);
H = eye(4);

for i = 1:N
    if linkList(i).isRotary == true
        H = H*dhTransform(linkList(i).a,linkList(i).d,linkList(i).alpha,paramList(i));
    else
        H = H*dhTransform(linkList(i).a,paramList(i),linkList(i).alpha,linkList(i).theta);
    end
end

end

