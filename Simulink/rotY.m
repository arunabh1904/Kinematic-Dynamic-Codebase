%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: rotY
%Returns rotation matrix describing rotation about Y axis

%[R] = rotY(theta)

%R = the rotation matrix describing rotation about Y axis for a given theta

%theta = input angle, rotated about the Y axis in radians


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R = rotY(theta)
R=zeros(3,3);
R =[cos(theta) 0 sin(theta) ; 0 1 0 ;-sin(theta) 0 cos(theta)];
end
