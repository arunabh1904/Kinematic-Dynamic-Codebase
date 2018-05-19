%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: transform2Twist
%Returns 6x1 twist vector  for a given homogenous transformation matrix
% t1,t2 & t3 are the terms in the equation relating V and d

%t = input 6x1 vector of the form [v, w_t], where w_t = w*theta and v is
%related to the translational vector

%H = the homogenous transformation matrix describing transformation. It is
%mentioned in the following format:
%H = [R d;0 0 0 1];
%R = the rotation matrix about axis w, rotation angle theta
%d = translation vector about which translation is performed

%Name: Vineet Pandey
%CWID: 10826588
%Course Number: MEGN544
%Date: 09/29/2017
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function [ t ] = transform2Twist( H )
R = H(1:3,1:3);
d = H(1:3,4);
[w,theta] = rot2AngleAxis(R);
if theta~=0
    t1 = ((sin(theta)/(2*(1-cos(theta))))*eye(3));
    t2 = (((2*(1-cos(theta))-(theta*sin(theta)))/(2*theta*(1-cos(theta))))*(w*w'));
    t3 = (0.5*cpMap(w));
    v = (t1+t2-t3)*d;
    t = [v;theta*w];
else  
    v=d;
    t = [v;0;0;0];
end
end