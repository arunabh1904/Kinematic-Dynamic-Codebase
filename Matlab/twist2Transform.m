%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Function name: twist2Transform
%Returns homogenous transformation matrix for a given 6x1 twist vector
%[H] = twist2Transform(t)
%Twist vector is mentioned in the following format:
%[v w_t]
%v is taken to be the first 3 rows of t; w, theta are found out from w_t
%rotation matrix 'R' for the rotation angle theta,about axis w is found out
%the translation vector 'd' is found out from v using the inverse of the
%inverse formula 
%w_t is the angular rotation obtained from the twist
%w is the normalised angular rotation 
%H = the homogenous transformation matrix describing transformation
%t = input 6x1 vector of the form [v, w_t], where w_t = w*theta and v is
%related to the translational vector
%t1, t2 & t3 are the 3 terms in the equation relating V and d


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function [ H ] = twist2Transform( t )
v = t(1:3);
w_t = t(4:6);
theta = norm(w_t);
if theta~=0
    w = w_t/theta;
    R = (cos(theta)*eye(3))+(sin(theta)*cpMap(w))+((1-cos(theta))*(w*w'));
    t1 = ((sin(theta)/(2*(1-cos(theta))))*eye(3));
    t2 = (((2*(1-cos(theta))-(theta*sin(theta)))/(2*theta*(1-cos(theta))))*(w*w'));
    t3 = (0.5*cpMap(w));
    d = inv(t1+t2-t3)*v;
    H = [R d; 0 0 0 1];

else 
    R=eye(3,3);
    d=v;
    H = [R d; 0 0 0 1];
end
end
