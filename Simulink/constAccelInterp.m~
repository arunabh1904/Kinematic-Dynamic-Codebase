% constAccelInterp provides the position, velocity and acceleration of the
% trajectory
% 
% 
% 
% [p,v,a] = constAccelInterp(t,trajectory,transPercent)
% Provides the position (p), velocity (v), and acceleration (a) at a time t
% for a trajectory interpolated using the constant acceleration approach.
% Each of these are length M vectors corresponding to the dimensions.
% 
% Outputs
% p - shows the position of the object at a given time t, and is of length M 
% v - shows the velocity of the object at a given time t, and is of length M 
% a - shows the acceleration of the object at a given time t, and is of length M 
% 
% 
% Inputs
% t is the time of interest
% 
% trajectory – a Nx(M+1) array of points. There are N waypoints in the trajectory of dimension M.
% The first column is the time for each point in the trajectory and the remaining M columns are
% the point to be reached at that time.
%
% transPercent – The percentage of the trajectory
% time to use for the constant acceleration
% transition.  
% Vineet Pandey
% 10826588




function [p,v,a] = constAccelInterp(t,trajectory,transPercent)
%% Setup
[m,n]=size(trajectory);
t_traj=trajectory(:,1);
for i=1:(length(t_traj)-1)
    tau(i)=transPercent*(t_traj(i+1)-t_traj(i));
end
for i=2:n
    x(:,i-1)=trajectory(:,i);
end

for i=1:n-1
    for j=1:m-2

        delta_p1=x(j+1,i)-x(j,i);
        delta_p2=x(j+2,i)-x(j+1,i);

%Trajectory along Const Vel and acc
        if t_traj(j)<= t && (t_traj(j+1)-tau(j))>=t
            p(i)=x(j+1,i)-(((t_traj(j+1)-t)/t_traj(j+1))*delta_p1);
            a(i)=0;
            v(i)=delta_p1/t_traj(j+1);
        elseif (t_traj(j+1)-tau(j))<=t && (t_traj(j+1)+tau(j))>=t
            p(i)=(x(j+1,i))-(delta_p1/(4*tau(j)*t_traj(j+1)))*(t-t_traj(j+1)-tau(j))^2+(delta_p2/(4*tau(j)*t_traj(2+1)))*(t-t_traj(j+1)+tau(j))^2;
            a(i)=(0.5/tau(j))*((delta_p2/t_traj(j+2))-(delta_p1/t_traj(j+1)));
            v(i)=(delta_p1/x(j+1))+a(i)*(t-(0.5*tau(j)));
            break;
        elseif (t_traj(j+1)+tau(j))<=t && (t_traj(j+1)+t_traj(j+2))>=t
            p(i)=x(j+1,i)+(((t-t_traj(j+1))/t_traj(j+2))*delta_p2);
            a(i)=0;
            v(i)=delta_p2/t_traj(j+2);
        end

     end
end

end