% abbInvKine: Returns the joint angles required to reach the desired transformation for the ABB arm
% 
%[ th1,th2,th3,th4,th5,th6,reachable ] = abbInvKine( T_des, th_last )
% Returns the joint angles required to reach the desired transformation for the ABB arm
% 
% 
% th1 – th6 :the 6 joint angles. These are real scalar values if th_last is provided and are 8x1 vectors if th_last is not provided.
% reachable = is true if the transform can be achieved else false
%
% T_des - T_des is the desired homogeneous transform(4-by-4)
% th_last-th_last is a 6x1 vector of the last set of thetas used to select a specific solution
%Vineet Pandey
%10826588


function [ th1,th2,th3,th4,th5,th6,reachable ] = abbInvKine( T_des, th_last )

%% Joint Parameters
a = [0 0.270 0.070 0 0 0];
d = [0.290 0 0 0.302 0 0.072];
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0];
d06 = T_des(1:3,4);
z06 = T_des(1:3,3);
d05 = d06 - z06*d(6);
z00 = [0; 0; 1];
theta = zeros(8,6);
%% determining the value of theta1
for i = 1:4
    theta(i,1) = atan2(d05(2),d05(1));
end

for i = 5:8
    theta(i,1) = atan2(d05(2),d05(1))+pi;
end


T01_l = dhTransform(a(1),d(1),alpha(1),theta(1,1));
T01_r = dhTransform(a(1),d(1),alpha(1),theta(5,1));

d15 = zeros(3,1,8);

for i = 1:4
    d15(:,:,i) = (T01_l(1:3,1:3).')*(d06-z06*d(6)-z00*d(1));
end
for i = 5:8
    d15(:,:,i) = (T01_r(1:3,1:3).')*(d06-z06*d(6)-z00*d(1));
end


%% determining the value of theta3
d15_mag = zeros(2,1);
d15_mag(1) = sqrt(d15(1,:,1)^2+d15(2,:,1)^2+d15(3,:,1)^2);
d15_mag(2) = sqrt(d15(1,:,5)^2+d15(2,:,2)^2+d15(3,:,5)^2);


nu = atan2(d(4),a(3));

phi_l_p = 2*atan(sqrt((2*a(2)*sqrt(d(4)^2+a(3)^2)-a(2)^2-d(4)^2-a(3)^2+d15_mag(1)^2)/(2*a(2)*sqrt(d(4)^2+a(3)^2)+a(2)^2+d(4)^2+a(3)^2-d15_mag(1)^2)));
phi_l_n = -phi_l_p;

phi_r_p = 2*atan(sqrt((2*a(2)*sqrt(d(4)^2+a(3)^2)-a(2)^2-d(4)^2-a(3)^2+d15_mag(2)^2)/(2*a(2)*sqrt(d(4)^2+a(3)^2)+a(2)^2+d(4)^2+a(3)^2-d15_mag(2)^2)));
phi_r_n = -phi_r_p;

for i = 1:8
    if i == 1 || i == 2
        theta(i,3) = pi - phi_l_p - nu;
    elseif i == 3 || i ==  4
        theta(i,3) = pi - phi_l_n - nu;
    elseif i == 5 || i ==  6
        theta(i,3) = pi - phi_r_p - nu;
    elseif i == 7 || i ==  8
        theta(i,3) = pi - phi_r_n - nu;
    end
end

if isreal(theta(1:8,3)) == 0
    reachable = 0;
    theta(:,3) = real(theta(:,3));
else
    reachable = 1;
end

%% determining the value of theta2

alfa = zeros(8,1);
beta = zeros(8,1);

for i = 1:8
    alfa(i) = a(2) + a(3)*cos(theta(i,3)) + d(4)*cos(theta(i,3)+pi/2);
    beta(i) = a(3)*sin(theta(i,3)) + d(4)*sin(theta(i,3)+pi/2);
    theta(i,2) = (atan2((alfa(i)*d15(2,:,i)-beta(i)*d15(1,:,i))/(alfa(i)^2+beta(i)^2),(alfa(1)*d15(1,:,i)+beta(i)*d15(2,:,i))/(alfa(i)^2+beta(i)^2)));
end
%% Determining the values for theta4, theta5, theta6
T03 = zeros(4,4,8);
for i = 1:8
    T03(:,:,i) = dhTrans(a(1:3),d(1:3),alpha(1:3),theta(i,1:3),0,3);
end
%Evaluating the value of R36
R36 = zeros(3,3,8);
for i = 1:8
    R36(:,:,i) = T03(1:3,1:3,i).'*T_des(1:3,1:3);
end

for i = 1:8
    if mod(i,2) == 1
        theta(i,5) = atan2(sqrt(R36(3,1,i)^2+R36(3,2,i)^2),R36(3,3,i));
    else
        theta(i,5) = atan2(-sqrt(R36(3,1,i)^2+R36(3,2,i)^2),R36(3,3,i));
    end
end
%determining the value of theta5
for i = 1:8
    if theta(i,5)==pi
        theta(i,4) = 0;
        theta(i,6)=atan2(-R36(1,2,i),R36(1,1,i));    
    else
        theta(i,6) = atan2(-R36(3,2,i)/sin(theta(i,5)),R36(3,1,i)/sin(theta(i,5)));
        theta(i,4) = atan2(-R36(2,3,i)/sin(theta(i,5)),-R36(1,3,i)/sin(theta(i,5)));
    end
end

for i = 1:8
    for j = 1:6
        if theta(i,j) >= pi
            theta(i,j) = theta(i,j) - 2*pi;
        end
    end
end

if exist('th_last','var') == 0
    th1 = theta(:,1);
    th2 = theta(:,2);
    th3 = theta(:,3);
    th4 = theta(:,4);
    th5 = theta(:,5);
    th6 = theta(:,6);
    return;
end
%% Selecting the path with the least cost function
diff = zeros(8,6);
costFunc = zeros(8,1);
th_last(2) = th_last(2) - pi/2;

for i = 1:8
    for j = 1:6
        diff(i,j) = abs(theta(i,j)-th_last(j));
        costFunc(i) = costFunc(i) + diff(i,j);
    end
end

[min_cost,branch] = min(costFunc);

th1 = theta(branch,1);
th2 = theta(branch,2) + pi/2;
th3 = theta(branch,3);
th4 = theta(branch,4);
th5 = theta(branch,5);
th6 = theta(branch,6);

end

