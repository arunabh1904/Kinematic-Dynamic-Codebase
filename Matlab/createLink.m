% createLink: Creates a structure with the input given below
%
% L = createLink(a,d,alpha,theta,centOfMass,mass,inertia) creates a
% structure L the following input elements.
% 
% output = L( Structure) 
%
% a – DH parameter a (meters)
% d – DH parameter d (meters)
% alpha – DH parameter alpha (radians)
% theta – DH parameter theta (radians)
% mass – link mass (kg)
% inertia – link mass moment of inertia (kg m^2)
% com – the position of the link’s center of mass
% isRotary – Boolean true if it is a rotary joint false if it is a prismatic joint.


function L = createLink(a,d,alpha,theta,centOfMass,mass,inertia)
%% Initializing isRotary to Zero
isRotary=0;
L.a=a;
L.d=d;
L.alpha=alpha;
L.theta=theta;
L.com=centOfMass;
L.mass=mass;
L.inertia=inertia;
%% if theta=0 then its prismatic joint else rotary and based upon that pass empty array
if theta==0
    isRotary==0;
    L.d=[];

else
    isRotary==1;
    L.theta=[];
end

