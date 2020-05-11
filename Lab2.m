clear;
clc;

%% Modelo Phantom X Pincher 

L1 = 10.5; L2 = 10.5; L3 = 11.0;
theta1 = 0; theta2 = 0; theta3 = 0; theta4 = 0;

%         dh(thetai,      di,      ai-1,  alpha-1,   sigma, offset)

dh1(1,:)= [ theta1,        9.5,       0,         0,      0,      0];
dh1(2,:)= [ theta2,        0,       0,      pi/2,      0,      0];
dh1(3,:)= [ theta3,        0,      L1,         0,      0,      0];
dh1(4,:)= [ theta4,        0,      L2,         0,      0,      0];

% Eslabones
L(1) = Link(dh1(1,:), 'modified');
L(2) = Link(dh1(2,:), 'modified');
L(3) = Link(dh1(3,:), 'modified');
L(4) = Link(dh1(4,:), 'modified');

Phantom = SerialLink(L,'name','Phantom X Pincher');

Phantom.tool = transl(L3,0,0)*troty(pi/2);


%Phantom.teach();
%% FORWARD KINEMATICS
syms q1 q2 q3 q4 
format 
format compact
T=simplify(vpa(Phantom.fkine([q1 q2 q3 q4])));

%% paint and plot
for q1=0:0.3:pi/2
    Phantom.plot([ 0 0 0 0 ],'noa');
    pause(1)
end
for q2=0:0.1:pi/2
    Phantom.plot([q1 q2 0 0 ],'noa');
    pause(0.2)
end
for q3=0:0.1:pi/2
    Phantom.plot([q1 q2 q3 0 ],'noa');
    pause(0.2)
end
for q4=0:0.1:pi/2
    Phantom.plot([q1 q2 q3 q4 ],'noa');
    pause(0.2)
end

%% Conexion Matlab y Ros
rosinit  
rostopic list;  
rostopic info /phantom/my_values/joint_states; 
suscriptor=rossubscriber("/joint_states","sensor_msgs/JointState"); 
publicador=rospublisher('/phantom/my_values/joint_states','sensor_msgs/JointState');
%% Matlab + PeterCorke + ROS

c1 = [0 0 0 0];
c2 = [-20 -20 -20 -20]*pi/180;
c3 = [30 -30 30 -30]*pi/180;
c4 = [-90 15 -55 17]*pi/180;
c5 = [-90 45 -55 45]*pi/180;

%Se grafica en el toolbox de Peter corke
figure(1)
Phantom.plot([c2(1) c2(2) c2(3) c2(4)]);
axis([-45 45 -45 45 -45 45])

%Se envia el mensaje a ROS
msg= rosmessage('sensor_msgs/JointState');
nombres=["j1","j3","j10","j17"];
for i=1:4
    msg.Name=nombres(i)
    msg.Position=c2(i);
    msg.Velocity=0.5;
    send(publicador,msg)
end
msg2=receive(suscriptor,10);
msg2.Position(1:4)
rosshutdown
