clear all
rosinit
rostopic list;
rostopic info /phantom/my_values/joint_states;
suscriptor=rossubscriber("/joint_states","sensor_msgs/JointState");
publicador=rospublisher('/phantom/my_values/joint_states','sensor_msgs/JointState');
msg= rosmessage('sensor_msgs/JointState');
nombres=["j1","j3","j10","j17"];
rotacion = [pi -pi/4 pi/2 pi/3];
for i=1:4
    msg.Name=nombres(i)
    msg.Position=rotacion(i);
    msg.Velocity=0.5;
    send(publicador,msg)
end
rosshutdown