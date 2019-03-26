function [uTg,posetip]=SSCdirectGripper(scbconsts,Q,theta1,pitch,roll)
% Given yaw (theta1), pitch and roll (in radians w.r.t. univ) as defined for the 
% gripper of the
% Scorbot (see gripper_as_solid.png), and the cartesian position Q of the
% end point of the forearm of
% the robot in space (in the universal frame), return the homogeneus
% transform uTg that translate from points in the gripper egocentric frame to
% the universal frame, and also the position x,y,z,yaw,pitch,roll of the 
% tip of the grip into POSETIP, w.r.t. the universal, with the yaw being
% calculated for the orientation of the gripper in the universal frame


    % rotate according to pitch, roll and yaw
    c1=cos(theta1);
    s1=sin(theta1);
    cr=cos(roll);
    sr=sin(roll);
    cp=cos(pitch);
    sp=sin(pitch);
    
    Rtheta1=[c1 -s1 0 ; s1 c1 0 ; 0 0 1];
    Rroll=[1 0 0 ; 0 cr sr ; 0 -sr cr];
    Rpitch=[cp 0 -sp ; 0 1 0 ; sp 0 cp];
    
    % do the transformation
    R=Rtheta1*Rpitch*Rroll; 
    uTg=[R Q(1:3) ; 0 0 0 1];

    % tip of the grip
    p=uTg*[scbconsts.lgripper;0;0;1]; % tip of the gripper w.r.t. universal
    % unused: v=p(1:3)-Q; % vector pointing as the gripper
    tipyaw=theta1; %atan2(v(2),v(1));
    tippitch=pitch;
    tiproll=roll;
    posetip=[p(1) p(2) p(3) tipyaw tippitch tiproll];

end
