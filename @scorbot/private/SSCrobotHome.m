function [thetas,pitch,roll,gripper]=SSCrobotHome(scbconsts)
% return values for the D-H angles of the robot that set the robot at home

    thetas=[0 scbconsts.ang0shoulder -pi/2];
    pitch=-pi/4;
    roll=0;
    gripper=0;

end