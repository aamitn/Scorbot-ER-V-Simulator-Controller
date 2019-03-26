function [thetas,pitch,roll]=SSCmotorsToThetas(scbconsts,motors,mpitch,mroll)
% Given in SCBCONSTS the definitions of the robot and in MOTORS-MPITCH-MROLL a vector of
% joint motor commands as displayed and used in the positions of the real
% Scorbot, return a vector of thetas and PITCH-ROLL for using the kinematic model.
% MOTORS is a vector of 3 elements, being the encoder readings of axes
% 1-3, with a resolution of scbconsts.axis1resol, scbconsts.axis2resol and
% scbconsts.axis3resol parts per degree (Note: in motor space, the elbow angle is
% always measured w.r.t. the universal frame -plane xy- and not w.r.t. the 
% shoulder link).
% MPTICH and MROLL are tenths of degrees and measured as in the real robot
% THETAS are defined as in angulosDH1/2.jpg. 
% PITCH and ROLL are as defined in SSCdirectGripper()

    if (~isvector(motors))||(length(motors)~=3)||(~isreal(motors))||(~isreal(mpitch))||(~isreal(mroll))
        error('Invalid motors values');
    end
    
    pitch=mpitch/10*pi/180;
    roll=mroll/10*pi/180;
    
    th1=motors(1)*scbconsts.axis1resol*pi/180;
    th2=scbconsts.ang0shoulder-motors(2)*scbconsts.axis2resol*pi/180;
    th3=motors(3)*scbconsts.axis3resol*pi/180-th2;
    thetas=[th1 th2 th3];
    
end     