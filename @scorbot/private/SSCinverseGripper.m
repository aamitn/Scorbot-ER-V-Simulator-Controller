function solutions=SSCinverseGripper(scbconsts,posetip)
% Given the scorbot consts, and the pose in the universal frame of the tip 
% of the gripper in POSETIP (see SSCdirectGripper()),  
% calculate the position x,y,z in the universal frame
% of the origin of the gripper frame (Q) and also the orientations THETA1 of
% the X axis of the gripper frame w.r.t. the x axis of the universal frame
% and the pitch and roll (everything as SSCdirectGripper()).
% SOLUTIONS is a cell with as many elements as solutions found (from 0 to
% 2), each one being a struct with fields theta1,pitch,roll,Q.

    solutions={};

    pitch=posetip(5);
    roll=posetip(6); 

    [thetas,p,r,gripper]=SSCrobotHome(scbconsts); % just to obtain some values of theta2 and theta3 within range
    
    % first solution
    if (posetip(1)==0)&&(posetip(2)==0) % tip is in the vertical of universal z
        theta1=posetip(4); % theta1 is the same as given in the pose, because it can be any value (any value is valid)
    else
        theta1=atan2(posetip(2),posetip(1)); % theta1 orients the body to reach the tip point
    end
    outofrange=SSCcheckThetasPR(scbconsts,[theta1 thetas(2) thetas(3)],pitch,roll);
    if (~outofrange)
        solutions=addSolution(solutions,scbconsts,theta1,pitch,roll,posetip);                        
    end
    
    % second solution
    theta1=complementaryAngle(theta1);
    outofrange=SSCcheckThetasPR(scbconsts,[theta1 thetas(2) thetas(3)],pitch,roll);
    if (~outofrange)
        solutions=addSolution(solutions,scbconsts,theta1,pitch,roll,posetip);                        
    end    
    
end

function newsols=addSolution(solutions,scbconsts,theta1,pitch,roll,posetip)
% add the given solution to the existing ones (it does not check whether it
% is reachable with the motors)

    cy=cos(posetip(4));%theta1);
    sy=sin(posetip(4));%theta1);
    Ryaw=[cy -sy 0 ; sy cy 0 ; 0 0 1];
    cp=cos(-pitch);
    sp=sin(-pitch);
    Rpitch=[cp 0 sp ; 0 1 0 ; -sp 0 cp];
    u=Ryaw*Rpitch*[1;0;0];
    Q=posetip(1:3)'-u*scbconsts.lgripper;

    newsols=solutions;
    newsols{length(solutions)+1}=struct('theta1',theta1,...
                                        'pitch',pitch,...
                                        'roll',roll,...
                                        'Q',Q);

end

function angle=complementaryAngle(a)
% Return the complementary angle to A for the base of the scorbot (theta1)

    if (a<-2*pi)||(a>2*pi)
        error('Invalid A');
    end
    
    if (a==0)
        angle=pi;
    elseif (a>0)
        angle=a-pi;
    else
        angle=a+pi;
    end

end