function [newworld,errorchange]=SSCchangeWorldByRobot(scbconsts,scboldstate,scbnewstate,world)
% Given the old state of the robot and the new one (still not used), check
% whether there are pieces in the world that need to change due to grasping,
% dropping, etc., and return the new world in any case.
% In ERRORCHANGE return an error code indicating any anomaly:
%   0-> no anomaly
%   1-> more than one piece in the radius of grasping

    newworld=world;
    errorchange=0;
    if (isempty(world))
        return;
    end
    
    if (newworld.piecetaken>0) % already taken a piece
    else % no piece taken yet
        if (sum(scbnewstate.thetas==scboldstate.thetas)==3) && ...
           (scbnewstate.pitch==scboldstate.pitch) && ...
           (scbnewstate.roll==scboldstate.roll) && ...
           (scboldstate.gripper==100) && (scbnewstate.gripper<100) % want to take
            % calculate coordinates of the gripper and tip
            outofrange=SSCcheckThetasPR(scbconsts,scbnewstate.thetas,scbnewstate.pitch,scbnewstate.roll);
            if (outofrange)
                error('New state is out of range');
            end
            [uTs,Q]=SSCdirectArm(scbconsts,scbnewstate.thetas);
            [uTg,posetip]=SSCdirectGripper(scbconsts,Q,scbnewstate.thetas(1),scbnewstate.pitch,scbnewstate.roll);
            % check whether the gripper is close enough to any piece
            n=length(newworld.pieces);
            taken=[];
            for (f=1:n)
                pc=newworld.pieces{f}.T*[0;0;0;1];
                dist=sqrt((pc(1)-posetip(1))^2+(pc(2)-posetip(2))^2+(pc(3)-posetip(3))^2);
                if (dist<max([newworld.pieces{f}.xsize newworld.pieces{f}.ysize newworld.pieces{f}.zsize]))
                    taken=[taken ; f dist];
                end
            end
            if (~isempty(taken)) % a piece is taken
                [nps,aux]=size(taken);
                if (nps>1)
                    errorchange=1;
                    return;
                end
                newworld.piecetaken=taken(1,1);
            end
        end
    end
    
end