function path=SSCdefinePath(scbconsts,resol,thetas0,pitch0,roll0,thetas1,pitch1,roll1)
% Return a sequence of joint poses (thetas+pitch+roll) for the robot that
% move it from 0 to 1 with a resolution of RESOL radians in the angle that
% requires more steps to reach the target. The path interpolates linearly
% all the angles in order to reach the target simultaneously
% THETAS* are in radians as in the D-H model; PITCH* and ROLL* are in
% radias as specified for the gripper.

    if (~isreal(resol)) || (resol<=0)
        error('Invalid resolution');
    end
    diffths=[thetas1-thetas0 pitch1-pitch0 roll1-roll0];
    maxdiff=max(abs(diffths));
    if (maxdiff<=resol)
        path={ [thetas0 pitch0 roll0], [thetas1 pitch1 roll1] };
    else
        nsteps=floor(maxdiff/resol);
        incs=diffths/nsteps;
        path={ [thetas0 pitch0 roll0] };
        for (f=1:nsteps)
            path{f+1}=[thetas0 pitch0 roll0]+incs*f;
        end
    end

end
