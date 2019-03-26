function [nosols,thetas,pitch,roll]=SSCinverseModel(scbconsts,posetip,orientthetas)
% Return the best solution to set the gripper as in POSETIP (see
% SSCdirectModel()). ORIENTTHETAS may be empty (then this function chooses the
% best solution on its own) or have three values for theta1-3 (the this
% function chooses the solution that is closest to it).
% NOTE: the motors corresponding to the returned thetas are guaranteed to
% be within range.
%
% NOSOLS: number of solutions found, or 0 if none (in that case, the rest 
% of outputs are filled with nothing)
% THETAS: the three angles of the arm (rads, as in D-H)
% PITCH,ROLL: the angles of the gripper (rads, w.r.t. the univ. frame)
    
    nosols=0;
    thetas=[];
    pitch=0;
    roll=0;

    % inverse gripper model
    solsg=SSCinverseGripper(scbconsts,posetip);
    ng=length(solsg);
    if (ng==0)
        return;
    end
    
    % inverse arm model for each inverse gripper model
    solsa={};
    for (f=1:ng)
        solsf=SSCinverseArm(scbconsts,solsg{f}.Q,solsg{f}.theta1);
        nf=length(solsf);
        for (k=1:nf)
            solsa{length(solsa)+1}=struct('thetas',solsf{k},...
                                          'pitch',solsg{f}.pitch,...
                                          'roll',solsg{f}.roll);
        end
    end
    
    % chooses best solution
    nosols=length(solsa);
    if (nosols<1)
        return
    end
    if (nosols==1)||(length(orientthetas)~=3) % if only 1 solution or we have no orienthetas, take the first
        thetas=solsa{1}.thetas;
        pitch=solsa{1}.pitch;
        roll=solsa{1}.roll;
        return;
    end    
    dists=zeros(1,nosols);
    for (f=1:dists)
        dists(f)=norm(solsa{f}.thetas,orientthetas);
    end
    [mindist,indmindist]=min(dists);
    thetas=solsa{indmindist}.thetas;
    pitch=solsa{indmindist}.pitch;
    roll=solsa{indmindist}.roll;

end
