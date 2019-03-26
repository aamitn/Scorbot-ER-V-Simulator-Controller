function solutions=SSCinverseArm(scbconsts,Q,theta1)
% Given in SCBCONSTS the constants that define the scorbot, in Q the
% x,y,z position in the universal frame of the end of the third link (can
% be obtained through SSCinverseGripper()), in THETA1 the angle of the base of
% the scorbot (also obtainable through SSCinverseGripper()), return in 
% SOLUTIONS a cell with as many
% arrays of thetas (3 angles per each) as solutions it finds for placing
% the arm at Q, or an empty cell if no solution is found.

    solutions={};
 
    % calculate the origin of frame 2 in the universal frame - see cininv2.png
    t1T2=SSClinkT(2,scbconsts,[theta1 0 0]); % we do not care here about any angle but theta(1); in particular, we only use the origin of the frame 2, not its orientation
    t0T1=SSClinkT(1,scbconsts,[theta1 0 0]);
    tuT0=SSClinkT(0,scbconsts,[theta1 0 0]);
    tuT2=tuT0*t0T1*t1T2;
    P=tuT2*[0;0;0;1];
    P=P(1:3); 
    
    % calculate the origin of frame 1 in the univesal frame 
    W=tuT0*t0T1*[0;0;0;1];
    W=W(1:3);
        
    % rotate P-Q-W points around the universal z to null the theta1 angle;
    % after that, the coordinates x-y of the points are the same as their
    % coordinates on a vertical plane where we are going to work
    c=cos(-theta1);
    s=sin(-theta1);
    rot=[c -s 0 ; c s 0 ; 0 0 1];
    rotQ=rot*Q;
    rotP=rot*P;
    rotW=rot*W;
    
    % If P is the origin of that plane, set Q relative to it. Then, theta2
    % will be measured counterclockwise from the x axis of the plane
    Qp=rotQ-rotP;
    Qp=[Qp(1);Qp(3)]; % not interested on the universal y coordinate, that should be 0 anyway at this point
    angKp=atan2(Qp(2),Qp(1)); % from plane x axis to k line; positive or negative
    k=norm(Qp); % distance from P to Q
    if (k==0)
        return; % unreachable
    end
    if (k>=scbconsts.l2+scbconsts.l3)
        return; % unreachable
    end
    
    % calculate angles 2 and 3 of the triangle P-elbow-Q on the plane - see
    % cininv2.png
    solutions=changeAngleInAllSolutions(solutions,1,theta1);

    gamma2=acos((scbconsts.l2^2+k^2-scbconsts.l3^2)/(2*scbconsts.l2*k)); % negative argument yields towards pi; positive, towards 0
    gamma3=acos((scbconsts.l3^2+scbconsts.l2^2-k^2)/(2*scbconsts.l2*scbconsts.l3));

    if (Qp(2)>=0) % P-Q line in the first/second quadrants of the triangle plane; angKp positive

        % elbow (point R in the triangle) to the left of the line P-Q
        theta2=angKp+gamma2;
        theta3=pi+gamma3;
        sols1=changeAngleInAllSolutions(solutions,3,theta3);
        sols1=addAngleToAllSolutions(sols1,3,complementaryAngle(theta3));        
        sols1=changeAngleInAllSolutions(sols1,2,theta2);
        sols1=addAngleToAllSolutions(sols1,2,complementaryAngle(theta2));

        % elbow to the right of the line P-Q
        theta2=angKp-gamma2;
        theta3=pi-gamma3;
        sols2=changeAngleInAllSolutions(solutions,3,theta3);
        sols2=addAngleToAllSolutions(sols2,3,complementaryAngle(theta3));        
        sols2=changeAngleInAllSolutions(sols2,2,theta2);
        sols2=addAngleToAllSolutions(sols2,2,complementaryAngle(theta2));

        solutions=mergeSolutions(sols1,sols2);

    else % P-Q line in the third/fourth quadrant of the triangle plane; angKp negative

        % elbow (point R in the triangle) to the left of the line P-Q
        theta2=angKp-gamma2;
        theta3=pi-gamma3;
        sols1=changeAngleInAllSolutions(solutions,3,theta3);
        sols1=addAngleToAllSolutions(sols1,3,complementaryAngle(theta3));        
        sols1=changeAngleInAllSolutions(sols1,2,theta2);
        sols1=addAngleToAllSolutions(sols1,2,complementaryAngle(theta2));

        % elbow to the right of the line P-Q
        theta2=angKp+gamma2;
        theta3=pi+gamma3;
        sols2=changeAngleInAllSolutions(solutions,3,theta3);
        sols2=addAngleToAllSolutions(sols2,3,complementaryAngle(theta3));        
        sols2=changeAngleInAllSolutions(sols2,2,theta2);
        sols2=addAngleToAllSolutions(sols2,2,complementaryAngle(theta2));

        solutions=mergeSolutions(sols1,sols2);

    end
    
    % drop out-of-limit solutions
    n=length(solutions);
    invalid=zeros(1,n);
    for (f=1:n)
        outofrange=SSCcheckThetasPR(scbconsts,solutions{f},0,0); % we do not care about pitch and roll, thus we use values that are within range
        if (outofrange)
            invalid(f)=1;
        end
    end
    solutions=dropSolutions(solutions,invalid);

end


function drawtriangle(Q,R,fmt,lab)
% Just while debugging

    plot([0 R(1) Q(1)],[0 R(2) Q(2)],fmt);
    text(R(1),R(2),lab);

end

function newsols=changeAngleInAllSolutions(solutions,indangle,angle)
% Put ANGLE into the index INDANGLE of all existing SOLUTIONS, or create a
% new solution with that angle and the rest at 0.

    n=length(solutions);
    if (n==0)
        s=zeros(1,3);
        s(indangle)=angle;
        newsols={};
        newsols{1}=s;
    else
        newsols=solutions;
        for (f=1:n)
            newsols{f}(indangle)=angle;
        end
    end

end

function newsols=addAngleToAllSolutions(solutions,indangle,angle)
% Takes all existing SOLUTIONS and create as many new solutions as
% solutions exist already, the new ones containing ANGLE in the INDANGLE
% index. If there is no solutions yet, create only one with that value of
% angle and the rest angles equalling 0

    n=length(solutions);
    if (n==0)
        newsols=changeAngleInAllSolutions(solutions,indangle,angle);
    else
        newsols=solutions;
        for (f=1:n)
            newsols{n+f}=solutions{f};
            newsols{n+f}(indangle)=angle;
        end
    end

end

function newsols=mergeSolutions(sols1,sols2)
% Concatenate both solutions into only one

    n1=length(sols1);
    n2=length(sols2);
    newsols=sols1;
    for (f=1:n2)
        newsols{n1+f}=sols2{f};
    end

end

function newsols=dropSolutions(solutions,indexestodrop)
% Return a new set of solutions without those that have the given indexes

    newsols={};
    n=length(solutions);
    if (n~=length(indexestodrop))
        error('Length of indexes does not match with length of solutions');
    end
    
    ind=1;
    for (f=1:n)
        if (indexestodrop(f)==0)
            newsols{ind}=solutions{f};
            ind=ind+1;
        end
    end

end

function angle=complementaryAngle(a)
% Return the complementary angle to A (the one at +-pi from A)

    if (a<-2*pi)||(a>2*pi)
        error('Invalid A');
    end
    
    if (a==0)
        angle=-2*pi;
    elseif (a>0)
        angle=-(2*pi-a);
    else
        angle=2*pi+a;
    end

end