function outofrange=SSCcheckThetasPR(scbconsts,thetas,pitch,roll)
% Check whether this set of thetas angles (D-H) in radians, and the pitch
% and roll (in radians) are inside the valid reachable ranges or not

    if (~isvector(thetas))||(length(thetas)~=3)||(~isreal(thetas))||(~isreal(pitch))||(~isreal(roll))
        error('Invalid thetas and gripper orientation');
    end

    [motors,mpitch,mroll]=SSCthetasToMotors(scbconsts,thetas,pitch,roll);
    outofrange=1;
        
    if (motors(1)<scbconsts.minjoints(1)/scbconsts.axis1resol)||...
       (motors(1)>scbconsts.maxjoints(1)/scbconsts.axis1resol)
        return;
    end
    
    if (motors(2)<scbconsts.minjoints(2)/scbconsts.axis2resol)||...
       (motors(2)>scbconsts.maxjoints(2)/scbconsts.axis2resol)
        return;
    end

    if (motors(3)<scbconsts.minjoints(3)/scbconsts.axis1resol)||...
       (motors(3)>scbconsts.maxjoints(3)/scbconsts.axis1resol)
        return;
    end
    
    if (mpitch<scbconsts.minjoints(4)*10)||...
       (mpitch>scbconsts.maxjoints(4)*10)
        return;
    end

    if (mroll<scbconsts.minjoints(5)*10)||...
       (mroll>scbconsts.maxjoints(5)*10)
        return;
    end
    
    outofrange=0;

end