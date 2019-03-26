function [motors,mpitch,mroll]=SSCthetasToMotors(scbconsts,thetas,pitch,roll)
% The inverse of SSCmotorToThetas()

    mpitch=pitch*180/pi*10; % notice that the pitch must follow the universal reference and not the elbow one; therefore, the device receiving this value must interpret w.r.t. the universal
    mroll=roll*180/pi*10;
    
    motors=[thetas(1)*180/pi/scbconsts.axis1resol,...                          
            (scbconsts.ang0shoulder-thetas(2))*180/pi/scbconsts.axis2resol,... % here we make corrections for the elbow to follow the universal reference instead of the shoulder one
            (thetas(2)+thetas(3))*180/pi/scbconsts.axis3resol]; % this depends on two angles because in motor space it is measured w.r.t. the universal frame and not w.r.t. the previous robot link
        
end 
