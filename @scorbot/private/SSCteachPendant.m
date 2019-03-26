function [scbstate,newdisplay]=SSCteachPendant(scbconsts,display,world)
% Given the DISPLAY, allows the user to move the robot and return the state
% of the robot when finished. The world is not displayed while using this
% pendant.

    showhelp();
    displayaux=SSCpathOff(display);
    newdisplay=SSCupdateDisplay(displayaux,scbconsts,[],world);
    scbstate=newdisplay.lastscbstate;
    incmotor=40;
    incanglegripper=30;
    incanglegrippercart=3*pi/180;
    inccartesian=0.01;
    finish=0;
    while (~finish)
        w=waitforbuttonpress;
        if (w) % key, not mouse
            ch=get(newdisplay.handle,'CurrentCharacter');
            [motors,mpitch,mroll]=SSCthetasToMotors(scbconsts,scbstate.thetas,scbstate.pitch,scbstate.roll);
            [uTs,Q,uTg,posetip]=SSCdirectModel(scbconsts,scbstate.thetas,scbstate.pitch,scbstate.roll);
            newst=scbstate;
            newtip=posetip;
            updatedisp=1; % 0-> no update, 1->joint update, 2->cartesian update
            switch (ch)
                case 13 % finish
                    finish=1;
                    updatedisp=0;
                case 'h' % help
                    showhelp;
                    updatedisp=0;
                case 'H' % home
                    [thetas,pitch,roll,gripper]=SSCrobotHome(scbconsts);
                    [motors,mpitch,mroll]=SSCthetasToMotors(scbconsts,thetas,pitch,roll);
                case 'b' % base positive
                    motors(1)=motors(1)+incmotor;
                case 'B' % base negative
                    motors(1)=motors(1)-incmotor;
                case 's' % shoulder positive
                    motors(2)=motors(2)+incmotor;
                case 'S' % shoulder negative
                    motors(2)=motors(2)-incmotor;
                case 'e' % elbow positive
                    motors(3)=motors(3)+incmotor;
                case 'E' % elbow negative
                    motors(3)=motors(3)-incmotor;
                case 'p' % pitch positive
                    mpitch=mpitch+incanglegripper;
                case 'P' % pitch negative
                    mpitch=mpitch-incanglegripper;
                case 'r' % roll positive
                    mroll=mroll+incanglegripper;
                case 'R' % roll negative
                    mroll=mroll-incanglegripper;
                case 'i' % pitch positive (cartesian)
                    newtip(5)=newtip(5)+incanglegrippercart;
                    updatedisp=2;
                case 'I' % pitch negative (cartesian)
                    newtip(5)=newtip(5)-incanglegrippercart;
                    updatedisp=2;
                case 'g' % open/close the gripper
                    if (newst.gripper>0)
                        newst.gripper=0;
                    else
                        newst.gripper=100;
                    end
                case 't' % shows/hides tip orientation
                    if (newdisplay.tiporient)
                        newdisplay=SSCtiporientOff(newdisplay);
                    else
                        newdisplay=SSCtiporientOn(newdisplay);
                    end
                case 'x' % x axis positive
                    newtip(1)=newtip(1)+inccartesian;
                    updatedisp=2;
                case 'X' % x axis negative
                    newtip(1)=newtip(1)-inccartesian;
                    updatedisp=2;
                case 'y' % y axis positive
                    newtip(2)=newtip(2)+inccartesian;
                    updatedisp=2;
                case 'Y' % y axis negative
                    newtip(2)=newtip(2)-inccartesian;
                    updatedisp=2;
                case 'z' % z axis positive
                    newtip(3)=newtip(3)+inccartesian;
                    updatedisp=2;
                case 'Z' % Z axis negative
                    newtip(3)=newtip(3)-inccartesian;
                    updatedisp=2;
            end
            switch (updatedisp)
                case 1 % joint update
                    [thetas,pitch,roll]=SSCmotorsToThetas(scbconsts,motors,mpitch,mroll);
                    outofrange=SSCcheckThetasPR(scbconsts,thetas,pitch,roll);
                    if (~outofrange)
                        newst.thetas=thetas;
                        newst.pitch=pitch;
                        newst.roll=roll;
                        [world,errorchange]=SSCchangeWorldByRobot(scbconsts,scbstate,newst,world);
                        if (errorchange~=0)
                            fprintf('World error code: %d\n',errorchange);
                        end
                        scbstate=newst;
                        newdisplay=SSCupdateDisplay(newdisplay,scbconsts,scbstate,world);
                    end
                case 2 % cartesian update
                    [nosols,thetas,pitch,roll]=SSCinverseModel(scbconsts,newtip,scbstate.thetas);
                    if (nosols>0)
                        newst.thetas=thetas;
                        newst.pitch=pitch;
                        newst.roll=roll;
                        [world,errorchange]=SSCchangeWorldByRobot(scbconsts,scbstate,newst,world);
                        if (errorchange~=0)
                            fprintf('World error code: %d\n',errorchange);
                        end
                        scbstate=newst;
                        newdisplay=SSCupdateDisplay(newdisplay,scbconsts,scbstate,world);
                    end
            end
        end
    end

end

function showhelp()

    fprintf('Keyword map for the teach pendant (DO NOT KEEP ANY KEY PRESSED):\n\n');
    fprintf('\t[ENTER] -> finish the teach pendant process.\n');
    fprintf('\th     -> displays this help.\n');
    fprintf('\tH     -> homing.\n');
    fprintf('\tt     -> toggle tip of the gripper location and orientation.\n');
    fprintf('\tb,s,e -> positive motion in joints Base, Shoulder, Elbow.\n');
    fprintf('\tB,S,E -> negative motion in joints Base, Shoulder, Elbow.\n');
    fprintf('\tp,r   -> positive motion in Pitch, Roll (joint motion).\n');
    fprintf('\tP,R   -> negative motion in Pitch, Roll.\n');
    fprintf('\ti     -> positive motion in Pitch (cartesian).\n');
    fprintf('\tI     -> negative motion in Pitch (cartesian).\n');
    fprintf('\tx,y,z -> positive motion in cartesian X, Y, Z.\n');
    fprintf('\tX,Y,Z -> negative motion in cartesian X, Y, Z.\n');
    fprintf('\tg     -> toggle the gripper (open/close).\n');
    fprintf('\n');

end