function display=SSCupdateDisplay(olddisplay,scbconsts,scbstate,world)
% Update the display with the given status of the robot and its world
% The state of the robot is a struct with the following elements:
%   thetas -> a vector with the three D-H thetas for the joints 1-3
%   pitch -> the D-H value for the pitch of the gripper
%   roll -> the D-H value for the roll of the gripper
%   gripper -> the percentage of opening of the gripper (0-100)
% WORLD is a cell with as many elements as pieces exist in the world. Each
% piece is a prismatic one, and is defined by this structure:
%   xsize,ysize,zsize -> sizes along the three axis of the frame of the
%   piece
%   T -> transform for the position and orientation of the piece in space
%   w.r.t. the universal frame, that is, univ_T_piece.
%   c -> color of the piece (just a character)
% If SCBSTATE is empty, the display is only refreshed (if there is a
% previous state on it; otherwise, nothing is done)

    display=olddisplay;
    if (display.justcreated)
        display.view_az=75;
        display.view_el=20;
        display.justcreated=0;
    else
        [display.view_az,display.view_el]=view; % just for the case the user has manipulated the view
    end
    
    if (~isempty(scbstate))
        display.lastscbstate=scbstate;
    end    
    
    if (~isempty(display.lastscbstate))
        outofrange=SSCcheckThetasPR(scbconsts,display.lastscbstate.thetas,display.lastscbstate.pitch,display.lastscbstate.roll);
        if (~outofrange)
            [uTs,Q,uTg,posetip]=SSCdirectModel(scbconsts,display.lastscbstate.thetas,display.lastscbstate.pitch,display.lastscbstate.roll);
            if (display.pathon)
                display.path=[display.path ; posetip(1:3)];
            end
        end
    end
    
    figure(display.handle);

    if (~isempty(display.lastscbstate))
        
        % limits of safe workspace
        hold off;
        plot3(0,0,0,'.');
        rectangle('Position',[-0.7 -0.7 1.4 1.4],'Curvature',[1 1]);
        hold on;
        
        % axes appearance
        axis equal;
        axis([-0.7 0.7 -0.7 0.7 0 0.85]);
        grid;
        
        % world elements
        if (~isempty(world))
            n=length(world.pieces);
            for (f=1:n)
                piece=world.pieces{f};
                if (f==world.piecetaken)
                    piece.T=uTg;
                end
                drawpiece(piece,sprintf('%d',f));
            end
        end

        if (~outofrange)
            % past path
            if (display.pathon)
                plot3(display.path(:,1),display.path(:,2),display.path(:,3),'m-');
                plot3(display.path(1,1),display.path(1,2),display.path(1,3),'mo');
            end
            % robot
            SSCdrawScorbot(scbconsts,uTs,uTg,Q,posetip,display.lastscbstate.gripper,display.tiporient,'b',zeros(1,4));
            
            ti1 = sprintf('joints: th1=%3d, th2=%3d, th3=%3d, pitch=%3d, roll=%3d',...
                   round(display.lastscbstate.thetas(1)*180/pi),round(display.lastscbstate.thetas(2)*180/pi),round(display.lastscbstate.thetas(3)*180/pi),...
                   round(display.lastscbstate.pitch*180/pi),round(display.lastscbstate.roll*180/pi));
            ti2 = sprintf('cartesian: x=%.3f y=%.3f z=%.3f yaw=%3d pitch=%3d roll=%3d',...
                   posetip(1),posetip(2),posetip(3),...
                   round(posetip(4)*180/pi),...
                   round(posetip(5)*180/pi),...
                   round(posetip(6)*180/pi));
            title(ti2);%{ti1,ti2});
        else
            title('Position out of valid range');
        end

        % set view perspective and update drawing
        view(display.view_az,display.view_el);
        drawnow;
        
    end

end

function drawpiece(piece,txt)

    x1=piece.xsize/2;
    y1=piece.ysize/2;
    z1=piece.zsize/2;
    
    pol0=[x1 -y1 z1 ; x1 y1 z1 ; x1 y1 -z1 ; x1 -y1 -z1];
    pol1=[x1 -y1 z1 ; -x1 -y1 z1 ; -x1 -y1 -z1 ; x1 -y1 -z1];
    pol2=[-x1 -y1 z1 ; -x1 y1 z1 ; -x1 y1 -z1 ; -x1 -y1 -z1];
    pol3=[-x1 y1 z1 ; x1 y1 z1 ; x1 y1 -z1 ; -x1 y1 -z1];
    
    drawpolygon(piece.T,pol0,piece.c);
    drawpolygon(piece.T,pol1,piece.c);
    drawpolygon(piece.T,pol2,piece.c);
    drawpolygon(piece.T,pol3,piece.c);
    
    if (~isempty(txt))
        p=piece.T*[0;0;0; 1];
        text(p(1),p(2),p(3),txt,'Color',piece.c);
    end

end

function drawpolygon(univT,pol,col)

    [npts,~]=size(pol);
    for (f=1:npts)
        p=univT*[pol(f,1:3)' ; 1];
        pol(f,1:3)=p(1:3);
    end
    
    pol=pol';
    xs=pol(1,:);
    ys=pol(2,:);
    zs=pol(3,:);
    
    plot3([xs(:);xs(1)],[ys(:);ys(1)],[zs(:);zs(1)],sprintf('%c-',col));

end