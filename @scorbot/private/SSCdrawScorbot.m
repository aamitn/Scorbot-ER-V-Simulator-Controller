function SSCdrawScorbot(scbconsts,uTs,uTg,Q,posetip,gripper,tiporientation,c,frames)
% Given the current direct model of the scorbot in uTS, Q (like in SSCdirectArm())
% uTg, posetip (like in SSCdirectGripper()), and the
% current state of the gripper, calculate the cartesian poses of the
% scorbot and draw it in the current figure.
% SCBCONSTS is as in SSCdirectArm()
% GRIPPER is in [0,100], indicating the percentage of opening
% If TIPORIENTATION is 1, shows the orientation of the tip
% C is a one-letter color character for drawing
% FRAMES is 1x4. It draws the reference
% frames of the indexes of FRAMES that contain non-zero, being 1-3 the
% frames of links 1-3 and 4 the frame of the gripper (the universal frame
% is always drawn).

    hold on;
    
    % frames
    lv=0.3;
    drawvector([0 0 0],[lv 0 0],'r','','xu');
    drawvector([0 0 0],[0 lv 0],'c','','yu');
    drawvector([0 0 0],[0 0 lv],'g','','zu');
    lv=0.1;
    for (f=1:3)
        if (frames(f)~=0)
            p0=[0 0 0 1]';
            px=[lv 0 0 1]';
            py=[0 lv 0 1]';
            pz=[0 0 lv 1]';
            up0=uTs{f+1}*p0;
            upx=uTs{f+1}*px;
            upy=uTs{f+1}*py;
            upz=uTs{f+1}*pz;
            drawvector(up0,upx,'r','',sprintf('x%d',f));
            drawvector(up0,upy,'c','',sprintf('y%d',f));
            drawvector(up0,upz,'g','',sprintf('z%d',f));
        end
    end
    if (frames(4)~=0)
        px=[lv 0 0 1]';
        py=[0 lv 0 1]';
        pz=[0 0 lv 1]';
        upx=uTg*px;
        upy=uTg*py;
        upz=uTg*pz;
        drawvector(Q,upx,'r','','xg');
        drawvector(Q,upy,'c','','yg');
        drawvector(Q,upz,'g','','zg');        
    end
    
    % tip of gripper
    if (tiporientation)
        p1=[scbconsts.lgripper+0.05;0;0;1];
        up1=uTg*p1;
        drawvector(posetip(1:3),up1,'m','','');
        plot3(posetip(1),posetip(2),posetip(3),'m*');
        plot3(posetip(1),posetip(2),0,'m.');
    end
    
    % robot

    % base
    x1=scbconsts.baseradius;
    y1=scbconsts.baseradius;
    z1=scbconsts.l0;
    z2=scbconsts.l0-scbconsts.baseheight;
    p00=[x1 y1 -z1 ; ...
        x1 y1 -z2 ; ...
        -x1 y1 -z2 ; ...
        -x1 y1 -z1];
    p01=[-x1 y1 -z1 ; ...
        -x1 y1 -z2 ; ...
        -x1 -y1 -z2;
        -x1 -y1 -z1];
    p02=[-x1 -y1 -z1 ; ...
        -x1 -y1 -z2 ;...
        x1 -y1 -z2 ; ...
        x1 -y1 -z1];
    p03=[x1 -y1 -z1 ;...
        x1 -y1 -z2 ;...
        x1 y1 -z2 ;...
        x1 y1 -z1];
    drawpolygon(uTs{1},p00,'b');
    drawpolygon(uTs{1},p01,'b');
    drawpolygon(uTs{1},p02,'b');
    drawpolygon(uTs{1},p03,'b');

    % body
    x1=scbconsts.bodydepth/2;
    y1=scbconsts.bodywidth/2;
    z1=scbconsts.bodyheight;
    z2=0;
    p10=[x1 -y1 -z1 ; ...
         x1 -y1 z2 ; ...
         -x1 -y1 z2 ; ...
         -x1 -y1 -z1];
    p11=[-x1 -y1 -z1 ; ...
         -x1 -y1 z2 ; ...
         -x1 y1 z2 ; ...
         -x1 y1 -z1];
    p12=[-x1 y1 -z1 ; ...
         -x1 y1 z2 ; ...
         x1 y1 z2 ; ...
         x1 y1 -z1];
    p13=[x1 y1 -z1 ;...
         x1 y1 z2 ; ...
         x1 -y1 z2 ; ...
         x1 -y1 -z1];
    drawpolygon(uTs{2},p10,'b');
    drawpolygon(uTs{2},p11,'b');
    drawpolygon(uTs{2},p12,'b');
    drawpolygon(uTs{2},p13,'b');

    % arm
    x1=scbconsts.armwidth/2;
    x2=scbconsts.l2+scbconsts.armwidth/2;
    y1=scbconsts.armwidth/2;
    z1=scbconsts.bodywidth/2;
    p20=[-x1 y1 z1 ; ...
         -x1 -y1 z1 ; ...
         x2 -y1 z1 ; ...
         x2 y1 z1 ];
    p21=[-x1 y1 -z1 ; ...
         -x1 -y1 -z1 ; ...
         x2 -y1 -z1 ; ...
         x2 y1 -z1 ];
    drawpolygon(uTs{3},p20,'b');
    drawpolygon(uTs{3},p21,'b');

    % forearm
    x1=scbconsts.armwidth/2;
    x2=scbconsts.l3+scbconsts.armwidth/2;
    y1=scbconsts.armwidth/2;
    z1=scbconsts.bodywidth/2;
    p30=[-x1 y1 z1 ; ...
         -x1 -y1 z1 ; ...
         x2 -y1 z1 ; ...
         x2 y1 z1 ];
    p31=[-x1 y1 -z1 ; ...
         -x1 -y1 -z1 ; ...
         x2 -y1 -z1 ; ...
         x2 y1 -z1 ];
    drawpolygon(uTs{4},p30,'b');
    drawpolygon(uTs{4},p31,'b');

    % pitch/roll supporting bar            
    x1=0.005; %scbconsts.armwidth/2;
    y1=0.005; %scbconsts.armwidth/2;
    z1=scbconsts.bodywidth/2;
    p40=[-x1 y1 z1 ; ...
         -x1 y1 -z1 ; ...
         -x1 -y1 -z1 ; ...
         -x1 -y1 z1];
    p41=[-x1 -y1 z1 ; ...
         x1 -y1 z1 ; ...
         x1 -y1 -z1 ; ...
         -x1 -y1 -z1];
    p42=[x1 y1 z1 ; ...
         x1 y1 -z1 ; ...
         x1 -y1 -z1 ; ...
         x1 -y1 z1];
    p43=[-x1 y1 z1 ; ...
         x1 y1 z1 ; ...
         x1 y1 -z1 ; ...
         -x1 y1 -z1];
    moff=zeros(4,3);
    moff(:,1)=scbconsts.l3;

    drawpolygon(uTs{4},p40+moff,'b');
    drawpolygon(uTs{4},p41+moff,'b');
    drawpolygon(uTs{4},p42+moff,'b');
    drawpolygon(uTs{4},p43+moff,'b');

    % rigid part of the gripper
    y1=scbconsts.hgripper;
    z1=scbconsts.wgripper/2;
    x1=scbconsts.wgripper/2;
    x2=scbconsts.lgripper-scbconsts.l2gripper;
    p50=[-x1 y1 z1; ...
         x2 y1 z1; ...
         x2 y1 -z1; ...
         -x1 y1 -z1 ];
    p51=[-x1 y1 -z1 ; ...
         -x1 -y1 -z1 ; ...
         x2 -y1 -z1; ...
         x2 y1 -z1];
    p52=[-x1 -y1 z1 ; ...
         x2 -y1 z1; ...
         x2 -y1 -z1; ...
         -x1 -y1 -z1 ];
    p53=[-x1 y1 z1 ; ...
         -x1 -y1 z1 ; ...
         x2 -y1 z1; ...
         x2 y1 z1];

    % p6 and p7 are the tips of the gripper: the x coordinates range from
    % +/-(scbconsts.hgripper+scbconsts.wgripper) for the open
    % gripper to 0 for the closed gripper
    y1=scbconsts.hgripper*gripper/100+scbconsts.wgripper;
    y2=scbconsts.hgripper*gripper/100;
    z1=scbconsts.wgripper/2;
    x1=scbconsts.lgripper-scbconsts.l2gripper;
    x2=scbconsts.lgripper;
    p60=[x1 y1 z1 ; ...
         x2 y1 z1; ...
         x2 y1 -z1; ...
         x1 y1 -z1];
    p61=[x1 y1 -z1; ...
         x1 y2 -z1; ...
         x2 y2 -z1; ...
         x2 y1 -z1];
    p62=[x1 y2 z1; ...
         x2 y2 z1; ...
         x2 y2 -z1; ...
         x1 y2 -z1];
    p63=[x1 y1 z1; ...
         x1 y2 z1; ...
         x2 y2 z1; ...
         x2 y1 z1 ];

    p70=[x1 -y1 z1; ...
         x2 -y1 z1; ...
         x2 -y1 -z1; ...
         x1 -y1 -z1];
    p71=[x1 -y1 -z1; ...
         x1 -y2 -z1; ...
         x2 -y2 -z1; ...
         x2 -y1 -z1];
    p72=[x1 -y2 z1; ...
         x2 -y2 z1; ...
         x2 -y2 -z1; ...
         x1 -y2 -z1];
    p73=[x1 -y1 z1 ; ...
         x1 -y2 z1; ...
         x2 -y2 z1; ...
         x2 -y1 z1];

    drawpolygon(uTg,p50,'b');
    drawpolygon(uTg,p51,'b');
    drawpolygon(uTg,p52,'b');
    drawpolygon(uTg,p53,'b');

    drawpolygon(uTg,p60,'b');
    drawpolygon(uTg,p61,'b');
    drawpolygon(uTg,p62,'b');
    drawpolygon(uTg,p63,'b');

    drawpolygon(uTg,p70,'b');
    drawpolygon(uTg,p71,'b');
    drawpolygon(uTg,p72,'b');
    drawpolygon(uTg,p73,'b');
            
    
end

function drawvector(p0,p1,col,otherfmt,txt)

    if (~isempty(otherfmt))
        cc=sprintf('%c%s-',col,otherfmt);
    else
        cc=sprintf('%c',col);
    end
    plot3([p0(1) p1(1)],[p0(2) p1(2)],[p0(3) p1(3)],cc);
    if (~isempty(txt))
        text(p1(1),p1(2),p1(3),txt,'Color',col);
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