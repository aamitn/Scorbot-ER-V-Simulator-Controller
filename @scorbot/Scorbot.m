% Matlab toolbox for simulation and control of the Scorbot ER-V
% 
% (c) Amit Kumar Nandi
% BIRLA INSTITUTE OF TECHNOLOGY, MESRA
% PRODUCTION ENGINEERING DEPARTMENT
% AUTONOMOUS SYSTEMS LABORATORY
%
% Create Scorbot objects passing as parameter their mode.
%
% The functions of this library allow the Matlab programmer to command
% basic actions in the Scorbot ER-V and ER-V plus robots, including 
% simulations. You can act on the simulated robot, on the real robot or in
% both at once (at reading statuses in this latter hybrid mode, you can
% read either the simulated or real ones).
%
% The resources consumed by these functions in the real robot controller 
% are minimum: just a position called PMCL0 is created for internal 
% matters.
%
% --- NOTES: 
%
%	1.- DO NOT HOLD any key pressed while in the pendant method.
%
%	2.- DO NOT DRAW anything in a figure while using the scorbot simulator,
%	unless your figure has an explicit ID and is selected by its ID.
%
%	3.- DO NOT MIX poses coming from the real robot with poses coming from 
%   the simulator.
%
%	4.- DO NOT SAVE the scorbot object along with your recorded poses. Save 
%   only poses.
%
% The methods you can use are (please use "help Scorbot.<method>" to get
% more details):
%
%   home -> homing of the scorbot. Should be called once after ScbInit.
%
%	changeGripper -> open/close the gripper.
%
%   speed -> read the current speed of motion of the robot.
%
%   changeSpeed -> change the speed of motion of the robot.
%
%   currentPos -> return an object containing all the information about
%       the current position of the robot, both in joint and cartesian
%       coordinates. This object can be read and assigned to other vars.,
%       but it is not recommended to change its values directly (use 
%       functions below).
%
%   changePosJoint -> change the joint values of a given position to new
%       ones, updating the cartesian position as well.
%
%
%   changePosXYZ -> change the cartesian coordinates of a given position
%       to a new ones, updating the joint position as well.
%
%   changePosPitch -> change the pitch of the gripper in a given position.
%
%   changePosRoll -> change the roll of the gripper in a given position.
%
%   move -> move the robot to a given position.
%
%   clearMoveQueue -> clear the current queue of move commands, if any.
%
%   abort -> abort any pending or current motion of the robot abruptly.
%
%   pendant -> [only in simulation mode] opens a teach pendant to change
%   position of the robot manually. 

classdef Scorbot < handle
    % It is a handle for being modified when passed to a function; copies
    % will refer to the same object, but you can create any number of
    % objects (more than one real scorbot will produce errors in
    % communications anyway).
    
    properties (Constant)
    
        MODESIM=1; % simulation
        MODEREAL=2; % real control of the robot
        MODEHYB=3; % both
        
        MODESTRINGS=['simulation';...
                     'real      ';...
                     'hybrid    ']; % names of modes
        
    end
    
    properties (Access = private)
    
        mode;

        % simulation properties
        
        consts;
        state;
        scbspeed;
        world;
        display;
        
        % real properties
        
        conn;
        
    end
    
    
    methods (Access = public)
        
        % ----- Constructor

        function obj = Scorbot(mode)
           
            if (nargin==0)
                mode=obj.MODESIM; % simulation by default
            end

            obj.checkmode(mode);
            
            obj.mode=mode;
            
            if (obj.issimmode())
                % simulation robot initialization
                
                obj.consts=SSCscorbotConsts();
                [thetas,pitch,roll,gripper]=SSCrobotHome(obj.consts);
                obj.state=struct('thetas',thetas,'pitch',pitch,'roll',roll,'gripper',gripper);
                obj.display=SSCcreateDisplay(obj.consts,[],[]);
                obj.display=SSCtiporientOn(obj.display);
                obj.display=SSCupdateDisplay(obj.display,obj.consts,obj.state,[]);
                obj.scbspeed=60;
                obj.world=[];
                
            end
                
            if (obj.isrealmode())
                % real robot initialization
                
                obj.conn=ACLinit;
                if (strcmp(obj.conn.Status,'open'))
                    % define the auxiliary position for rest of commands
                    [rescode,restxt]=ACLcommand(obj.conn,'echo',0);
                    if (rescode~=1)
                        ACLfinish(obj.conn);
                        error('Error setting echo ON [%s].',restxt);
                    end
                    fprintf('--> Echo ON.\n');
                    [rescode,restxt]=ACLcommand(obj.conn,'defp pmcl0',0);
                    if (rescode~=1)
                        pos=searchStr('*** WARNING *** point already exists.',restxt);
                        if (pos==0)
                            ACLfinish(obj.conn);
                            error('Error creating auxiliary position [%s].',restxt);
                        else
                            rescode=1;
                        end
                    end
                    if (rescode==1)
                        [rescode,restxt]=ACLcommand(obj.conn,'here pmcl0',0);
                        if (rescode~=1)
                            ACLfinish(obj.conn);
                            error('Error initiating auxiliary position [%s].',restxt);
                        end
                        fprintf('--> Position PMCL0 defined in the controller for Matlab use.\n');
                    end
                    fprintf('--> Matlab connection with Scorbot controller established OK.\n');

					obj.changeSpeed(30);
					fprintf('--> Speed set automatically to 30 for safety.\n');

                 else
                    fprintf('Error opening RS232 connection:\n');
                    obj.conn
                    ACLfinish(obj.conn);
                    error('Error opening connection.');
                 end
              
            end
            
            fprintf('Created Scorbot object in %s mode. Delete it with "clear obj".\n',obj.MODESTRINGS(obj.mode,:));
           
        end
        
        % ----- Destructor
        
        function delete(obj)

            obj.checkmode();
            
            if (obj.issimmode())
                SSCdestroyDisplay(obj.display);
                obj.display=[];
            end
            
            if (obj.isrealmode())
                ACLfinish(obj.conn);
            end
            
            fprintf('Deleted Scorbot object.\n');
            
        end
        
        % ----- Public methods
        
        function s=speed(obj,m)
        % USAGE: res=obj.speed() or res=obj.speed(m)
        %
        %   RES -> current speed
        %   M -> (optional) If the robot is in hybrid mode, this argument
        %   indicates whether to read the simulated speed (default) or the 
        %   real one.
        %
        % Return the current speed of motion of the Scorbot
            
            obj.checkmode();
            
            if (nargin==2) 
                checkgetmode(obj,m);
            else 
                if (obj.mode==Scorbot.MODEHYB)
                    m=Scorbot.MODESIM;
                else
                    m=obj.mode;
                end
            end

            if (obj.issimmode(m))
                s=obj.scbspeed;
            end
                
            if (obj.isrealmode(m))
                [rescode,restxt]=ACLcommand(obj.conn,'show speed',0);
                if (rescode~=1)
                    error(restxt);
                end
                poscolons=strfind(restxt,':');
                if (length(poscolons)~=2)
                    error('Error interpreting speeds.');
                end
                [speeds,valid]=ACLextractNvalsfrompos(restxt,poscolons(1:2));
                s=speeds(1);                
            end

        end
        
        function changeSpeed(obj,s)
        % USAGE: obj.changeSpeed(speed)
        %
        %   SPEED <- New speed value (from 1 to 100)
        %
        % Change the speed to the given one.
           
            obj.checkmode();
            
            Scorbot.checkint(s,1,100,'Invalid speed value');
    
            if (obj.issimmode())
                obj.scbspeed=s;
            end
            
            if (obj.isrealmode())
                [rescode,restxt]=ACLcommand(obj.conn,sprintf('speed %d',s),0);
                if (rescode~=1)
                    error(restxt);
                end
            end
    
        end
        
        function home(obj)
        % USAGE: obj.home()
        %
        % Move the robot to its homing position.
            
            obj.checkmode();
            
            if (obj.issimmode())
                [thetas,pitch,roll,gripper]=SSCrobotHome(obj.consts);
                obj.state=struct('thetas',thetas,'pitch',pitch,'roll',roll,'gripper',gripper);      
                obj.display=SSCupdateDisplay(obj.display,obj.consts,obj.state,obj.world);
            end
            
            if (obj.isrealmode())
                [rescode,restxt]=ACLcommand(obj.conn,'home',1);
                if (rescode~=1)
                    error(restxt);
                end
            end

        end
        
        function pos=currentPos(obj,m)
        % USAGE: pos=obj.currentPos() or pos=obj.currentPos(m)
        %
        %   POS -> Current complete position of the Scorbot.
        %   M -> (optional) If the robot is in hybrid mode, this argument
        %   indicates whether to read the simulated pose (default) or the 
        %   real one.
        %
        % Return into POS an object with all the information about the 
        % current robot position.
        %
        % NOTE: Never change any position such as POS directly: use the 
        %       changePos* methods of the Scorbot class.
            
            obj.checkmode();

            if (nargin==2) 
                checkgetmode(obj,m);
            else 
                if (obj.mode==Scorbot.MODEHYB)
                    m=Scorbot.MODESIM;
                else
                    m=obj.mode;
                end
            end
            
            if (obj.issimmode(m))
                [uTs,Q,uTg,posetip]=SSCdirectModel(obj.consts,obj.state.thetas,obj.state.pitch,obj.state.roll);
                [motors,mpitch,mroll]=SSCthetasToMotors(obj.consts,obj.state.thetas,obj.state.pitch,obj.state.roll);
                pos=struct('joints',[round(motors) 0 0],...
                           'xyz',round(posetip(1:3)*1000*10),...
                           'pitch',mpitch,...
                           'roll',mroll);
            end
            
            if (obj.isrealmode(m))
                pos=struct();
                [rescode,restxt]=ACLcommand(obj.conn,'listpv position',0);
                if (rescode~=1)
                    error(restxt);
                else
                    poscolons=strfind(restxt,':');
                    if (length(poscolons)~=10)
                        error('Error interpreting position (1)');
                    end
                    [joints,valid]=ACLextractNvalsfrompos(restxt,poscolons(1:5));
                    if (~valid)
                        error('Could not extract joints from robot response');
                    end
                    [xyz,valid]=ACLextractNvalsfrompos(restxt,poscolons(6:8));
                    if (~valid)
                        error('Could not extract XYZ from robot response');
                    end
                    [pr,valid]=ACLextractNvalsfrompos(restxt,poscolons(9:10));
                    if (~valid)
                        error('Could not extract pitch and roll from robot response');
                    end
                    pos=struct('joints',joints,...
                               'xyz',xyz,...
                               'pitch',pr(1),...
                               'roll',pr(2));
                end
            end
                
        end

        function [p,t]=move(obj,pos,wait)
        % USAGE: [p,t]=obj.move(pos,wait)
        %
        %   POS <- Position to move to
        %   WAIT <- 0 for not waiting to completion, 1 or 2 for waiting
        %	P -> if WAIT==2, a cell with intermediate positions of the 
        %        robot while it went to the target position, not including 
        %        the initial or final positions necessarily; otherwise, an
        %        empty cell.
        %	T -> if WAIT==2, a vector with the time in seconds at which 
        %        each intermediate position was retrieved (considering 0 
        %        as the time at which the movement command was replied from
        %        the robot); otherwise, an empty vector
        %
        % Move the scorbot to position POS.
        %
        % If WAIT==1, waits until the movement is finished.
        % If WAIT==2, waits until the movement is finished and records 
        %   intermediate positions.
        % If WAIT==0 and the robot has received previously other move 
        %   commands, finishes them before doing this one, i.e., move 
        %   commands are queued in the robot controller. This means that 
        %   other commands will be executed concurrently to the movements 
        %   (for instance, opening/closing the gripper)
        %
        % NOTE: In simulation mode, WAIT==0 is not supported.
        
            obj.checkmode();
            
            p={};
            t=[];

            if (obj.issimmode()) 
                if (wait==0)
                    error('Invalid wait code in simulation mode.');
                end
                [thetas1,pitch1,roll1]=SSCmotorsToThetas(obj.consts,pos.joints(1:3),pos.pitch,pos.roll);
                outofrange=SSCcheckThetasPR(obj.consts,thetas1,pitch1,roll1);
                if (outofrange)
                    error('Destination position unreachable.');
                end
                path=SSCdefinePath(obj.consts,obj.scbspeed/100*4*pi/180,...
                                   obj.state.thetas,obj.state.pitch,obj.state.roll,...
                                   thetas1,pitch1,roll1);
                n=length(path);
                t0=tic;
                for (f=1:n)
                    obj.state.thetas=path{f}(1:3);
                    obj.state.pitch=path{f}(4);
                    obj.state.roll=path{f}(5);
                    obj.display=SSCupdateDisplay(obj.display,obj.consts,obj.state,obj.world);

                    [uTs,Q,uTg,posetip]=SSCdirectModel(obj.consts,obj.state.thetas,obj.state.pitch,obj.state.roll);
                    [motors,mpitch,mroll]=SSCthetasToMotors(obj.consts,obj.state.thetas,obj.state.pitch,obj.state.roll);
                    t1=toc(t0);
                    if (wait==2)
                        t=[t t1];
				        curpos=struct('joints',[round(motors) 0 0],...
					                  'xyz',round(posetip(1:3)*1000*10),...
				                      'pitch',mpitch,...
				                      'roll',mroll);            
                        p{length(p)+1}=curpos;
                    end
                end
                obj.display=SSCupdateDisplay(obj.display,obj.consts,obj.state,obj.world);
            end

            if (obj.isrealmode())
                if (ACLsetauxpos(obj.conn,pos,'j')==0)
                    error('Error changing position in robot');
                end
                [rescode,restxt]=ACLcommand(obj.conn,'move pmcl0',0);
                t0=tic;
                if (rescode~=1)
                    error(restxt);
                end
                if (wait==1)||(wait==2)
                    finished=0;
                    while (~finished)
                        curpos=obj.currentPos();
                        t1=toc(t0);
                        if (wait==2)
                            t=[t t1];
                            p{length(p)+1}=curpos;
                        end
                        dist=sum(abs(curpos.joints-pos.joints));
                        if (dist<5*2)
                            finished=1;
                        end
                    end
                end
            end
    
        end
        
        function newpos=changePosPitch(obj,pos,pitch)
        % USAGE: newpos=obj.changePosPitch(pos,pitch)
        %
        %   POS <- Position to be modified
        %   PITCH <- New pitch value
        %   NEWPOS -> Modified position
        %
        % Change the value of the pitch angle of the gripper in POS to the 
        % new value PITCH, returning into NEWPOS the new position adjusted
        % for being a valid one in all its parts.
        %
        % NOTE: In simulation mode, this does not affect the last encoder 
        % values of the joints (4,5), as it does in real mode.
           
            obj.checkmode();
            
            newpos=pos;
            Scorbot.check16int(pitch,'Invalid pitch value');

            if (obj.issimmode()) 
                [thetas,p,r]=SSCmotorsToThetas(obj.consts,pos.joints(1:3),pitch,pos.roll);
                outofrange=SSCcheckThetasPR(obj.consts,thetas,p,r);
                if (outofrange)
                    error('Position out of range.\n');
                end
                newpos.pitch=pitch;
            end
    
            if (obj.isrealmode())    
                newpos.pitch=pitch;
                if (ACLsetauxpos(obj.conn,newpos,'p')==0)
                    error('Error in sending change to the robot');
                end
                [res,newpos]=ACLgetauxpos(obj.conn);
                if (~res)
                    error('Error in receiving robot response');
                end
            end

        end
                
        function newpos=changePosRoll(obj,pos,roll)
        % USAGE: newpos=obj.changePosRoll(pos,roll)
        %
        %   POS <- Position to be modified
        %   ROLL <- New roll value
        %   NEWPOS -> Modified position
        %
        % Change the value of the roll angle of the gripper in POS to the 
        % new value ROLL, returning into NEWPOS the new position adjusted
        % for being a valid one in all its parts.
        %
        % NOTE: In simulation mode, this does not affect the last encoder 
        % values of the joints (4,5), as it does in real mode.
        
            obj.checkmode();

            newpos=pos;
            Scorbot.check16int(roll,'Invalid roll value');

            if (obj.issimmode()) 
                [thetas,p,r]=SSCmotorsToThetas(obj.consts,pos.joints(1:3),pos.pitch,roll);
                outofrange=SSCcheckThetasPR(obj.consts,thetas,p,r);
                if (outofrange)
                    error('Position out of range.');
                end
                newpos.roll=roll;
            end    

            if (obj.isrealmode())    
                newpos.roll=roll;
                if (ACLsetauxpos(obj.conn,newpos,'r')==0)
                    error('Error changing position in controller');
                end
                [res,newpos]=ACLgetauxpos(obj.conn);
                if (~res)
                    error('Error getting new position from controller');
                end
            end
            
        end
        
        function newpos=changePosJoint(obj,pos,values)
        % USAGE: newpos=obj.changePosJoint(pos,values)
        %
        %   POS <- Position to be modified
        %   VALUES <- New joint values for that position
        %   NEWPOS -> Modified position
        %
        % Change all the joints of POS (5 values) to the new ones in VALUES 
        % (encoder values), adjusting the cartesian part part of the
        % position for completing a valid joint+xyz+p+r position.
        %
        % NOTE: recall that joint 3 moves relative to the universal frame, 
        % and not to the previous joint; also, changing the gripper joints 
        % (4,5) is not advisable because they have a mixed effect on pitch 
        % and roll that is difficult to calculate (use changePosPitch/Roll 
        % instead).

            obj.checkmode();
            
            newpos=pos;
            if (length(values)~=5)
                error('Invalid joint vector: it must have 5 elements.');
            end
            for (f=1:5)
                Scorbot.check16int(values(f),'Invalid encoder value');
            end

            if (obj.issimmode())
                [motors,mpitch,mroll]=SSCthetasToMotors(obj.consts,obj.state.thetas,obj.state.pitch,obj.state.roll);
                [thetas,pitch,roll]=SSCmotorsToThetas(obj.consts,values(1:3),mpitch,mroll); % we leave pitch/roll unmodified
                outofrange=SSCcheckThetasPR(obj.consts,thetas,pitch,roll);
                if (outofrange)
                    error('Destination position is unreachable.');
                end
                [uTs,Q,uTg,posetip]=SSCdirectModel(obj.consts,thetas,pitch,roll);
                newpos.joints=values;
                newpos.xyz=round(posetip(1:3)*1000*10);
            end

            if (obj.isrealmode()) 
                newpos.joints=values;
                if (ACLsetauxpos(obj.conn,newpos,'j')==0)
                    error('Error sending change to the controller');
                end
                [res,newpos]=ACLgetauxpos(obj.conn);
                if (~res)
                    error('Error getting answer from the controller');
                end
            end

        end
        
        function newpos=changePosXYZ(obj,pos,xyz)
        % USAGE: newpos=obj.changePosXYZ(pos,xyz)
        %
        %   POS <- Position to be modified
        %   XYZ <- New cartesian values for that position
        %   NEWPOS -> Modified position
        %
        % Change the value of coordinates X, Y, Z of POS to the new values 
        % in the three-element vector XYZ, returning into NEWPOS the new 
        % position adjusted for being a valid one in all its parts, 
        % including the joints.
        %
        % NOTE: The x-y-z coordinates correspond to the mid point of the 
        % gripper tip (care if the tip is extended with some extra piece). 
        % Pitch and roll are not changed by this routine.

            obj.checkmode();

            newpos=pos;
            if (length(xyz)~=3)
                error('Invalid xyz vector: its length must be 3');
            end
            for (f=1:3)
                Scorbot.check16int(xyz(f),'Invalid coordinate value');
            end

            if (obj.issimmode()) 
                [thetas,pitch,roll]=SSCmotorsToThetas(obj.consts,pos.joints(1:3),pos.pitch,pos.roll);
                outofrange=SSCcheckThetasPR(obj.consts,thetas,pitch,roll);
                if (outofrange)
                    error('Original position is out of range.');
                end
                [uTs,Q,uTg,posetip]=SSCdirectModel(obj.consts,thetas,pitch,roll);
                [nosols,thetas,pitch,roll]=SSCinverseModel(obj.consts,[xyz/10/1000 posetip(4:6)],thetas);
                if (nosols==0)
                    error('Destination position is unreachable.');
                end
                [motors,mpitch,mroll]=SSCthetasToMotors(obj.consts,thetas,pitch,roll); % no need to check outofrange because inverse model already does
                newpos.joints=[round(motors(1:3)) 0 0];
                newpos.xyz=xyz;
            end

            if (obj.isrealmode())
                newpos.xyz=xyz;
                if (ACLsetauxpos(obj.conn,newpos,'x')==0)
                    error('Error sending change to the controller');
                end
                [res,newpos]=ACLgetauxpos(obj.conn);
                if (~res)
                    error('Error getting results from the controller');
                end
            end

        end
        
        function changeGripper(obj,openorclose)
        % USAGE: obj.gripper(openorclose)
        %
        %   OPENORCLOSE <- 1 to open, 0 to close the gripper
        %
        % Open (openorclose==1) or close (openorclose==0) the gripper of 
        % the Scorbot and return when it is done.

            obj.checkmode();

            if (obj.issimmode())
                if (openorclose==1)
                    obj.state.gripper=100;
                else
                    obj.state.gripper=0;
                end
                obj.display=SSCupdateDisplay(obj.display,obj.consts,obj.state,obj.world);
            end
            
            if (obj.isrealmode())
                if (openorclose==1)
                    [rescode,restxt]=ACLcommand(obj.conn,'open',0);
                else
                    [rescode,restxt]=ACLcommand(obj.conn,'close',0);
                end
                if (rescode~=1)
                    error(restxt);
                end
            end

        end
        
        function abort(obj)
        % USAGE: obj.abort()
        %
        % Stop immediately any motion of the robot and print message if 
        % any error.
        %
        % NOTE: This function does nothing in simulation mode.

            obj.checkmode();

            if (obj.isrealmode())           
                [rescode,restxt]=ACLcommand(obj,'A',0);
                if (rescode~=1)
                    error(restxt);
                end
            end

        end
        
        function clearMoveQueue(obj)
        % USAGE: obj.clearMoveQueue()
        %
        % Clear the queue of pending move commands.
        %
        % NOTE: This function does nothing in simulation mode.

            obj.checkmode();

            if (obj.isrealmode())           
                [rescode,restxt]=ACLcommand(obj.conn,'clrbuf',0);
                if (rescode~=1)
                    error(restxt);
                end
            end

        end
        
        function lastpos=pendant(obj)
        % USAGE: lastpos=obj.pendant()
        %
        % Allows the user to move manually the position of the robot. Some 
        % help is shown in the console, and can be re-displayed by pressing 
        % 'h'. After ending the application, the last position of the robot is
		% returned into LASTPOS.
        %
        % NOTE: This method does nothing in real mode.

            obj.checkmode();

            if (obj.issimmode()) 
                [obj.state,obj.display]=SSCteachPendant(obj.consts,obj.display,obj.world);
				lastpos=obj.currentPos();
            end

        end
        
    end % --- end public methods
    

    methods (Static, Access = private)

        function unimplemented()
            % produce an unimplemented error
            error('Unimplemented operation');
        end

        function res=isoneint(x)
            % return 1 if X is an integer (non-array)
            res=(x==fix(x)); 
        end
        
        function checkint(x,minx,maxx,txt)
            % produce an error if X is not an integer within that range
            if (~Scorbot.isoneint(x))||(x<minx)||(x>maxx)
                error('%s: it must be an integer from %d to %d.',...
                      txt,minx,maxx);
            end
        end
        
        function check16int(x,txt)
            % produce an error if X is not an int between -32768 and +32767
            Scorbot.checkint(x,-32768,32767,txt);
        end
        
    end
    
    
    methods (Access = private)
                
        function checkmode(obj,m)
            % produce an error if the mode is invalid
            if (nargin<2)
                m=obj.mode;
            end
            Scorbot.checkint(m,1,Inf,'Invalid mode');
            switch (m)
                case {Scorbot.MODEREAL, Scorbot.MODESIM, Scorbot.MODEHYB}
                otherwise
                    error('Invalid mode (%d) for Scorbot object',m);
            end
        end
        
        function r=isrealmode(obj,m)
            % return 1 if the object is in real or hybrid modes
            if (nargin<2)
                m=obj.mode;
            end
            switch (m)
                case {Scorbot.MODEREAL,Scorbot.MODEHYB}
                    r=1;
                otherwise
                    r=0;
            end
       end

        function r=issimmode(obj,m)
            % return 1 if the object is in simulation or hybrid modes
            if (nargin<2)
                m=obj.mode;
            end
            switch (m)
                case {Scorbot.MODESIM,Scorbot.MODEHYB}
                    r=1;
                otherwise
                    r=0;
            end
        end

        function checkgetmode(obj,m)
            % check whether the mode of the object and M are correct when M
            % is used as an indicator for the mode of reading a robot value     
            if (obj.mode~=Scorbot.MODEHYB)
                error('Object must be in hybrid mode to use arg. M.');
            end
            obj.checkmode(m);
            if (m==Scorbot.MODEHYB)
                error('Arg. M cannot be hybrid mode.');
            end
        end
                    
    end

    
end
