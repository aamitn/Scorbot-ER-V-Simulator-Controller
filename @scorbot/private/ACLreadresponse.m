function [rescode,res]=ACLreadresponse(s,command,verbose)
% echo = 0 -> no echo, 1-> normal echo of serial, 2-> debug echo
% rescode = 1-> command executed ok, 2-> detected error or warning, 3-> timeout

    res='';
    rescode=-1;

    commandresponse='';
    if (searchStr('move',command)==1) || ...
        (searchStr('here',command)==1) || ...
        (searchStr('open',command)==1) || ...
        (searchStr('close',command)==1) || ...
        (searchStr('print',command)==1) || ...
        (searchStr('setpv',command)==1) || ...
        (searchStr('setpvc',command)==1)
         commandresponse='Done.';
    elseif (searchStr('home',command)==1)
        commandresponse='Homing complete(robot).';
    elseif (searchStr('listpv',command)==1)
        if (searchStr('listpv position',command)==0)
            commandresponse='Position';
        end
    elseif (searchStr('listp',command)==1)
        commandresponse='             DEFINED POINTS';
    elseif (searchStr('list',command)==1)
        commandresponse='                PROGRAM';
    end
    if (verbose==2)
        fprintf('---> Command response set to [%s]\n',commandresponse);
    end
  
    % state machine: 0-> waiting echo, 1-> reading lines of response, 2-> 
    % waiting for next prompt to end, -1-> ending
    state=0;
    writecr=0;
    readbuffer=[];
    iteration=0;
    tic;
    while (state>=0)
        lenbuf=length(readbuffer);
        if (verbose==2)
            fprintf('=> iteration #%d, state=%d, buffer of %d, available of %d\n',iteration,state,lenbuf,s.BytesAvailable);
            char(readbuffer)
            readbuffer-0
        end
        % if there is a line in the buffer, read it; otherwise add bytes to
        % the buffer
        doread=1;
        if (lenbuf==1) && (readbuffer(1)=='>')
            a=[readbuffer 13 10];
            len=length(a);
            readbuffer=[];
            doread=0;
        else
            for (f=1:lenbuf)
                if (readbuffer(f)==13)
                    doread=0;
                    if (f<lenbuf) && (readbuffer(f+1)==10)
                        a=readbuffer(1:f+1);
                        readbuffer=readbuffer(f+2:end);
                    else
                        a=readbuffer(1:f);
                        readbuffer=readbuffer(f+1:end);
                    end
                    len=length(a);
                    break;
                end
            end
        end
        if (doread) % add new bytes this iteration
            % write cr if needed
            if (writecr)
                if (verbose==2)
                    fprintf('\t---> Writing cr...\n');
                end
                fwrite(s,13,'uint8'); % to force some response
            end
            % read next bytes
            if (verbose==2)
                fprintf('\t---> Reading bytes...\n');
                toc
            end
    %        a=fgets(s);
            while (s.BytesAvailable<=0)
				pause(0.05); % Arlandi
            end
            a=char(fread(s,s.BytesAvailable)');
            len=length(a);
            if (verbose==2)
                toc
                fprintf('\t---> Read %d:\n',len);
                a
                a-0
            end
            for (f=1:len)
                if (a(f)>0)
                    readbuffer(end+1)=a(f);
                end
            end
        else % interpret line this iteration  
            a=char(a);
            if (verbose==2)
                fprintf('\t--->Interpreting line:\n');
                a
                a-0
            end
            concatit=0;
            writecr=0;
            if (len==0) 
                if (verbose==2)
                    fprintf('!Read zero length\n');
                end
            elseif (len>0)
                % do echo of reading
                if (verbose==1) 
                    for (f=1:len)
                        if (a(f)~=10) % to avoid double new line in matlab
                            fprintf('%c',a(f));
                        end
                    end
                end
                % purge useless characters at the beginning and the end
                if (len==3) && (a(1)=='>') && (a(2)==13) && (a(3)==10)
                    anocrlfs='>';
                    lanocrlfs=1;
                else
                    anocrlfs=a;
                    lanocrlfs=len;
                    while (lanocrlfs>0)
                        if (anocrlfs(1)==10) || (anocrlfs(1)==13) || (anocrlfs(1)=='>')
                            if (lanocrlfs==1) && (anocrlfs(1)=='>')
                                break;
                            end
                            anocrlfs=anocrlfs(2:end);
                            lanocrlfs=lanocrlfs-1;
                        else
                            break;
                        end
                    end
                    while (lanocrlfs>0)
                        if (anocrlfs(end)==10) || (anocrlfs(end)==13) || (anocrlfs(end)=='>')
                            if (lanocrlfs==1) && (anocrlfs(end)=='>')
                                break;
                            end
                            anocrlfs=anocrlfs(1:end-1);
                            lanocrlfs=lanocrlfs-1;
                        else
                            break;
                        end
                    end
                end
                if (verbose==2)
                    fprintf('Read data without trailing characters:\n');
                    anocrlfs
                    anocrlfs-0
                end
                if (lanocrlfs<=0)
                    if (verbose==2)
                        fprintf('Empty purged text\n');
                    end
                else
                    % make decisions
                    switch (state)
                        case 0  % waiting echo
                            if (searchStr(command,anocrlfs)==1)
                                if (verbose==2)
                                    fprintf('Received echo\n');
                                end
                                a=a(length(command)+1:end);
                                len=length(a);
                                if (len>0)
                                    concatit=1;
                                end
                                state=1;
                            end
                        case 1  % reading response lines
                            if (lanocrlfs>=3) && (searchStr('*** ',anocrlfs))
                                concatit=1;
                                if (verbose==2)
                                    fprintf('Detected error\n');
                                end
                                state=-1;
                                rescode=2;
                            elseif (length(commandresponse>0))
                                if (searchStr(commandresponse,anocrlfs)==1) % may be a longer line
                                    if (verbose==2)
                                        fprintf('Received expected response\n');
                                    end
                                    state=2;
                                else
                                    concatit=1;
                                end
                            elseif (lanocrlfs>=9) && (strcmp(anocrlfs(1:9),'Type <cr>'))
                                if (verbose==2)
                                    fprintf('Inserting CR\n');
                                end
                                writecr=1;
                            elseif (anocrlfs(1)=='>')
                                if (verbose==2)
                                    fprintf('Detected prompt\n');
                                end
                                rescode=1;
                                state=-1;
                            else % any text
                                concatit=1;
                            end
                        case 2  % waiting for next prompt to end
                            if (anocrlfs(1)=='>')
                                if (verbose==2)
                                    fprintf('Detected prompt\n');
                                end
                                rescode=1;
                                state=-1;
                            else
                                concatit=1;
                            end
                        otherwise
                            error('Unknown state');
                    end
                end
            else
                error('Read zero length with data');
            end

            if (concatit)
                res=[res a];
            end
        end
        iteration=iteration+1;
    end
    if (verbose==1)
        fprintf('\n');
    end

end

function pos=searchStr(str1,str2)
% find STR1 into STR2

    pos=strfind(str2,str1);
    if (isempty(pos)) 
        pos=0;
    else
        pos=pos(1);
    end

end
