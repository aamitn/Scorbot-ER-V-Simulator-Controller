function s=ACLinit()

    s = serial('COM1');
    set(s,'Port','COM1');
    set(s,'BaudRate',9600);
    set(s,'DataBits',8);
    set(s,'StopBits',1);
    set(s,'FlowControl','none');
    set(s,'Parity','none');
    set(s,'DataTerminalReady','on');
    set(s,'RequestToSend','on');
    set(s,'TimeOut',30);
    set(s,'ReadAsyncMode','continuous');
    fopen(s);
	readasync(s); % Arlandi
    if (strcmp(s.Status,'open'))
        if (s.BytesAvailable>0)
            fprintf('Opening port. Please wait...\n');
            while (s.BytesAvailable>0)
                A=fread(s,s.BytesAvailable);
            end
        end
    else
        error('Not opened port!');
    end

end
