function [rescode,restxt]=ACLcommand(s,c,verbose)
% verbose is as in ACLreadresponse

    fwrite(s,[c-0 13],'uint8','async'); % async - Arlandi
    [rescode,restxt]=ACLreadresponse(s,c,verbose);
%     fprintf('Response: code=%d, text=[%s]\n',rescode,restxt);
%     restxt-0
    
end