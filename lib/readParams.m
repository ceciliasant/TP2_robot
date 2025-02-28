function [HBL, LBL, WBL, LD, LC, LB, LA, LX, H, LZ, HTA, STF, HTB, DTF, LTF, DTT, LTT, HTC, WTS]=readParams(file)
    % readParams - Reads parameters from a file and creates variables in the workspace.
    %
    % Input:
    %   file - Name of the file containing parameters in "name = value" format
   
try
     tfid = fopen('tp2.txt');
     tdata = textscan(tfid,'%s=%s');
     fclose(tfid);
     if( numel(tdata{1}) ~= numel(tdata{2}))
         disp('Error reading file. Missing = !');
         clear tdata tfid;
     else
         ndata={ tdata{1} repmat('=', size(tdata{1})) tdata{2}};
         sdata=strcat(ndata{1},ndata{2},ndata{3});  
        for i=1:numel(sdata)
             try
                eval(sdata{i});
             catch
                sprintf('Bad format in line %d of data file!',i);
             end
         end
         clear i tfid ndata tdata sdata;
     end
catch
     disp('Cannot open file.');
end
