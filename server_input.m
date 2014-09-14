function [data] = server_input(t,n)
 if(n==1)
  while(t.BytesAvailable == 0) 
  end
  message = fgets(t);
  data = sprintf('%s',message);
  disp(data);
  flushinput(t);
 elseif(n==2)
  if(t.BytesAvailable == 0)
       data=[];
  else
      message = fread(t,t.BytesAvailable);
      data = sprintf('%s',message);
      if(~(isempty(data)))
        disp(data);
      flushinput(t);
   end
 end
end

