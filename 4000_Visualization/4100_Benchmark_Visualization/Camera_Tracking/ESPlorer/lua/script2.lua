--Startup file--
NextFile="two.lua"
 l = file.list();
    for k,v in pairs(l) do
    --  print("name:"..k..", size:"..v)
         if k == NextFile then
         print("Wait 5 seconds please")
         tmr.alarm(0, 5000, 0, function() dofile(NextFile) end)
         print("Started file ".. NextFile)
         else
       --  do nothing
         end
    end
print("End of startup")   
