station_cfg={}
station_cfg.ssid="DroneWifi"
station_cfg.pwd="DroneWifi"
wifi.ap.config(station_cfg)
wifi.setmode(wifi.SOFTAP)
print(wifi.ap.getip())
print("Password is DroneWifi")

NextFile = "telemetry.lua"
    l = file.list();
    for k,v in pairs(l) do
        print("name:"..k, "size:"..v)
        if k == NextFile then
        print("Wait 5 Seconds")
        tmr.alarm(0,5000,0,function() dofile(NextFile) end)
        print("Started file "..NextFile)
        else
        end
        
    end
print("End of Startup")
