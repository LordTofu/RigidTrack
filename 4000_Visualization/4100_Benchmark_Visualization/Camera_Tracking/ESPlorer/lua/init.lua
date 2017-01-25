-- Commands to set up a Wifi AP on Windows 10
--netsh wlan set hostednetwork mode=allow ssid=DroneWifi key=DroneWifi
--netsh wlan start hostednetwork

-- Uncomment files to set up an AP by the ESP8266
station_cfg={}
station_cfg.ssid="DroneWifi"
station_cfg.pwd="DroneWifi"
wifi.ap.config(station_cfg)
wifi.setmode(wifi.SOFTAP)
print(wifi.ap.getip())
print("Password is DroneWifi")

-- Connect to an existing AP
--wifi.setmode(wifi.STATION)
--wifi.sta.config("DroneWifi","DroneWifi")
--print(wifi.sta.getip())

NextFile = "circuitBreaker.lua"
    l = file.list();
    for k,v in pairs(l) do
        print("name:"..k, "size:"..v)
        if k == NextFile then
        tmr.alarm(1,5000,0,function() print(wifi.sta.getip()) end)
        print("Wait 10 Seconds")
        tmr.alarm(0,10000,0,function() dofile(NextFile) end)
        print("Started file "..NextFile)
        else
        end
        
    end
print("End of Startup")

