print("Started Circuit Breaker") 
print(wifi.sta.getip())
node.setcpufreq(node.CPU160MHZ)

ENABLE_PIN = 4
enable = false
safety_start = false

gpio.mode(ENABLE_PIN, gpio.OUTPUT)

-- Timer every 100 ms
tmr.register(0, 200, tmr.ALARM_SEMI, function() 
    if enable then 
        gpio.write(ENABLE_PIN, 1)
    else
        gpio.write(ENABLE_PIN, 0)
    end
    enable = false
    tmr.start(0)
end)

function udp_received(srv, c)
        tmr.start(0)
        command = tonumber(c)
        --print(command)
        if command == 9 then safety_start = true end
        if command == 1 and safety_start then enable = true end
        if command == 0 then  enable = false safety_start = false end   
end


gpio.write(ENABLE_PIN, 1)
port=9156
srv=net.createServer(net.UDP)
srv:on("receive", function(srv, c) udp_received(srv, c) end )
srv:listen(port)
