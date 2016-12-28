node.setcpufreq(node.CPU160MHZ)
STEP_PIN = 7
DIR_PIN = 8
ENABLE_PIN = 6
US_TO_MS = 1
gpio.mode(STEP_PIN, gpio.OUTPUT)
gpio.mode(DIR_PIN, gpio.OUTPUT)
gpio.mode(ENABLE_PIN, gpio.OUTPUT)
delay = 500
pos_u = 0
pos = 0
diff = 0
dir = 1 -- 1 positive -1 negative turn direction
output = 0
enable = 0

tmr.register(0, 1, tmr.ALARM_SEMI, function() 
    diff = pos_u - pos
    if diff > 0 then gpio.write(DIR_PIN, 1) dir = 1  end
    if diff < 0 then gpio.write(DIR_PIN, 0) dir = -1 end
    if diff == 0 then 
    
    else 
    
    gpio.write(STEP_PIN, 1)
    tmr.delay(100)
    gpio.write(STEP_PIN, 0)
    pos = pos + dir  
    end 
    tmr.start(0)
end)

function udp_received(srv, c)
        command = tonumber(c)
        if command == 99999 then enable = 1 end
        if command == -99999 then  enable = 0 end
        if enable == 1 then 
        pos_u = tonumber(c) 
        end
        print(command)      
end
tmr.start(0)
gpio.write(ENABLE_PIN, 1)
port=5000
srv=net.createServer(net.UDP)
srv:on("receive", function(srv, c) udp_received(srv, c) end )
srv:listen(port)


