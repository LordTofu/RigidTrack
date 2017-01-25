print("Started Circuit Breaker") 
print("IP:") 
print(wifi.sta.getip())
node.setcpufreq(node.CPU160MHZ)

ENABLE_PIN = 1
RED_PIN = 7
GREEN_PIN = 5
BLUE_PIN = 6

enable = false
safety_start = false

gpio.mode(ENABLE_PIN, gpio.OUTPUT)
gpio.mode(RED_PIN, gpio.OUTPUT)
gpio.mode(GREEN_PIN, gpio.OUTPUT)
gpio.mode(BLUE_PIN, gpio.OUTPUT)

-- disable CB
gpio.write(ENABLE_PIN, 0)
-- set led to red 
gpio.write(GREEN_PIN, 1)
gpio.write(RED_PIN, 0)
gpio.write(BLUE_PIN, 1)

-- Timer every 100 ms
tmr.register(0, 100, tmr.ALARM_SEMI, function() 
    -- enable CB and set light to green
    if enable then 
        gpio.write(ENABLE_PIN, 1)
        gpio.write(GREEN_PIN, 0)
        gpio.write(RED_PIN, 1)
        gpio.write(BLUE_PIN, 1)
    -- disable CB and set light to red
    else
        gpio.write(ENABLE_PIN, 0)
        gpio.write(GREEN_PIN, 1)
        gpio.write(RED_PIN, 0)
        gpio.write(BLUE_PIN, 1)
    end
    enable = false
    tmr.start(0)
end)

function udp_received(srv, c)
        command = tonumber(c)
        if command == 9 then safety_start = true end
        if command == 1 and safety_start then enable = true end
        if command == 0 then  enable = false safety_start = false end   
end

tmr.start(0)
port=9156
srv=net.createServer(net.UDP)
srv:on("receive", function(srv, c) udp_received(srv, c) end )
srv:listen(port)
