print("Started Circuit BreakerSender") 
node.setcpufreq(node.CPU160MHZ)

ENABLE_PIN = 6
timesPressed=0
increase = 1

gpio.mode(ENABLE_PIN, gpio.OUTPUT, gpio.PULLUP)

-- Timer every 10 ms
tmr.register(0, 10, tmr.ALARM_SEMI, function() 
    -- enable CB and set light to green
    if gpio.read(6) == 1 then 
        if increase == 0 then
        conn:connect(9156,"192.168.4.1")
        conn:send(0)
        conn:close()
        increase = 1
        end
        timesPressed = 0
    else 
        timesPressed = timesPressed + increase
        if timesPressed == 100 then
         conn:connect(9156,"192.168.4.1")
        conn:send(9)
        conn:send(1)
        conn:close()
        timesPressed = 101
        increase = 0
        elseif timesPressed == 101 then
            conn:connect(9156,"192.168.4.1")
            conn:send(1)
            conn:close()
        end
    end
    tmr.start(0)
end)

function udp_send(srv, c)
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

conn = net.createConnection(net.UDP, 0)

conn:connect(9156,"192.168.4.1")
