print("Started Circuit BreakerSender") 
node.setcpufreq(node.CPU160MHZ)

ENABLE_PIN = 6
RED_PIN = 1
GREEN_PIN = 4
BLUE_PIN = 2
timesPressed=0
increase = 1

gpio.mode(ENABLE_PIN, gpio.OUTPUT, gpio.PULLUP)
gpio.mode(RED_PIN, gpio.OUTPUT)
gpio.mode(GREEN_PIN, gpio.OUTPUT)
gpio.mode(BLUE_PIN, gpio.OUTPUT)

gpio.write(RED_PIN, gpio.HIGH)
gpio.write(GREEN_PIN, gpio.HIGH)
gpio.write(BLUE_PIN, gpio.LOW)


-- Timer every 50 ms
tmr.register(0, 50, tmr.ALARM_SEMI, function() 
    
    -- check if connected
    if wifi.sta.getip() == NULL then 
        gpio.write(RED_PIN, gpio.HIGH)
        gpio.write(BLUE_PIN, gpio.HIGH)
        gpio.write(BLUE_PIN, gpio.LOW)
    else
        gpio.write(BLUE_PIN, gpio.HIGH)
    -- enable CB and set light to green
    if gpio.read(ENABLE_PIN) == 1 then
        gpio.write(GREEN_PIN, gpio.HIGH)
        gpio.write(RED_PIN, gpio.LOW)
        print('red')
        if increase == 0 then
        conn:connect(9156,"192.168.4.1")
        conn:send(0)
        conn:close()
        increase = 1
        end
        timesPressed = 0
    else 
        timesPressed = timesPressed + increase
        if timesPressed == 20 then
         conn:connect(9156,"192.168.4.1")
        conn:send(9)
        conn:send(1)
        conn:close()
        timesPressed = 21
        increase = 0
        print('green')
        gpio.write(GREEN_PIN, gpio.LOW)
        gpio.write(RED_PIN, gpio.HIGH)
        elseif timesPressed == 21 then
            conn:connect(9156,"192.168.4.1")
            conn:send(1)
            conn:close()
        end
    end
    end
    tmr.start(0)
end)

tmr.register(1, 100, tmr.ALARM_SEMI, function() 
    if wifi.sta.getip() ~= NULL then 
        tmr.start(0) 
        gpio.write(RED_PIN, gpio.LOW)
        gpio.write(BLUE_PIN, gpio.HIGH)
    else 
        tmr.start(1)
    end
end)

tmr.start(1)
port=9156
srv=net.createServer(net.UDP)
conn = net.createConnection(net.UDP, 0)
conn:connect(9156,"192.168.4.1")
