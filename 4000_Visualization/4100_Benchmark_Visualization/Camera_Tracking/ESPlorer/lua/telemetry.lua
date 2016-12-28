-- Create Telemetry stuff
crc_module = require "crc16"

tmr.register(0, 10, tmr.ALARM_SEMI, function() 
    sendData()
    --tmr.start(0)
end)

function udp_received(srv, c)
    command = tonumber(c)
    print(command)   
   
end

function sendData()
    checksum = crc_module.hash(string.char(0xFF, 0x01, 0x2F))
    --uart.write(0, 0x00, 0x00, 0xFF, 0x01, 0x2F, 0xFF)
    print(checksum)
    print("\n")
end

--uart.setup(0, 921600, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)

port=5000
srv=net.createServer(net.UDP)
srv:on("receive", function(srv, c) udp_received(srv, c) end )
srv:listen(port)
tmr.start(0)
