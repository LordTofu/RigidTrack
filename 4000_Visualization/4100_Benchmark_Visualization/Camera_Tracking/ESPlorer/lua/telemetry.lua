-- Create Telemetry stuff
--crc_module = require "crc16"
print("Started Telemetry")

payload = {} -- 0 is vel z 1 is roll 2 is pitch 3 is heading
for i = 1, 4 do
    payload[i] = 0.01
end

tmr.register(0, 100, tmr.ALARM_SEMI, function() 
    --print(payload[1])
    --print(payload[2])
    --print(payload[3])
    --print(payload[4])
    sendData()
    tmr.start(0)
end)

index = 0

function udp_received(srv, c)
    command = tonumber(c)
    if command == -9991 then index = 1
    elseif command == -9992 then index = 2
    elseif command == -9993 then index = 3
    elseif command == -9994 then index = 4
    else
        command = string.format("%x", command);
        if string.len(command)==2 then command = "00"..command end
        if string.len(command)==3 then command = "0"..command end
        payload[index] = command
    end
end

function sendData()
    --checksum = crc_module.hash(string.char(0xFF, 0x01, 0x2F))
    uart.write(0,'\n', 'n', 0xFF, 0x24) -- start bytes, ID and length
    -- Position: MSB 2 3 LSB; MSP 2 3 LSB; MSB 2 3 LDB;
    uart.write(0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00) --Position
    uart.write(0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, payload[1]) --Velocity
    uart.write(0, payload[2], payload[3], payload[4] ) --Euler
    uart.write(0, 0x00, 0x00) --CRC

    
end

--uart.setup(0, 115200, 8, uart.PARITY_NONE, uart.STOPBITS_1, 0)

port=9155
srv=net.createServer(net.UDP)
srv:on("receive", function(srv, c) udp_received(srv, c) end )
srv:listen(port)
tmr.start(0)
