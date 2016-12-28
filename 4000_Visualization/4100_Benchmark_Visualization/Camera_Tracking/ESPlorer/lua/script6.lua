print(wifi.sta.getip())
--nil
node.setcpufreq(node.CPU160MHZ)
STEP_PIN = 7
DIR_PIN = 8
US_TO_MS = 1
gpio.mode(STEP_PIN, gpio.OUTPUT)
gpio.mode(DIR_PIN, gpio.OUTPUT)
delay = 500

output = 0

tmr.register(0, delay, tmr.ALARM_SEMI, function() 
  if output == 0 then output = 1 else output = 0 end
  gpio.write(STEP_PIN, 1)
  tmr.delay(100)
  gpio.write(STEP_PIN, 0)
  tmr.start(0)
end)
tmr.start(0)

-- setup I2c and connect display
function init_i2c_display()
    -- SDA and SCL can be assigned freely to available GPIOs
    local sda = 2 -- GPIO14
    local scl = 1 -- GPIO12
    local sla = 0x3c -- 0x3c or 0x3d
    i2c.setup(0, sda, scl, i2c.SLOW)
    disp = u8g.ssd1306_64x48_i2c(sla)
end


-- graphic test components
function prepare()
    disp:setFont(u8g.font_chikita)
    disp:setFontRefHeightExtendedText()
    disp:setDefaultForegroundColor()
    disp:setFontPosTop()
end


function graphics_test()
    
    disp:firstPage()
    repeat
        disp:drawStr(0, 0, string1)
        disp:drawStr(0, 7, string2)
        disp:drawStr(0, 14, ip)
        disp:drawStr(0, 21, time)
        disp:drawStr(0, 28, string3)
    until disp:nextPage() == false        
    
    tm = rtctime.epoch2cal(rtctime.get())
    time = string.format("%02d:%02d:%02d", tm["hour"]+1, tm["min"],tm["sec"])
    tmr.start(1)
end

string1 =""
string2 =""
string3 ="Pulse Rate"
ip = ""
time=""

init_i2c_display()
--init_spi_display()
prepare()

wifi.setmode(wifi.SOFTAP)
cfg={}
cfg.ssid="ServoMotor"
cfg.pwd="kdjeu!29"
wifi.ap.config(cfg)

tmr.alarm(3, 1000, 1, function()
     if wifi.sta.getip() == nil then
         string1 = "Connecting..."
         string2 = ""
     else
         tmr.stop(3)
         string1 = "Connected to:"
         string2 = "Internet"
         ip = wifi.sta.getip()
         sntp.sync("pool.ntp.org", function()
            print("Done")
           
        end)
     end
end)


-- set up timer 0 with short interval, will be retriggered in graphics_test()
tmr.register(1, 1000, tmr.ALARM_SEMI, function() graphics_test() end)

print("--- Starting Graphics Test ---")
--tmr.start(1)


function udp_received(srv, c)
        --c = tonumber(c)
        --if c > 0 then gpio.write(DIR_PIN, 1) else c = c*-1 gpio.write(DIR_PIN, 0) end 
        --tmr.interval(0,c)
        print(c)
        string3 = c
end

port=5000
srv=net.createServer(net.UDP)
srv:on("receive", function(srv, c) udp_received(srv, c) end )
srv:listen(port)


