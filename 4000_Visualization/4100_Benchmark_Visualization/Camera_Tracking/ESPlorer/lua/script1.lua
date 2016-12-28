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
    until disp:nextPage() == false        
    
    tm = rtctime.epoch2cal(rtctime.get())
    time = string.format("%02d:%02d:%02d", tm["hour"]+1, tm["min"],tm["sec"])
    tmr.start(0)
end

string1 =""
string2 =""
ip = ""
time=""

init_i2c_display()
--init_spi_display()
prepare()

wifi.setmode(wifi.STATION)
wifi.sta.config("Internet","ghetto123")
wifi.sta.connect()
tmr.alarm(1, 1000, 1, function()
     if wifi.sta.getip() == nil then
         string1 = "Connecting..."
         string2 = ""
     else
         tmr.stop(1)
         string1 = "Connected to:"
         string2 = "Internet"
         ip = wifi.sta.getip()
         sntp.sync("pool.ntp.org", function()
            print("Done")
           
        end)
     end
end)


-- set up timer 0 with short interval, will be retriggered in graphics_test()
tmr.register(0, 100, tmr.ALARM_SEMI, function() graphics_test() end)

print("--- Starting Graphics Test ---")
tmr.start(0)
