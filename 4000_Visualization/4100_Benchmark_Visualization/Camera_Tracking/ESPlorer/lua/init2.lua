LED_PIN = 7
US_TO_MS = 1
delay = 1000
gpio.mode(LED_PIN, gpio.OUTPUT)

while true do
   gpio.write(LED_PIN, gpio.HIGH)
   tmr.delay(delay * US_TO_MS)
   gpio.write(LED_PIN, gpio.LOW)
   tmr.delay(delay * US_TO_MS)
   delay = delay -1
   if delay < 10 then
   delay=5000
   end
end
