LED_PIN = 7
US_TO_MS = 1000
delay = 100
gpio.mode(LED_PIN, gpio.OUTPUT)

while true do
   gpio.write(LED_PIN, gpio.HIGH)
   tmr.delay(delay * US_TO_MS)
   gpio.write(LED_PIN, gpio.LOW)
   tmr.delay(delay * US_TO_MS)
end
