import lcdapi as lcd
import time

time_h = 15
time_m = 5
time_s = 2

alt = 4000
counter = 0

lcd.init("COM10")
lcd.switch_line(0)
lcd.clear_screen()

while True:
    time_s = time_s + 1
    if time_s == 60:
        time_s = 0
        time_m = time_m + 1
    counter = counter + 1
    alt = int(round(0.25*(counter**2) + 9990))
    lcd.switch_line(0)
    lcd.print_lcd("OS42-1    "+str(alt)+"m")
    lcd.switch_line(1)
    lcd.print_lcd(str(time_h)+":"+str(time_m)+":"+str(time_s)+"  SYS OK")
    time.sleep(1)
    #lcd.clear_screen()
    if counter % 10 == 0:
        lcd.clear_screen()
