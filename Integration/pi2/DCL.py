from RPLCD.i2c import CharLCD
import time
lcd = CharLCD(i2c_expander='PCF8574', address=0x27, port=1, cols=20, rows=4)

lcd.clear()

lcd.cursor_pos = (0,0)
lcd.write_string("Row 0, Col 0")

lcd.cursor_pos = (1, 0)
lcd.write_string("Row 1, Col 0")

lcd.cursor_pos = (2, 0)
lcd.write_string("Row 2, Col 0")

lcd.cursor_pos = (3, 0)
lcd.write_string("Row 3, Col 0")
