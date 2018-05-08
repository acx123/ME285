#include <LiquidCrystal_I2C.h>
#include <stdio.h>

LiquidCrystal_I2C lcd(0x27,16,2)

int main() {
  lcd.begin();
  char line1 [16];
  char line2 [16];
  bool run = true;
  while(run) {
      scanf("%s\t%s", line1, line2);
      if(line1[0] == '\n')
      lcd.clear();
      lcd.set_cursor(0,0);
      lcd.print(line1);
      lcd.set_cursor(0,1);
      lcd.print(line2);
  }
}
