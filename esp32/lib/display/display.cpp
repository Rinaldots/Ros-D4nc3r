#include "display.h"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setupDisplay(){

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x64
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10,10);
  display.setTextColor(SSD1306_WHITE);
  display.print("MicroROS");
  display.drawRect(5,9,120,20,WHITE);
  display.setCursor(10, 40);   
  display.setTextSize(1.8);       
  display.print("10 Dimensoes");
  display.display();
  delay(2000);
}

void displayLineFollowing(int &lpwm,int &rpwm){
    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0,0);
    display.setTextColor(SSD1306_WHITE);
    display.print("lpwm :");
    display.println(lpwm);
    display.print("rpwm :");
    display.println(rpwm);
    display.display();
}

void drawRightArrow() {
      display.clearDisplay();
    for (int i = -1; i <= 1; i++) {
      display.drawLine(30, 15 + i, 80, 15 + i, WHITE);
    }
    // Upper diagonal
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        display.drawLine(80, 15, 60 + i, 10 + j, WHITE);
      }
    }
    // Lower diagonal
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        display.drawLine(80, 15, 60 + i, 20 + j, WHITE);
      }
    }
    display.display();
  }

void drawLeftArrow() {
    display.clearDisplay();

    for (int i = -1; i <= 1; i++) {
      display.drawLine(80, 15 + i, 30, 15 + i, WHITE);
    }
    // Upper diagonal
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        display.drawLine(30, 15, 50 + i, 10 + j, WHITE);
      }
    }
    // Lower diagonal
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        display.drawLine(30, 15, 50 + i, 20 + j, WHITE);
      }
    }
    display.display();
  }

void drawForwardArrow() {
    display.clearDisplay();

    // up
    for (int i = -1; i <= 1; i++) {
      display.drawLine(55 + i, 5, 55 + i, 25, WHITE);
    }
    // Left diagonal
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        display.drawLine(45, 15, 55 + i, 5 + j, WHITE);
      }
    }
    // Right diagonal
    for (int i = -1; i <= 1; i++) {
      for (int j = -1; j <= 1; j++) {
        display.drawLine(65, 15, 55 + i, 5 + j, WHITE);
      }
    }
    display.display();
  }

void drawBackwardArrow() {
        display.clearDisplay();
    // Vertical line
    for (int i = -1; i <= 1; i++) {display.drawLine(55 + i, 5, 55 + i, 20, WHITE);}
    // Left diagonal
    for (int i = -1; i <= 1; i++) {for (int j = -1; j <= 1; j++) {display.drawLine(45, 15, 55 + i, 20 + j, WHITE);}}
    // Right diagonal
    for (int i = -1; i <= 1; i++) {for (int j = -1; j <= 1; j++) {display.drawLine(65, 15, 55 + i, 20 + j, WHITE);}}
    display.display();
  }

void drawStopCircle() {
    display.clearDisplay();
    for (int i = -1; i <= 1; i++) {display.drawCircle(64, 15, 10 + i, WHITE);}
    display.display();
  }

void drawTopic(uint8_t arraytopic_bit_map[1024]){
    display.clearDisplay();
    display.drawBitmap(0,0,arraytopic_bit_map,128,64,WHITE);
    display.display();
}

void custom_draw(int number){
  //coloque scripts aqui
  switch (number) {
  case 1:
    drawForwardArrow();
    
    break;
  case 2:
    drawBackwardArrow();
    break;
  case 3:
    drawLeftArrow();
    break;
  case 4:
    drawRightArrow();
    break;
  case 5:
    // comando(s)
    break;
  default:
    // comando(s)
    break;
}
}

