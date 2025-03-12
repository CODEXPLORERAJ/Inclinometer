#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <EEPROM.h>
#include <Fonts/FreeMono9pt7b.h>

// SSD1306 OLED
const byte SCREEN_WIDTH = 128;
const byte SCREEN_HEIGHT = 64;
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// Menu & menu button definitions
const byte MENU_BTN = 3;
const byte ENTER_BTN = 4;
unsigned long menuStartTime = 0;
byte menuItem = 0;
const int menuTimeout = 10000;
bool precisionMode = false;

// MPU Address for I2C
byte devAddr = 0x68;
MPU6050 mpu(devAddr);

// MPU control/status vars
bool dmpReady = false;
byte mpuIntStatus;
byte devStatus;
int packetSize;
int fifoCount;
byte fifoBuffer[64];
Quaternion q;

// New variables for sliding text
const String createdByText = "Created By RoboVate";
int textX = SCREEN_WIDTH;  // Start position for sliding text
const int textSpeed = 2;   // Pixels to move per frame

// Function to show loading animation
void showLoadingScreen() {
  display.clearDisplay();
  
  // Show sliding text
  display.setTextSize(1);
  display.setFont();
  
  // Slide text until it's off screen
  while (textX > -120) {  // -120 ensures text moves fully off screen
    display.clearDisplay();
    display.setCursor(textX, 20);
    display.println(createdByText);
    
    // Draw loading bar frame
    display.drawRect(14, 40, 100, 10, WHITE);
    
    // Calculate loading bar progress based on text position
    int progress = map(textX, SCREEN_WIDTH, -120, 0, 96);
    if (progress > 0) {
      display.fillRect(16, 42, min(96, progress), 6, WHITE);
    }
    
    display.display();
    textX -= textSpeed;
    delay(50);
  }
  
  // Show 100% loaded state briefly
  display.clearDisplay();
  display.setCursor(0, 20);
  display.println(createdByText);
  display.drawRect(14, 40, 100, 10, WHITE);
  display.fillRect(16, 42, 96, 6, WHITE);
  display.display();
  delay(500);
}

// Quaternion to Euler Angle conversion
VectorFloat QtoEulerAngle(Quaternion qt) {
  VectorFloat ret;
  double sqw = qt.w * qt.w;
  double sqx = qt.x * qt.x;
  double sqy = qt.y * qt.y;
  double sqz = qt.z * qt.z;

  ret.x = atan2(2.0 * (qt.x * qt.y + qt.z * qt.w), (sqx - sqy - sqz + sqw));
  ret.y = asin(2.0 * (qt.x * qt.z - qt.y * qt.w) / (sqx + sqy + sqz + sqw));
  ret.z = atan2(2.0 * (qt.y * qt.z + qt.x * qt.w), (-sqx - sqy + sqz + sqw));

  ret.x = ret.x * 180 / PI;
  ret.y = ret.y * 180 / PI;
  ret.z = ret.z * 180 / PI;
  return ret;
}

void epromWriteWord(int startAddr, int value) {
  EEPROM.update(startAddr, value);
  EEPROM.update(startAddr+1, value >> 8);
}

int epromReadWord(int startAddr) {
  int value = EEPROM.read(startAddr);
  value = value | (EEPROM.read(startAddr+1) << 8);
  return value;
}

void getCalibration() {
  mpu.setXAccelOffset(epromReadWord(0));
  mpu.setYAccelOffset(epromReadWord(2));
  mpu.setZAccelOffset(epromReadWord(4));
  mpu.setXGyroOffset(epromReadWord(6));
  mpu.setYGyroOffset(epromReadWord(8));
  mpu.setZGyroOffset(epromReadWord(10));
}

void setCalibration() {
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);

  int Data[3];
  I2Cdev::readWords(devAddr, 0x06, 3, (int *)Data);
  epromWriteWord(0,Data[0]);
  epromWriteWord(2,Data[1]);
  epromWriteWord(4,Data[2]);
  I2Cdev::readWords(devAddr, 0x13, 3, (int *)Data);
  epromWriteWord(6,Data[0]);
  epromWriteWord(8,Data[1]);
  epromWriteWord(10,Data[2]);
}

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  Serial.begin(115200);

  pinMode(MENU_BTN, INPUT_PULLUP);
  pinMode(ENTER_BTN, INPUT_PULLUP);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  
  // Show loading animation
  showLoadingScreen();

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  getCalibration();

  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    display.clearDisplay();
    display.setCursor(0, 40);
    display.println("IMU FAIL");
    display.display();
  }
}

// [Rest of the code remains exactly the same, including all menu functions and the main loop]
void dispMenu(byte itemSelect) {
  display.clearDisplay();
  display.setRotation(0);
  display.setFont(&FreeMono9pt7b);
  display.setCursor(0, 14);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(F("   Normal"));
  display.println(F(" Precision"));
  display.println(F(" Calibrate"));

  if (itemSelect == 0) {
    display.drawRect(5, 2, 120, 17, SSD1306_WHITE);
  }
  if (itemSelect == 1) {
    display.drawRect(5, 18, 120, 20, SSD1306_WHITE);
  }
  if (itemSelect == 2) {
    display.drawRect(5, 36, 120, 20, SSD1306_WHITE);
  }

  display.display();
}

void dispCalibrate(byte itemSelect) {
  display.clearDisplay();
  display.setFont(&FreeMono9pt7b);
  display.setCursor(2, 14);
  display.setTextColor(WHITE);
  display.setTextSize(1);
  display.println(F("Lay me down"));
  display.println(F("  face up"));
  display.setCursor(0, 55);
  display.println(F(" START   X"));

  if (itemSelect == 0) {
    display.drawRect(6, 41, 65, 20, SSD1306_WHITE);
  }
  else  {
    display.drawRect(95, 41, 20, 20, SSD1306_WHITE);
  }

  display.display();
}

void menuMainWait() {
  menuItem = 0;
  dispMenu(menuItem);

  menuStartTime = millis();

  while (millis() - menuStartTime <= menuTimeout) {
    if (digitalRead(MENU_BTN) == LOW) {
      delay(200);
      menuStartTime = millis();
      menuItem++;
      if (menuItem >= 3) {
        menuItem = 0;
        dispMenu(menuItem);
      }
      else {
        dispMenu(menuItem);
      }
    }
    if (digitalRead(ENTER_BTN) == LOW) {
      delay(200);
      menuStartTime = millis();
      if (menuItem == 0) {
        precisionMode = false;
        break;
      }
      if (menuItem == 1) {
        precisionMode = true;
        break;
      }
      if (menuItem == 2) {
        menuCalibrateWait();
        break;
      }
    }
  }
}

void menuCalibrateWait() {
  menuItem = 1;
  dispCalibrate(menuItem);

  menuStartTime = millis();
  while (millis() - menuStartTime <= menuTimeout) {
    if (digitalRead(MENU_BTN) == LOW) {
      delay(300);
      if (menuItem == 1) {
        menuItem = 0;
        dispCalibrate(menuItem);
      }
      else {
        menuItem = 1;
        dispCalibrate(menuItem);
      }
    }
    if (digitalRead(ENTER_BTN) == LOW) {
      delay(200);
      if (menuItem == 0) {
        display.clearDisplay();
        display.setCursor(0, 40);
        display.println("CALIBRATING");
        display.display();

        setCalibration();
        
        display.clearDisplay();
        display.setCursor(0, 40);
        display.println("COMPLETE!");
        display.display();
        delay(500);
        break;
      }
      if (menuItem == 1) {
        break;
      }
    }
  }
}

void formatDisplay(double angleVal, byte dispRotate) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setFont();
  display.setRotation(dispRotate);

  if (precisionMode == false) {
    if (dispRotate == 0 || dispRotate == 2) {
      display.setCursor(40, 10);
    }
    else {
      display.setCursor(3, 30);
    }

    display.setTextSize(5);
    display.println(round(abs(angleVal)));
  }
  else {
    if (dispRotate == 0 || dispRotate == 2) {
      display.setTextSize(4);
      display.setCursor(20, 10);
    }
    else {
      display.setTextSize(2);
      display.setCursor(6, 40);
    }

    display.println(abs(angleVal), 1);
  }

  if (round(abs(angleVal)) <= 1) {
    display.setTextSize(1);
    display.setTextColor(BLACK, WHITE);
    if (dispRotate == 0 || dispRotate == 2) {
      display.setCursor(37, 50);
      display.println(F("   LEVEL  "));
    }
    else {
      display.setCursor(0, 90);
      display.println(F("   PLUMB  "));
    }
  }

  display.setTextColor(WHITE, BLACK);
  display.display();
}

void loop() {
  if (digitalRead(MENU_BTN) == LOW) {
    delay(500);
    menuMainWait();
  }
  if (!dmpReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    VectorFloat ea = QtoEulerAngle(q);
    
    float angVal = 0;
    float dispRotate = 0;

    if (ea.x > 0 && ea.y > 35 && ea.y <= 90) {
      angVal = (90 - ea.y);
      dispRotate = 0;
    }
    if (ea.x > 0 && ea.y <= 35 && ea.y > -35) {
      angVal = ea.y;
      dispRotate = 1;
    }
    if (ea.x > 0 && ea.y <= -35 && ea.y > -90) {
      angVal = ea.y + 90;
      dispRotate = 2;
    }
    if (ea.x < 0 && ea.y > 35 && ea.y <= 90) {
      angVal = (90 - ea.y);
      dispRotate = 0;
    }
    if (ea.x < 0 && ea.y <= 35 && ea.y > -35) {
      angVal = ea.y;
      dispRotate = 3;
    }
    if (ea.x < 0 && ea.y <= -35 && ea.y > -90) {
      angVal = ea.y + 90;
      dispRotate = 2;
    }
    if (ea.x < 5 && ea.x > -20 && ea.y <= 35 && ea.y > -35 && ea.z < 5) {
      angVal = 90 - ea.y;
      if (angVal > 50) {
        angVal -= 90;
      }
      dispRotate = 0;
    }

    formatDisplay(angVal, dispRotate);
  }
}