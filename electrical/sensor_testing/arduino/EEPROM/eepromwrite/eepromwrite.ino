// Device pin connections to Arduino Uno:
// 1 = CS (Chip Select) -> D10
// 2 = SO (Master In Slave Out) -> D12
// 3 = WP -> GND
// 4 = 0V (Ground)-> GND
// 5 = SI (Master Out Slave In) -> D11
// 6 = SCK -> D13
// 7 = HOLD -> 3.3V
// 8 = Vcc -> 3.3V

// 5V also works

#include <SPI.h>

#define CS_PIN 10
#define EEPROM_SIZE 131072 

unsigned long usedBytes = 0;
unsigned long lastWriteTime = 0;
const unsigned long writeInterval = 100; // ms
bool generating = false;

// --- EEPROM SPI Functions ---
void writeEnable() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x06);
  digitalWrite(CS_PIN, HIGH);
}

byte readStatus() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x05);
  byte status = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return status;
}

void waitForWriteComplete() {
  while (readStatus() & 0x01) delay(1);
}

void writeByte(unsigned long addr, byte data) {
  writeEnable();
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x02);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
  waitForWriteComplete();
}

byte readByte(unsigned long addr) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x03);
  SPI.transfer((addr >> 16) & 0xFF);
  SPI.transfer((addr >> 8) & 0xFF);
  SPI.transfer(addr & 0xFF);
  byte val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return val;
}

// --- Header Management ---
void saveUsedBytes() {
  for (int i = 0; i < 4; i++)
    writeByte(i, (usedBytes >> (8 * (3 - i))) & 0xFF);
}

void loadUsedBytes() {
  usedBytes = 0;
  for (int i = 0; i < 4; i++)
    usedBytes = (usedBytes << 8) | readByte(i);
  if (usedBytes > EEPROM_SIZE) usedBytes = 4;
}

// --- EEPROM Utilities ---
void eraseEEPROM() {
  Serial.println("Erasing EEPROM...");
  writeEnable();
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0xC7); 
  digitalWrite(CS_PIN, HIGH);
  while (readStatus() & 0x01) { delay(100); Serial.print("."); }
  Serial.println("\nErase complete!");
  usedBytes = 4;
  saveUsedBytes();
}

void writeString(const String &s) {
  for (int i = 0; i < s.length(); i++) {
    if (usedBytes < EEPROM_SIZE)
      writeByte(usedBytes++, s[i]);
    else {
      Serial.println("EEPROM FULL!");
      return;
    }
  }
  saveUsedBytes();
}

// --- File Handling ---
int findLastFileIndex() {
  int maxIndex = 0;
  String pattern = "Flight_Data_";
  unsigned long addr = 4;
  while (addr < usedBytes) {
    byte b = readByte(addr++);
    if (b != '\n') continue;
    String check = "";
    for (int i = 0; i < pattern.length() && (addr + i) < usedBytes; i++)
      check += (char)readByte(addr + i);
    if (check == pattern) {
      int j = addr + pattern.length();
      String numStr = "";
      while (j < usedBytes) {
        char c = (char)readByte(j++);
        if (c < '0' || c > '9') break;
        numStr += c;
      }
      int val = numStr.toInt();
      if (val > maxIndex) maxIndex = val;
    }
  }
  return maxIndex;
}

void createNewFile() {
  int newFileNum = findLastFileIndex() + 1;
  String header = "\nFlight_Data_" + String(newFileNum) + "\n";
  writeString(header);
  Serial.print("Started new file: ");
  Serial.println(header);
}

void writeRandomRow(int numInts, int maxVal) {
  String row = "";
  for (int i = 0; i < numInts; i++) {
    row += String(random(maxVal));
    if (i < numInts - 1) row += ",";
  }
  row += "\n";
  writeString(row);
  Serial.print("Wrote row: ");
  Serial.println(row);
}

// --- Setup & Loop ---
void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  randomSeed(analogRead(0));

  loadUsedBytes();

  Serial.println("=== EEPROM Logger ===");
  Serial.print("Used bytes: "); Serial.println(usedBytes);

  Serial.print("Erase all files before starting new run? (y/n): ");
  while (!Serial.available());
  char c = Serial.read();
  if (c == 'y' || c == 'Y') eraseEEPROM();

  createNewFile();
}

void loop() {
  if (generating && millis() - lastWriteTime >= writeInterval && usedBytes < EEPROM_SIZE) {
    lastWriteTime = millis();
    writeRandomRow(5, 1000);
  }

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "g") {
      generating = !generating;
      Serial.print("Generation ");
      Serial.println(generating ? "ON" : "OFF");
    } else if (cmd == "e") {
      eraseEEPROM();
    }
  }
}