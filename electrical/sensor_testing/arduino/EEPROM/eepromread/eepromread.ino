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

void loadUsedBytes() {
  usedBytes = 0;
  for (int i = 0; i < 4; i++)
    usedBytes = (usedBytes << 8) | readByte(i);
  if (usedBytes > EEPROM_SIZE) usedBytes = 4;
}

// --- File handling ---
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

void readFile(int fileNum) {
  String header = "Flight_Data_" + String(fileNum);
  bool reading = false;
  String line = "";

  for (unsigned long addr = 4; addr < usedBytes; addr++) {
    char c = (char)readByte(addr);

    if (!reading) {
      String check = "";
      for (int j = 0; j < header.length() && (addr + j) < usedBytes; j++)
        check += (char)readByte(addr + j);
      if (check == header) {
        reading = true;
        addr += header.length();
        if (readByte(addr) == '\n') addr++; // skip newline
        continue;
      }
    } else {
      if (c == '\n') {
        if (line.length() > 0) Serial.println(line);
        line = "";
      } else line += c;
    }
  }

  if (!reading) Serial.println("File not found.");
  else if (line.length() > 0) Serial.println(line);
}

// --- Setup & Loop ---
void setup() {
  Serial.begin(115200);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  loadUsedBytes();

  Serial.println("=== EEPROM Reader ===");
  Serial.println("Commands:");
  Serial.println("  r -> read most recent file");
  Serial.println("  sX -> read selected file X (e.g., s3)");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "r") {
      int last = findLastFileIndex();
      if (last == 0) Serial.println("No files found.");
      else readFile(last);
    } else if (cmd.startsWith("s")) {
      int sel = cmd.substring(1).toInt();
      readFile(sel);
    }
  }
}