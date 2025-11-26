#include <SPI.h>
#include <mcp_can.h>

#define TF_RX 16
#define TF_TX 17
#define SPI_CS_PIN 5

HardwareSerial tfSerial(2);
MCP_CAN CAN(SPI_CS_PIN);

// Buffer for TF-Luna frame
uint8_t tfBuffer[9];
uint8_t tfIndex = 0;

void setup() {
  Serial.begin(115200);
  tfSerial.begin(115200, SERIAL_8N1, TF_RX, TF_TX);
  SPI.begin();

  while (CAN_OK != CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ)) {
    Serial.println("ESP32: CAN init failed, retrying...");
    delay(500);
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println("ESP32: CAN Sender Ready!");
}

void loop() {
  uint16_t distance = readTFLuna();

  if (distance > 0 && distance < 10000) {  // Sanity check
    static byte data[2];  // Use static to avoid stack churn
    data[0] = highByte(distance);
    data[1] = lowByte(distance);

    byte result = CAN.sendMsgBuf(0x321, 0, 2, data);

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      Serial.print("Sending distance: ");
      Serial.println(distance);
      lastPrint = millis();
    }

    if (result != CAN_OK) {
      Serial.print("CAN Send Error: ");
      Serial.println(result);
    }
  }
}

// Optimized TF-Luna data reader
uint16_t readTFLuna() {
  while (tfSerial.available()) {
    uint8_t b = tfSerial.read();

    // Frame sync: look for 0x59 0x59
    if (tfIndex == 0 && b != 0x59) continue;
    if (tfIndex == 1 && b != 0x59) { tfIndex = 0; continue; }

    tfBuffer[tfIndex++] = b;

    if (tfIndex == 9) {
      tfIndex = 0;
      uint8_t checksum = 0;
      for (int i = 0; i < 8; i++) checksum += tfBuffer[i];

      if (checksum == tfBuffer[8]) {
        return tfBuffer[2] + (tfBuffer[3] << 8);  // Distance (little endian)
      }
    }
  }

  return 0;  // No valid frame
}
