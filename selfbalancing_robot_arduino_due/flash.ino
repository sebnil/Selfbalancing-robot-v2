#define DATA_LENGTH   ((IFLASH1_PAGE_SIZE/sizeof(byte)) * 5)
#define  FLASH_START  ((byte *)IFLASH1_ADDR + 4)

byte testWriteData[DATA_LENGTH];


void saveConfigurationToFlash() {
  for (int i = 0; i < DATA_LENGTH; i++)
    testWriteData[i] = i;
}


void writeData() {
//  flash.write(FLASH_START, testWriteData, DATA_LENGTH);
  Serial.println("\nFlash data written");
}

void verifyData() {
  int errCount = 0;

  for (int i = 0; i < DATA_LENGTH; i++) {
    byte readB = FLASH_START[i];
    Serial.print(readB);
  }
  Serial.println();
}


