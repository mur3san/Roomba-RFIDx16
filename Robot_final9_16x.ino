#include <SPI.h>
#include <MFRC522.h>
#include <NewPing.h>
#include <Wire.h>
#include <HMC5883L.h>
#include <MPU6050.h>
HMC5883L compass;
MPU6050 mpu;
int ScoutCounter=0; // counter pentru LocalizareRfid
float N_min,N_max,W_min,W_max,S_min,S_max,E_min,E_max,N_avg,S_avg,E_avg,W_avg;
#define Right90Degree 85
#define Left90Degree 86
#define Catchup 600 // delay value
#define SIZE 16
#define SPEED 180  // viteza roomba
#define TURNSPEED 150 
#define MAX_DISTANCE 30
#define Travel_distance 50
#define Resolution 15 // Rezolutia pentru Magnetometru / Corectarea unghiului
int counter=0;    // counter pentru detectarea tipului de obiect mobil/fix 
int pozitie=1; //Nord=1 Sud=2 Est=3 Vest=4
int Array[SIZE][SIZE];
int x=0,y=0,DestinatieX=0,DestinatieY=0;
int k=0,k1=0,k2=0,k3=0,k4=0,k5=0,k6=0,k7=0,k8=0,k9=0,k10=0,k11=0,k12=0,k13=0,k14=0,k15=0;
int solution[SIZE][SIZE];
constexpr uint8_t RST_PIN = 5;     // Configurable *schimba pt Mega
constexpr uint8_t SS_PIN = 53;     // Configurable 
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
MFRC522::MIFARE_Key key;
NewPing Usenzor1(10, 9, MAX_DISTANCE);  
NewPing Usenzor2(35, 34, MAX_DISTANCE);
void setup() {
 Serial3.begin(57600);// robot serial
 Serial1.begin(9600); //Bluetooth serial
 Serial.begin(9600);
 SPI.begin();
 mfrc522.PCD_Init();
 mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
 for (byte i = 0; i < 6; i++) {
        key.keyByte[i] = 0xFF;
    }
    while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  mpu.setI2CMasterModeEnabled(false);
  mpu.setI2CBypassEnabled(true) ;
  mpu.setSleepEnabled(false);

  // Initialize Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  // Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  // Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  // Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  // Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0);
 delay(3000);
 startSafe();
 Serial.println("Setting up Cardinal POints");
  SetCardinalPoints(Resolution);
}
void loop() {
  Go();
}
void startSafe(){
  Serial3.write((byte)128); // START
  Serial3.write((byte)132); // SAFE MODE
  delay(2000);
  
}
void RfidRead(){
  Serial.println("Apropie Card:");
  if ( ! mfrc522.PICC_IsNewCardPresent())
        return;
  if ( ! mfrc522.PICC_ReadCardSerial())
        return;
        // Show some details of the PICC (that is: the tag/card)
    Serial.print(F("Card UID:"));
    dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);
    Serial.println();
    Serial.print(F("PICC type: "));
    MFRC522::PICC_Type piccType = mfrc522.PICC_GetType(mfrc522.uid.sak);
    Serial.println(mfrc522.PICC_GetTypeName(piccType));
        // Check for compatibility
 if (    piccType != MFRC522::PICC_TYPE_MIFARE_MINI
        &&  piccType != MFRC522::PICC_TYPE_MIFARE_1K
        &&  piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
        Serial.println(F("This sample only works with MIFARE Classic cards."));
        return;
    }
     byte sector2        = 2;
     byte sector3        = 3;
     byte sector4        = 4;
     byte sector5        = 5;
     byte sector6        = 6;
     byte sector7        = 7;
     byte blockAddr      = 8;
     byte blockAddr5     = 9;
     byte blockAddr6     = 10;
     byte blockAddr7     = 12;
     byte blockAddr8     = 13;
     byte blockAddr9     = 14;
     byte blockAddr10    = 16;
     byte blockAddr11    = 17;
     byte blockAddr12    = 18;
     byte blockAddr13    = 20;
     byte blockAddr14    = 21;
     byte blockAddr15    = 22;
     byte blockAddr16    = 24;
     byte blockAddr17    = 25;
     byte blockAddr18    = 26;
     byte blockAddr19    = 28;
     byte blockAddrXY    = 29;
     byte blockAddrFN    = 30;
     byte trailerBlock   = 11;
     byte trailerBlock1  = 15;
     byte trailerBlock2  = 19;
     byte trailerBlock3  = 23;
     byte trailerBlock4  = 27;
     byte trailerBlock5  = 31;

    MFRC522::StatusCode status;
    byte buffer[18];
    byte size = sizeof(buffer);
     // Open Sector 7
     Serial.println(F("Current data in sector2 :"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector7);
    Serial.println();
  
     // Read BlockAddrXY
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddrXY, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
     x=buffer[0];
     y=buffer[1];
     Serial.println(x);
     Serial.println(y);  
     // Read BlockAddrFN
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddrFN, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }

     DestinatieX=buffer[0];
     DestinatieY=buffer[1];
     Serial.println(DestinatieX);
     Serial.println(DestinatieY);  
      // Read BlockAddr19
     status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr19, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[0][j]=buffer[k1];
     k1++;  
  }
  // Open Sector 6
     Serial.println(F("Current data in sector2 :"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector6);
    Serial.println();
  
     // Read BlockAddr18
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr18, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[1][j]=buffer[k2];
     k2++;  
  }
  // Read BlockAddr17
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr17, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[2][j]=buffer[k3];
     k3++;  
  }
  // Read BlockAddr16
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr16, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[3][j]=buffer[k4];
     k4++;  
  }
  // Open Sector 5
     Serial.println(F("Current data in sector2 :"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector5);
    Serial.println();
  
     // Read BlockAddr15
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr15, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[4][j]=buffer[k5];
     k5++;  
  }
  // Read BlockAddr14
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr14, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[5][j]=buffer[k6];
     k6++;  
  }
  // Read BlockAddr13
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr13, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[6][j]=buffer[k7];
     k7++;  
  }
  // Open Sector 4
     Serial.println(F("Current data in sector2 :"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector4);
    Serial.println();
  
     // Read BlockAddr12
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr12, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[7][j]=buffer[k8];
     k8++;  
  }
  // Read BlockAddr11
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr11, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[8][j]=buffer[k9];
     k9++;  
  }
  // Read BlockAddr10
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr10, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[9][j]=buffer[k10];
     k10++;  
  }
  // Open Sector 3
    Serial.println(F("Current data in sector3 :"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector3 );
    Serial.println();
    // Read BlockAddr9
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr9, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
     for(int j=0;j<SIZE;j++){
     Array[10][j]=buffer[k11];
     k11++;  
  }
     // Read BlockAddr8
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr8, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[11][j]=buffer[k12];
     k12++;  
  }
 // Read BlockAddr7
    status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr7, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[12][j]=buffer[k13];
     k13++;  
  }
    // Open Sector 2
     Serial.println(F("Current data in sector2 :"));
    mfrc522.PICC_DumpMifareClassicSectorToSerial(&(mfrc522.uid), &key, sector2);
    Serial.println();
  
     // Read BlockAddr6
    
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr6, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[13][j]=buffer[k14];
     k14++;  
  }
  // Read BlockAddr5
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr5, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[14][j]=buffer[k15];
     k15++;  
  }
  // Read BlockAddr
  status = (MFRC522::StatusCode) mfrc522.MIFARE_Read(blockAddr, buffer, &size);
    if (status != MFRC522::STATUS_OK) {
        Serial.print(F("MIFARE_Read() failed: "));
        Serial.println(mfrc522.GetStatusCodeName(status));
    }
    for(int j=0;j<SIZE;j++){
     Array[15][j]=buffer[k];
     k++;  
  }
  //x=y=0; // temporary fix(valorile pentru x,y trebuie scrise in Rfid Tag)
  k=k1=k2=k3=k4=k5=k6=k7=k8=k9=k10=k11=k12=k13=k14=k15=0;
  for(int i=0 ; i<SIZE;i++){
    Serial.println();  
    for(int j=0;j<SIZE;j++){
     Serial.print(Array[i][j]);
     Serial.print(" ");     
  }}
  Serial.println();
  // Halt PICC
  mfrc522.PICC_HaltA();
  // Stop encryption on PCD
  mfrc522.PCD_StopCrypto1();
  sol();
}
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        Serial.print(buffer[i] < 0x10 ? " 0" : " ");
        Serial.print(buffer[i], HEX);
    }
}
boolean CheckNorth(){
  int nextNumber = solution[x+1][y];
  if(nextNumber==1 && x+1 < SIZE || nextNumber==2 && x+1 < SIZE ){
    return true;
  } else {
    Serial.println("North false");
    return false;
  }
}
boolean CheckSouth(){
  int nextNumber = solution[x-1][y];
  if(nextNumber==1 && x-1 >= 0 || nextNumber==2 && x-1 >= 0 ){
    Serial.println("South true");
    return true;
  } else {
    Serial.println("South false");
    return false;
  }
}
boolean CheckWest(){
  int nextNumber = solution[x][y+1];  
  if(nextNumber == 1 && y+1 < SIZE || nextNumber == 2 && y+1 < SIZE ){
    Serial.println(" West true");
    return true;
  } else {
    Serial.println(" West false");
    return false;
  }
}
boolean CheckEast(){
  int nextNumber = solution[x][y-1];
  if(nextNumber == 1 && y-1 >= 0 || nextNumber == 2 && y-1 >= 0){
    Serial.println("East true");
 /* Serial.print("X=");
    Serial.println(x);
    Serial.print("Y=");
    Serial.println(y);  */
    return true;
  } else {
    Serial.println("East false");
    return false;
  }
}
boolean Checkfinish(){
  int nextNumber = Array[x][y];
  //if(nextNumber==Array[DestinatieX][DestinatieY]){
  if(x==DestinatieX && y==DestinatieY){
    Serial.println("finish true");
    return true;
  } else {
    Serial.println("finish false");
    return false;
  }
}

void TurnRight(int speed, int angle) {

  angle = -angle;

  Serial3.write((byte)137);
  Serial3.write((byte)(speed >> 8 & 0xFF));
  Serial3.write((byte)(speed & 0xFF));
  Serial3.write((byte)0xFF);
  Serial3.write((byte)0xFF);

  Serial3.write((byte)157);
  Serial3.write((byte)(angle >> 8 & 0xFF));
  Serial3.write((byte)(angle & 0xFF));
  delay(100);
  Serial3.write((byte)137);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
}
void TurnLeft(int speed, int angle) {

  Serial3.write((byte)137);
  Serial3.write((byte)(speed >> 8 & 0xFF));
  Serial3.write((byte)(speed & 0xFF));
  Serial3.write((byte)0);
  Serial3.write((byte)1);
  
  Serial3.write((byte)157);
  Serial3.write((byte)(angle >> 8 & 0xFF));
  Serial3.write((byte)(angle & 0xFF));
  delay(100);
  Serial3.write((byte)137);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
}
bool MoveForward(int speed, int distance) {  //merge
  if(Usenzori()==true){
    return false;
  }else
  Serial3.write((byte)137);
  Serial3.write((byte)(speed >> 8 & 0xFF));
  Serial3.write((byte)(speed & 0xFF));
  Serial3.write((byte)0x80);
  Serial3.write((byte)0);

  Serial3.write((byte)156);
  Serial3.write((byte)(distance >> 8 & 0xFF));
  Serial3.write((byte)(distance & 0xFF));
  delay(100);
  Serial3.write((byte)137);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  return true;
}
void Stop() {
  Serial3.write((byte)137);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
}
void MoveBackward(int speed, int distance) {   //merge
 
  speed = -speed;

  Serial3.write((byte)137);
  Serial3.write((byte)(speed >> 8 & 0xFF));
  Serial3.write((byte)(speed & 0xFF));
  Serial3.write((byte)0x80);
  Serial3.write((byte)0);

  distance = -distance;

  Serial3.write((byte)156);
  Serial3.write((byte)(distance >> 8 & 0xFF));
  Serial3.write((byte)(distance & 0xFF));
  delay(100);
  Serial3.write((byte)137);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
  Serial3.write((byte)0);
}
void song(){
  Serial3.write((byte)140);
  Serial3.write((byte)1);
  Serial3.write((byte)3);
  Serial3.write((byte)80);
  Serial3.write((byte)16);
  Serial3.write((byte)60);
  Serial3.write((byte)16);
  Serial3.write((byte)80);
  Serial3.write((byte)16);
  delay(100);
  Serial3.write((byte)141);
  Serial3.write((byte)1);
}
void printsolution()
{
    int i,j;
    for(i=0;i<SIZE;i++)
    {
        for(j=0;j<SIZE;j++)
        {
            Serial.print(solution[i][j]);
        }
        Serial.println();
    }
}
int solvemaze(int r, int c)
{
    if((r==DestinatieX) && (c==DestinatieY))
    {
        solution[r][c] = 1;
        return 1;
    }
    if(r>=0 && c>=0 && r<SIZE && c<SIZE && solution[r][c] == 0 && Array[r][c] == 0)
    {
        //if safe to visit then visit the cell
        solution[r][c] = 1;
        //going down
        if(solvemaze(r+1, c))
            return 1;
        //going right
        if(solvemaze(r, c+1))
            return 1;
        //going up
        if(solvemaze(r-1, c))
            return 1;
        //going left
        if(solvemaze(r, c-1))
            return 1;
        //backtracking
        solution[r][c] = 0;
        return 0;
    }
    return 0;

}
boolean sol(){
  int i1,j1;
    for(i1=0; i1<SIZE; i1++)
    {
        for(j1=0; j1<SIZE; j1++)
        {
            solution[i1][j1] = 0;
        }
    }
 
    if (solvemaze(x,y)){
      Serial.print("Xmaze=");
      Serial.print(x);
      Serial.println();
      Serial.print("Ymaze=");
      Serial.print(y);
      Serial.println();
        printsolution();
        return true;
    }
    else
        Serial.print("No solution\n");
        RfidRead();
        return false;
}
boolean Usenzori(){
    delay(50);
    if(Usenzor1.ping_cm()!= 0 || Usenzor2.ping_cm()!=0){
      Serial.println("Obiect detected");
      if(counter==5){
        Serial.println("Changing Path");
      switch(pozitie){
         case 1:
         Array[x+1][y]=1;
         break;
         case 2:
         Array[x-1][y]=1;
         break;
         case 3:
         Array[x][y-1]=1;
         break;
         case 4:
         Array[x][y+1]=1;
         break;
      }
      counter=0;
      } else
      counter++;
      return true;
    } else
    counter=0;
    Serial.println("Path Clear");
    return false;
  }
void Go(){
    if(sol()==true && Checkfinish()==false ){
      bool hasMovedForward = false;
      if(CheckNorth()==true){
       switch(Pozitie()){
        case 0:
        TurnRight(200,Resolution);
       // AngleCorrection();
        break;
         case 1:
          hasMovedForward = MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break;
        case 2:
          TurnLeft(TURNSPEED,180);
          Pozitie();
          delay(Catchup);
          hasMovedForward =  MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break;
        case 3:
          TurnLeft(TURNSPEED,Left90Degree);
          Pozitie();
          delay(Catchup);
          hasMovedForward = MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break;
        case 4:
          TurnRight(TURNSPEED,Right90Degree);
          Pozitie();
          delay(Catchup);
          hasMovedForward =  MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break; 
  }     
  if(hasMovedForward == false){
  return;
  } else
      //Array[x][y]=3;
      solution[x][y]=3;
      delay(100);
      if(x+1<SIZE)x++;
     }
    if(CheckSouth()==true && CheckNorth()==false ){
       switch(Pozitie()){
        case 0:
        TurnRight(200,Resolution);
        //AngleCorrection();
        break;
         case 1:
         TurnLeft(TURNSPEED,180);
         delay(Catchup);
         Pozitie();
         hasMovedForward = MoveForward(SPEED,Travel_distance);
         delay(Catchup);
         break;
        case 2:
         hasMovedForward = MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break;
        case 3:
          TurnRight(TURNSPEED,Right90Degree);
          Pozitie();
          delay(Catchup);
         hasMovedForward = MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break;
        case 4:
          TurnLeft(TURNSPEED,Left90Degree);
          Pozitie();
          delay(Catchup);
         hasMovedForward = MoveForward(SPEED,Travel_distance);
          delay(Catchup);
         break; 
  } 
  if(hasMovedForward == false){
  return;  
  } else 
      //Array[x][y]=3;
      solution[x][y]=3;
      delay(100);
      if(x>0)x--;
     }
     if(CheckWest()==true && CheckNorth()==false){
     switch(Pozitie()){
      case 0:
      TurnRight(200,Resolution);
       // AngleCorrection();
        break;
      case 1:
      TurnLeft(TURNSPEED,Left90Degree);
      Pozitie();
      delay(Catchup);
      hasMovedForward = MoveForward(SPEED,Travel_distance);
      delay(Catchup);
      break;
      case 2:
      TurnRight(TURNSPEED,Right90Degree);
      Pozitie();
      delay(Catchup);
      hasMovedForward = MoveForward(SPEED,Travel_distance);
      delay(Catchup);
      break;
      case 3:
      TurnLeft(TURNSPEED,180);
      Pozitie();
      delay(Catchup);
      hasMovedForward = MoveForward(SPEED,Travel_distance);
      delay(Catchup);
      break;
      case 4:
      hasMovedForward = MoveForward(SPEED,Travel_distance);
      delay(Catchup);
      break;
    }
    if(hasMovedForward == false){
  return;
    }else
    //Array[x][y]=3;
    solution[x][y]=3;
      delay(100);
      if(y<SIZE)y++;  
     }
     if(CheckEast()==true && CheckNorth()==false){
     switch(Pozitie()){
    case 0:
    TurnRight(200,Resolution);
    //AngleCorrection();
    break;
    case 1:
    TurnRight(TURNSPEED,Right90Degree);
    Pozitie();
    delay(Catchup);
    hasMovedForward = MoveForward(SPEED,Travel_distance);
    delay(Catchup);
    break;
    case 2:
    TurnLeft(TURNSPEED,Left90Degree);
    Pozitie();
    delay(Catchup);
    hasMovedForward = MoveForward(SPEED,Travel_distance);
    delay(Catchup);
    break;
    case 3:
    hasMovedForward = MoveForward(SPEED,Travel_distance);
    delay(Catchup);
    break;
    case 4:
    TurnRight(TURNSPEED,180);
    Pozitie();
    delay(Catchup);
    hasMovedForward = MoveForward(SPEED,Travel_distance);
    delay(Catchup);
    break;
  }
  if(hasMovedForward == false){
  return;
  }else
      //Array[x][y]=3;
      solution[x][y]=3;
      delay(100);
      if(y>0)y--;
     }
}else
ScoutCounter++;
if(ScoutCounter >=6){
 scoutRfid();
}else
 RfidRead();
}

int Pozitie(){
  if(N_min <= Compass() && Compass() < N_max){
    Serial.println( N_max);
    Serial.println( N_min);
    return 1;
  } else 
  if(W_min <= Compass() && Compass() < W_max){
    Serial.println( W_max);
    Serial.println( W_min);
    return 4;
  } else 
  if(S_min <= Compass() && Compass() < S_max){
    Serial.println( S_max);
  Serial.println( S_min);
    return 2;
  }
  if(E_min <= Compass() && Compass() < E_max){
    Serial.println( E_max);
    Serial.println( E_min);
    return 3;
  }
  return 0;
}
void SetCardinalPoints(int rezolutie){
  N_avg=Compass();
  N_min=Compass()-rezolutie;
  N_max=Compass()+rezolutie;
  Serial.println( N_max);
  Serial.println( N_min);
  TurnRight(200,Right90Degree);
  delay(2500);
  E_avg=Compass();
  E_min=Compass()-rezolutie;
  E_max=Compass()+rezolutie;
  Serial.println( E_max);
  Serial.println( E_min);
  TurnRight(200,Right90Degree);
  delay(2500);
  S_avg=Compass();
  S_min=Compass()-rezolutie;
  S_max=Compass()+rezolutie;
  Serial.println( S_max);
  Serial.println( S_min);
  TurnRight(200,Right90Degree);
  delay(2500);
  W_avg=Compass();
  W_min=Compass()-rezolutie;
  W_max=Compass()+rezolutie;
  Serial.println( W_max);
  Serial.println( W_min);
  TurnRight(200,Right90Degree); 
}

float Compass(){
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (5.0 + (43.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }
 
  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 
  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();
  delay(100);
  return headingDegrees;
}
void scoutRfid(){
  int p=20;
  while(! mfrc522.PICC_IsNewCardPresent()){
    Serial3.write((byte)145);
  Serial3.write((byte)(p >> 8 & 0xFF));
  Serial3.write((byte)(p & 0xFF));
  Serial3.write((byte)(p/(p/10) >> 8 & 0xFF));
  Serial3.write((byte)(p/(p/10) & 0xFF));
  if(p<101)p+=5;     
  }
  delay(50);
  Stop();
  ScoutCounter=0;
  return;
}
  
