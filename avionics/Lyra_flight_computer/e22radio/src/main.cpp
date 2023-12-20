#include <Arduino.h>
#include <E220.h>
#include <macros.h>
#include <e22222.h>

#define M0 3
#define M1 2
#define AUX 4


Stream &radioserial = (Stream &)Serial1;
                    // m0m1aux
E220 ebyte(&radioserial,M0,M1,AUX);

e22simple ebytesimple;

uint32_t ledtime = 0;
bool ledstate = true;

uint32_t packettime = 0;
uint32_t senddatatime = 0;

telepacket currentpacket;

bool senddata = false;


telepacket recivepacket(){
  telepacket downpacket;
  if (!Serial1.available())
  {
    Serial1.println("no packet");
    return downpacket;
  }
  if (Serial1.peek() != 0x12)
  {
    Serial.printf("invalid packet, expected 0x12 got 0x%x \n",Serial1.read());
    return downpacket;
  }
  uint8_t databuf[32] = {};

  Serial1.readBytes(databuf,sizeof(databuf));

  if (databuf[0] != 0x12 && databuf[31] != 0x34)
  {
    Serial.println("invalid packet, markers didnt pass");
    return downpacket;
  }

  Serial.println("good data packet");
  int j = 0;
  for (int i = 0; i < 32; i++)
  {
    downpacket.data[j] = databuf[j];
    j++;
  }
  packettime = millis();
  return downpacket;
}

void senddatatoserial(telepacket data){
  Serial.printf("AB,%f,%f,%f"//accel
  ",%f,%f,%f" // gyro
  ",%f,%f" // alt, vvel
  ",%f,%f,%f" // orientation
  ",%d,%d,%d,%d,%d,%d,CD \n", // uptime, errorflagmp, errorflagnav, dataage, selfuptimem,state
  float(data.r.accel.x)/100,float(data.r.accel.y)/100,float(data.r.accel.z)/100,
  float(data.r.gyro.x)/100,float(data.r.gyro.y)/100,float(data.r.gyro.z)/100,
  float(data.r.altitude)/10,float(data.r.verticalvel)/100,
  float(data.r.orientationeuler.x)/100,float(data.r.orientationeuler.y)/100,float(data.r.orientationeuler.z)/100,
  data.r.uptime,data.r.errorflagmp,data.r.errorflagnav,millis()-packettime,millis(),data.r.state);

}


void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  Serial.begin(115200);
  while (!Serial) delay(100);
  Serial.println("\n\n init");
  Serial1.begin(9600);
  while (!ebyte.init())
  {
    Serial.println("radio init fail");
  }
  ebyte.setAddress(0xffff,true);
  ebyte.setPower(Power_21,true);
  ebyte.setChannel(68,true);
  ebyte.setNetID(0,true);
  ebyte.setBaud(UDR_9600,true);
  ebyte.setSubPacketSize(SPS_64,true);
  ebyte.setAirDataRate(ADR_0300,true);
  ebyte.setEncryptionKey(0,true);
  ebyte.setLBT(true,true);
  ebyte.printBoardParameters();
  ebyte.setRadioMode(MODE_PROGRAM);
  ebytesimple.printreg(0x00,0x08);
  ebyte.setRadioMode(MODE_NORMAL);


  Serial1.begin(9600);
}

void loop() {
  if (millis()-ledtime > 1000)
  {
    ledtime = millis();
    digitalWrite(LED_BUILTIN,ledstate);
    ledstate = !ledstate;
  }
  
  if (Serial.available())
  {
    int buf = Serial.read();
    Serial.printf("echo %d %c\n",buf);
    if (buf == '_')
    {
      uint8_t sendbuf = Serial.read();
      Serial1.write('t');
      Serial.printf("sending %d\n",sendbuf);
      
    }
    else{
      switch (buf)
      {
      case 'o':
        Serial.print("send start");
        senddata = true;
        break;
      
      case '+':
        Serial.print("send off");
        senddata = false;
      break;
      
      default:
        break;
      }
    }
    

    
    
  }
  if (Serial1.available() > 0)
  {
    int len = 0;
    uint8_t databuf[50] = {};
    Serial.println("new message: ");
    currentpacket = recivepacket();
    
  }
  if (millis()-senddatatime > 100 && senddata)
  {
    senddatatoserial(currentpacket);
    senddatatime = millis();
  }
  

}
    // while (Serial1.available() > 0)
    // {
    //   uint8_t buf = Serial1.read();
    //   Serial.printf("0x%x, ",buf);
    //   printBin(buf);
    //   Serial.printf(", %d\n",buf);
    //   databuf[j] = buf;
    //   delay(10);
    //   len++;
    //   j++;
    // }
    // Serial.printf("len: %d\n",len);
    // if (databuf[0] == 0x12 && databuf[31] == 0x34)
    // {
    //   Serial.println("recived data packet");
    // }