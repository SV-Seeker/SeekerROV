#define MAX_PACKET_COMMANDS 6         //max number of different commands to be packed into a udp packet
#define ROV_COMMAND_SIZE 3            //size of a single command
#define TIMESTAMP_SIZE 2              //size of the UDP timestamp
#define MAX_ROV_PACKET TIMESTAMP_SIZE+ROV_COMMAND_SIZE*MAX_PACKET_COMMANDS

char IncomingCommandType;  //raw command characters from remote controller
int IncomingCommandValue;  //raw command values from remote controller
int curPacketSize = TIMESTAMP_SIZE;         //number of bytes in incoming packet, minimum of just the timestamp
unsigned long lastTimestamp = 0;            //an ordering number to prevent commands being processed out-of-order

float HdgCommand = 0.0;       //variables for the current processed commands from the pilot
float PchCommand = 0.0;       //     /
float SpdCommand = 0.0;       //   /
float AltCommand = 100.0;     // /

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(192, 168, 2, 109);

unsigned int localPort = 8888;      // local port to listen on

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;




char UpdateCommand(){
  char goodcommands = '0';
  char retstring[16];
  if(Udp.read()!='#') return('0');
  if(Udp.read()!=' ') return('0');
  unsigned int newTimestamp = ((((unsigned word)Udp.read())<<16) | (((unsigned int)Udp.read())<<8) | ((unsigned int)Udp.read()));
  if (newTimestamp > lastTimestamp) {
    lastTimestamp = newTimestamp;      //ignore any future packets that were sent before this one
    if (curPacketSize<=MAX_ROV_PACKET) {
      for (int i=0; i<((curPacketSize-TIMESTAMP_SIZE)/ROV_COMMAND_SIZE); i++) {
        IncomingCommandType = Udp.read();
        IncomingCommandValue = (int)((((unsigned word)Udp.read())<<8) | ((unsigned word)Udp.read()));
        Serial.print(IncomingCommandType);
        Serial.print(IncomingCommandValue);
        Serial.print(" ");
        switch (IncomingCommandType) {
          case 'A':  //altitude speed
            goodcommands++;
            break;
          case 'a':  //altitude position
            goodcommands++;
            break;
          case 'E':  //east-west speed
            goodcommands++;
            break;
          case 'F':  //level forward speed
            goodcommands++;
            break;
          case 'H':  //heading change rate 
            goodcommands++;   
            break;
          case 'h':  //heading position
            if ((IncomingCommandValue > 18000) && (IncomingCommandValue <= 36000)) {
              IncomingCommandValue -= 36000;    //fold 180:360 into negative 0:180 
            }
            if ((IncomingCommandValue >= -18000) && (IncomingCommandValue <= 18000)){
              goodcommands++;
              HdgCommand = ((float)(IncomingCommandValue))/100.0;
            }
            break;
          case 'L':  //lifter speed (body coordinates)
            goodcommands++;
            break;
          case 'M':  //mode of operation
            goodcommands++;
            break;
          case 'N':  //north-south speed
            goodcommands++;
            break;
          case 'P':  //pitch change rate
            goodcommands++;
            break;
          case 'p':  //pitch position
            if ((IncomingCommandValue >= -3000) && (IncomingCommandValue <= 3000)){
              goodcommands++;
              PchCommand = ((float)(IncomingCommandValue))/100.0;
            }
            break;
          case 'S':  //strafe speed (body coordinates)
            goodcommands++;
            break;
          case 'T':  //thruster speed (body coordinates)
            goodcommands++;
            break;
          case '#':  //relays
            goodcommands++;
            ESCRELAYPORT = (byte)IncomingCommandValue;
            break;
          default:
            break;
        }
      }
    }
  }
  Serial.println("");
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.write('@');
  Udp.write(goodcommands);
  Udp.endPacket();
  
  return(goodcommands);
}
