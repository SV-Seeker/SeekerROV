import hypermedia.net.*;                    //UDP library
import controlP5.*;                         //Library for creating the GUI
ControlP5 cp5;                              //GUI class; make an instance
UDP udp;

//Instances of buttons and widgets from ControlP5
Textarea console;
Textfield ipAddress, portNumber;
Button gogo;
Knob HDG, PCH;
PImage subb;
float realheading = 0;
int headingcmd = 0;
int pitchcmd = 0;

long PktIndex = 0;

void setup() {
  size(550,500);     //size of window
  udp = new UDP(this, 6000);
  udp.listen(true);
  cp5 = new ControlP5(this); 

  //Buttons and widgets; created by the ControlP5 library
  console = cp5.addTextarea("txt")
              .setPosition(10,125)
              .setSize(150, 255)
              .setFont(createFont("proggy",9))
              .setColor(color(255,128,0))
              .setColorBackground(color(0)); 
  ipAddress = cp5.addTextfield("ipAddress")
              .setPosition(25,25).setSize(110, 18)
              .setFont(createFont("proggy",13))
              .setColor(color(255,128,0))
              .setColorBackground(color(0))
              .setValue("192.168.2.109");
  portNumber = cp5.addTextfield("portNumber")
              .setPosition(25,75).setSize(110, 18)
              .setFont(createFont("proggy",13))
              .setColor(color(255,128,0))
              .setColorBackground(color(0))
              .setValue("8888"); 
  HDG = cp5.addKnob("HEADING")
              .setPosition(180,25)
              .setRadius(80)
              .setViewStyle(Knob.LINE)
              .setDragDirection(Knob.HORIZONTAL)
              .setStartAngle(PI/2)
              .setAngleRange(2*PI)
              .setNumberOfTickMarks(24)
              .setTickMarkWeight(3)
              .setTickMarkLength(6)
              .showTickMarks()
              .setRange(-180,180);
  PCH = cp5.addKnob("PITCH")
              .setPosition(180,250)
              .setRadius(80)
              .setViewStyle(Knob.LINE)
              .setDragDirection(Knob.VERTICAL)
              .setStartAngle(-PI/6)
              .setAngleRange(PI/3)
              .setNumberOfTickMarks(6)
              .setTickMarkWeight(3)
              .setTickMarkLength(6)
              .showTickMarks()
              .setRange(30,-30);
  gogo = cp5.addButton("GOGO").setPosition(10, 380).setSize(50, 25);            
  subb = loadImage("sub1.png");
}

void draw() {
  background(color(9, 10, 30));
  translate(450, 220);
  rotate(realheading);
  imageMode(CENTER);
  image(subb, 0, 0);
  rotate(-realheading);
  translate(-450, -220);
}

void GOGO() {
  packageAndSend('h','p', headingcmd, pitchcmd);
}

void HEADING() {
  headingcmd = round(HDG.getValue()*100);
  //packageAndSend('h', round(HDG.getValue()*100));
}

void PITCH() {
  pitchcmd = round(PCH.getValue()*100);
  //packageAndSend('p', round(PCH.getValue()*100));
}

void packageAndSend(char cmd1, char cmd2, int prm1, int prm2) {
  byte trail[] = {'#', ' ', byte(PktIndex>>16), byte(PktIndex>>8), byte(PktIndex), byte(cmd1), byte(prm1>>8), byte(prm1), byte(cmd2), byte(prm2>>8), byte(prm2)};
  String cluster = new String(trail);
  udp.send(cluster, ipAddress.getText(), int(portNumber.getText()));
  PktIndex++;
}

void receive( byte[] data){
  String myString = "";
  for(int i=0; i<data.length; i++){
    myString = new String(data);                  //convert the data to string
  }
  char startingChar = myString.charAt(0);
  if (startingChar!='@') addToConsole(myString);
  //addToConsole(myString);
  packageAndSend('h','p', headingcmd, pitchcmd);  
}

void addToConsole(String stringToAdd){
  String textInBox = console.getText();
  console.setText(textInBox + "\n " + stringToAdd);
  console.scroll(1);
  int first, last;
  if ((stringToAdd.indexOf("Y"))>=0) {
    first = stringToAdd.indexOf("Y");
    if ((stringToAdd.indexOf("P"))>0) {
      last = stringToAdd.indexOf("P");
      realheading = float(stringToAdd.substring(first+1, last))/100/180*PI;
    }
  }
  
}
