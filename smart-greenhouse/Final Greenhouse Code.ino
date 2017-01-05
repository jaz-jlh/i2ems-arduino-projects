
//Include Libraries
#include <LiquidCrystal.h>
#include <Servo.h>
#include <SPI.h>
#include <Ethernet.h>

Servo servo;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(8, 7, 6, 5, 2, 9);

//DATABASE STUFF
EthernetClient client;
byte mac[] = {0x90, 0xA2, 0xDA, 0x00, 0xD3, 0x27 };
IPAddress myIP; //my IP address is assigned using DHCP
IPAddress serverIP(128,143,6,188); //server's IP address

boolean lastConnected = false;
int setpoint = 23;

long duration = 30000;
long endTime = 0;
boolean heaterOn = false;
String currentLine = "";
int heaterValue = 0;
boolean lidOpen = false;

void setup() {

  pinMode(1, OUTPUT); //LED
  pinMode(A1, INPUT); //Internal temp sensor
  pinMode(A2, INPUT); //External temp sensor
  pinMode(A0, INPUT); //Pot
  pinMode(3, OUTPUT); //Relay
  servo.attach(19); //Servo (greenhouse lid)
  servo.write(180);
  Serial.begin(9600);
   
  lcd.begin(16,2);

  //start the ethernet connection:
  Serial.println("Attempting to connect to the interwebs.");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    //on failure, no point in carrying on, so do nothing forevermore:
    for(;;) {}
   }
  
  //On success, get our ip address...
  myIP=Ethernet.localIP();

  //...and print it
  Serial.print("My IP address: ");
  for (byte thisByte = 0; thisByte < 4; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(myIP[thisByte], DEC);
    Serial.print("."); 
  }
  Serial.println();
}




void loop() {
  if(millis() > endTime)
  {
    service();
    endTime = millis() + duration;
  }
  
  while(client.available()) {
      char c = client.read();
      //Serial.print(c);
      currentLine += c;
      if (c=='\n') {
        currentLine="";
      }
      
      if(currentLine.endsWith("</setpoint>")) {
        //Serial.print(currentLine);
        setpoint = (currentLine.substring(10).toInt());
        //Serial.print("Found the setpoint amongst the HTML, setpoint is:");
        //Serial.println(setpoint);
        currentLine="";
        //Serial.print(currentLine);
      }
    
    boolean currClient = client.connected();
    if (!currClient && lastConnected) {
     Serial.println();
     Serial.println("GetSetPoint loop disconnecting.");
     client.stop();
  }
  lastConnected = currClient;
}
}

void service() {
  
  float tempVoltage = 5./1024. * analogRead(A1);
  //Serial.println(tempVoltage);
 
  float tempVoltageExt = 5./1024. * analogRead(A2);  //external
  float tempCelsiusExt = (tempVoltageExt - 1.375)/0.0225; //external
  //Serial.println(tempVoltageExt);
  float tempCelsius = (tempVoltage - 1.375)/0.0225;
  
  // Heater Logic
  if (setpoint - 1 > tempCelsius && heaterOn == false) {
     digitalWrite(1, HIGH);  // turn the LED on (HIGH is the voltage level)
     digitalWrite(3, HIGH);  // activate relay
     heaterOn = true;
  }
  if (setpoint  < tempCelsius && heaterOn) {
    digitalWrite(1, LOW); // turn LED off
    digitalWrite(3, LOW); // deactivate relay
    heaterOn = false;
  }
  
  // Servo Logic
  if ((setpoint + 2) - 1 > tempCelsius && lidOpen) {
     servo.write(170); //close lid
     lidOpen = false;
  }
  if ((setpoint + 2) < tempCelsius && lidOpen == false) {
    servo.write(10);  // open lid
    lidOpen = true;
  }
  
  lcd.clear();
  lcd.print("Htr:");
  if(heaterOn == true) {
    lcd.print("On ");
  } else {
    lcd.print("Off");
  }
  lcd.print(" SPt:");
  lcd.print((int)setpoint);
  lcd.print("C");
  
  lcd.setCursor(0,1);
  lcd.print("Int:");
  lcd.print((int)tempCelsius);
  lcd.print("C");
  lcd.print(" Ext:");
  lcd.print((int)tempCelsiusExt);
  lcd.print("C");
  
  Serial.print("Set-point: ");
  Serial.print(setpoint);
  Serial.print(" Temperature Inside: ");
  Serial.print(tempCelsius);
  Serial.print(" C");
  Serial.print(" Temperature Outside: ");
  Serial.print(tempCelsiusExt);
  Serial.print(" C  ");
  Serial.println(lidOpen);
  //String dataString  = String(setpoint) + "," + String(tempCelsius) + "," + String(tempCelsiusExt) + String(heaterOn); //added in external temp
 
 if(heaterOn) {
   heaterValue = 35;
 } else {
   heaterValue = 0;
 }
 
  sendToDatabase(1, tempCelsius);
  sendToDatabase(2, tempCelsiusExt);
  sendToDatabase(3, setpoint);
  sendToDatabase(4, heaterValue); 
  sendToDatabase(5, lidOpen);
  
  //Get setpoint from databa
  getSetpointFromServer();
  
}

void sendToDatabase(int id, float value){
  EthernetClient client;
  if( client.connect(serverIP, 80) )
  {
     //upon success, create the sql statement:
     //first, print the name of the script
     client.print("GET /tlp2017/ldw8fb/Greenhouse.php?");
   
    //and then add some data
    client.print("id=");
    client.print(id);
    client.print("&value=");
    client.print(value); //inside  

    //finish up with some bookkeeping
    client.println(" HTTP/1.1");
    client.println("Host: 128.143.6.188");//unneeded????
    client.println("Connection: close");
    client.println(); //always end with a blank line -- this tells the web server that we're done
    client.stop(); //send a stop signal when we're finished
  }
  
  else{
    Serial.println("Send Connection failed!\n");
  }
  
  Serial.println("Send Done!");
}

/////////////////////////////////
void getSetpointFromServer() {
  // if there's a successful connection:
  if (client.connect(serverIP, 80)) {
    Serial.println("GetSetPoint connecting...");
    // send the HTTP PUT request:
    client.print("GET /tlp2017/ldw8fb/GetSetpoint.php"); //
//   client.println("Host: 128.143.6.188");
//    client.println("User-Agent: arduino-ethernet");
//    client.println("Connection: close");
    client.println();

    // note the time that the connection was made:
  //  lastConnectionTime = millis();
    
  //  Serial.println("CLIENT STUFF");
        
  } 
  else {
    // if you couldn't make a connection:
    Serial.println("GetSetPoint connection failed");
    Serial.println("GetSetPoint disconnecting.");
    client.stop();
  }
}

