/*
  WiFi Web Server LED Blink

  A simple web server that lets you blink an LED via the web.
  This sketch will create a new access point (with no password).
  It will then launch a new server and print out the IP address
  to the Serial Monitor. From there, you can open that address in a web browser
  to turn on and off the LED on pin 13.

  If the IP address of your board is yourAddress:
    http://yourAddress/H turns the LED on
    http://yourAddress/L turns it off

  created 25 Nov 2012
  by Tom Igoe
  adapted to WiFi AP by Adafruit
  
  modified 22 March 2023
  by Karl Söderby
 */

#include <SPI.h>
#include <WiFi.h>
#include "arduino_secrets.h"
///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int keyIndex = 0;           // your network key index number (needed only for WEP)

int status = WL_IDLE_STATUS;
WiFiServer server(80);

//Variable Declarations
int pump = 2;
int solenoid = 3;

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600);
    /*while (!Serial) {
      ;  // wait for serial port to connect. Needed for native USB port only
    }*/

  Serial.println("Access Point Web Server");

	// Set pump as an output pin
	pinMode(pump, OUTPUT);

  // Set solenoid as an output pin
	pinMode(solenoid, OUTPUT);

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  // by default the local IP address will be 192.168.4.1
  // you can override it with the following:
  // WiFi.config(IPAddress(10, 0, 0, 1));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true)
      ;
  }

  // wait 10 seconds for connection:
  delay(10000);

  // start the web server on port 80
  server.begin();

  // you're connected now, so print out the status
  printWiFiStatus();
}


void loop() {
  // compare the previous status to the current status
  if (status != WiFi.status()) {
    // it has changed update the variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
    }
  }

  WiFiClient client = server.available();  // listen for incoming clients

  if (client) {                    // if you get a client,
    Serial.println("new client");  // print a message out the serial port
    String currentLine = "";       // make a String to hold incoming data from the client
    while (client.connected()) {   // loop while the client's connected
      delayMicroseconds(10);       // This is required for the Arduino Nano RP2040 Connect - otherwise it will loop so fast that SPI will never be served.
      if (client.available()) {    // if there's bytes to read from the client,
        char c = client.read();    // read a byte, then
        Serial.write(c);           // print it out the serial monitor
        if (c == '\n') {           // if the byte is a newline character

          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();

            // the content of the HTTP response follows the header:
            client.print("<style>");
            client.print(".container {margin: 0 auto; text-align: center; margin-top: 100px;}");
            client.print("button {color: white; width: 100px; height: 100px;");
            client.print("border-radius: 50%; margin: 20px; border: none; font-size: 20px; outline: none; transition: all 0.2s;}");
            client.print(".red{background-color: rgb(196, 39, 39);}");
            client.print(".green{background-color: rgb(39, 121, 39);}");
            client.print(".blue {background-color: rgb(5, 87, 180);}");
            client.print(".off{background-color: grey;}");
            client.print("button:hover{cursor: pointer; opacity: 0.7;}");
            client.print("</style>");
            client.print("<div class='container'>");
            client.print("<button class='blue' type='submit' onmousedown='location.href=\"/BH\"'>ON</button>");
            client.print("<button class='off' type='submit' onmousedown='location.href=\"/BL\"'>OFF</button><br>");
            client.print("<button class='green' type='submit' onmousedown='location.href=\"/RH\"'>ON</button>");
            client.print("<button class='off' type='submit' onmousedown='location.href=\"/RL\"'>OFF</button><br><br><br>");

            client.print("<style>");
            client.print("button1 {color: #ffffff; background-color: #2d63c8; font-size: 20px; border: 0px solid #2d63c8; padding: 15px 50px; cursor: pointer}");  
            client.print("button1:hover {color: #2d63c8; background-color: #ffffff;}");    
            client.print("</style>");  
            client.print("</head>");
            client.print("<body>");
            client.print("<button1 class='blue' type='submit' onmousedown='location.href=\"/BH\"'>HOSE ON</button1>");
            client.print("<button1 class='off' type='submit' onmousedown='location.href=\"/BL\"'>HOSE OFF</button1><br><br><br>");
            client.print("<button1 class='green' type='submit' onmousedown='location.href=\"/RH\"'>PUMP ON</button1>");
            client.print("<button1 class='off' type='submit' onmousedown='location.href=\"/RL\"'>PUMP OFF</button1><br><br><br>");
            client.print("</body>");
            client.print("</html>");

            //client.print("<button class='green' type='submit' onmousedown='location.href=\"/GH\"'>ON</button>");
            //client.print("<button class='off' type='submit' onmousedown='location.href=\"/GL\"'>OFF</button><br>");
            client.print("</div>");


            // The HTTP response ends with another blank line:
            client.println();
            // break out of the while loop:
            break;
          } else {  // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request (turns ON/OFF the pump/solenoid)
        if (currentLine.endsWith("GET /RH")) {
          digitalWrite(pump, HIGH);  
        }
        if (currentLine.endsWith("GET /RL")) {
          digitalWrite(pump, LOW); 
        }
        if (currentLine.endsWith("GET /BH")) {
          digitalWrite(solenoid, HIGH);  
        }
        if (currentLine.endsWith("GET /BL")) {
          digitalWrite(solenoid, LOW);  
        }
      }
    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }
}

void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print where to go in a browser:
  Serial.print("To see this page in action, open a browser to http://");
  Serial.println(ip);
}