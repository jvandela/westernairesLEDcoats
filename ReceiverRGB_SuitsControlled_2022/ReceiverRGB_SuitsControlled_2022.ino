/*
 * Receiver code for Westernaires suit arduino systems. 
 * This sketch listens for radio signals from a single central transmitter indicating the current desired state of the lights. 
 * There are several patterns that are deterined by the state of the switches on the box (in the final design)
 * Jao van de Lagemaat March 2020
 */
#define RF24_SPI_SPEED 400000

#include <SPI.h>
#include "RF24.h"
#include <FastLED.h>

/*variables and arrays needed for the communication and monitoring of communication*/
//#define RF24_SPI_SPEED 10000000
RF24 radio(7,8);

byte addresses[][6] = {"Node1","Node2"};
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long interval = 500;           // interval at which to blink (milliseconds)
unsigned long receivedMessages=0;
unsigned long startMillis;
//int count;

uint8_t message[31];


/*Variable that holds the coat's address*/
uint8_t role=1;

/*Definitions and structures for the LED array addressing*/
#define NUM_LEDS_LEGS 18
#define NUM_LEDS_HAT 13
#define NUM_LEDS_BODY 25 // Really 15 but the belt is longer and will be divided in two
#define MIDPOINT_BELT 15 // This makes the belt symmetric (hopefully)
#define NUM_LEDS_TOTAL NUM_LEDS_LEGS+NUM_LEDS_HAT+NUM_LEDS_BODY
#define DATA_PIN_LEGS 9
#define DATA_PIN_MID 5 // This is now the pin for the hat LED strip
#define DATA_PIN_BODY 6
#define STOP_PIN 3

#define START_LEDS_HAT 0
#define START_LEDS_BODY NUM_LEDS_HAT
#define START_LEDS_LEGS NUM_LEDS_BODY+NUM_LEDS_HAT

#define MAX_BRIGHTNESS 128
#define MAX_POWER 30000
CRGBArray<NUM_LEDS_TOTAL> leds;
//CRGB leds[NUM_LEDS];
uint8_t startcolor=0;
boolean timeout = false;
boolean coatOn = true;
int currentbrightness = MAX_BRIGHTNESS;

void setup() {
  int count=0;
  for (count=0;count<31;count++)
    {message[count]=0;}
  Serial.begin(115200);

  pinMode(DATA_PIN_LEGS, OUTPUT); // The output pin used to communicate with the LED string on the Legs
  pinMode(DATA_PIN_MID, OUTPUT); // The output pin used to communicate with the LED string on the Hat
  pinMode(DATA_PIN_BODY, OUTPUT); // The output pin used to communicate with the LED string on the Body

  /*Figure out the coat's address */
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);

  /*This is the input used to see whether the coat should turn off*/
  pinMode(STOP_PIN, INPUT_PULLUP);
 
  role = !digitalRead(A0)+(!digitalRead(A1)*2)+(!digitalRead(A2)*4)
  +(!digitalRead(A3)*8)+(!digitalRead(A4)*16);
  //uint8_t role2=(!digitalRead(A6)*64)+(!digitalRead(A7)*128);

  if (role>20) role=0;

  Serial.print(F("This is coat number: "));
  Serial.println(role);
  
  /*Start the radio at 250 kbps*/
  radio.begin();


  radio.flush_rx();
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MAX); // Use max power on the radio
  radio.setDataRate(RF24_250KBPS); // Lowest bitrate enables largest range
  radio.enableDynamicAck();  // This allows multicast writes that don't expect ACKs to be sent back - increases range and speeds up


  radio.openWritingPipe(addresses[0]); 
  radio.openReadingPipe(1,addresses[1]);
//  radio.openReadingPipe(2,addresses[0]);
  radio.startListening();   // First, start listening.
   Serial.print("Radio receiving at channel: ");
  Serial.println(radio.getChannel());
 
  startMillis=millis();
  Serial.print(F("Initialized the radio at ms: "));
  Serial.println(startMillis);
  
  /*Set up the LED light string and blink the number of times of the coat's address*/
  FastLED.addLeds<WS2811, DATA_PIN_BODY, BRG>(leds, START_LEDS_BODY, NUM_LEDS_BODY);
  FastLED.addLeds<WS2811, DATA_PIN_MID, BRG>(leds, START_LEDS_HAT, NUM_LEDS_HAT);
  FastLED.addLeds<WS2811, DATA_PIN_LEGS, BRG>(leds, START_LEDS_LEGS, NUM_LEDS_LEGS);
  //FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  
  FastLED.setBrightness(MAX_BRIGHTNESS);FastLED.setMaxPowerInMilliWatts(MAX_POWER);
  for (int i=0;i<role;i++) {
    leds.fill_solid(CRGB::Red);FastLED.show();FastLED.delay(50);
    leds.fill_solid(CRGB::Black);FastLED.show();FastLED.delay(50);}
 
}


void loop() {
    
    radio.flush_tx();                               //Probably not necessary but should flush the radio's buffer             

    unsigned long started_waiting_at = millis();    // Set up a timeout period, get the current milliseconds
    
    uint8_t red, green, blue;
 
     if (!digitalRead(STOP_PIN))
      {
          FastLED.setBrightness(0);
          coatOn=false;
      }
      else 
      {
         FastLED.setBrightness(currentbrightness);
          coatOn=true;
         
      }

    while ( ! radio.available() ){                  // While nothing is received
      if (timeout==true) {  // Only when nothing was received last time
        //chasefour(CRGB::Red, CRGB::White, CRGB::Blue, CRGB::Black);
          //chasethree(CRGB::White, CRGB::Red, CRGB::Blue);
          ceylonthree(CRGB::White, CRGB::Red, CRGB::Blue); //default is hat:white, middle:red, bottom:blue
          //fill_grad();

         if (!digitalRead(STOP_PIN))
          {
              FastLED.setBrightness(0);
              coatOn=false;
          }
          else 
          {
             FastLED.setBrightness(currentbrightness);
              coatOn=true;
             
          }


          
          FastLED.show();
      }
       if (millis() - started_waiting_at > 5000 ){   // If waited longer than 5s, indicate timeout and exit while loop
          timeout = true;
          break;
      } 
 
    }
    if ( timeout){                                        
        if (receivedMessages==0)
        {
          Serial.println(F("Failed, response timed out."));
          Serial.print(F("Coats: "));
          Serial.println(coatOn);
          Serial.print(F("Input STOP_PIN: "));
          Serial.println(digitalRead(STOP_PIN));
          currentbrightness = MAX_BRIGHTNESS;
          FastLED.setBrightness(currentbrightness);
          //chasethree(CRGB::White, CRGB::Red, CRGB::Blue);
          //fill_grad();
          //FastLED.show();}
    }}
    
    if (radio.available()){
     timeout=false;
     radio.read(&message,31);
     if(message[0]==(role>9)){
         receivedMessages++;
         
        int startnumber=0;
        if(role>9){startnumber=1+(role-10)*3;}
        else{startnumber=1+role*3;}
        
        //Serial.println(startnumber);
        red=message[startnumber];
        green=message[startnumber+1];
        blue=message[startnumber+2];
        
        fill_solid(leds,NUM_LEDS_TOTAL,CRGB(red,green,blue));
        FastLED.show();
        
        /*if (receivedMessages>500){
          unsigned long currMillis=millis();
          long rate=1000*receivedMessages/(currMillis-startMillis);
          Serial.print("message rate = ");Serial.print(rate);Serial.println(" messages per second");
          //Serial.print("last message start = ");Serial.println(message[0]);
          receivedMessages=0;startMillis=currMillis;
        }*/
     }
    
    if (message[0]==2){
      receivedMessages++;
      CRGB CRGB1=CRGB( message[1],message[2],message[3]);
      CRGB CRGB2=CRGB( message[4],message[5],message[6]);
      CRGB CRGB3=CRGB( message[7],message[8],message[9]);
      CRGB CRGB4=CRGB( message[10],message[11],message[12]);
      startcolor=message[13];
      chasefour(CRGB1, CRGB2, CRGB3, CRGB4);
    }

     if(message[0]==3){
        receivedMessages++;

      uint8_t hue1,sat1,bright1,hue2,sat2,bright2;
      hue1=message[1];sat1=message[2];bright1=message[3];
      hue2=message[4];sat2=message[5];bright2=message[6];
      //fill_grad2(CHSV(hue1,sat1,bright1),CHSV(hue2,sat2,bright2));
        if (hue1 < hue2) {
          leds.fill_gradient(CHSV(hue1,sat1,bright1), CHSV(hue2,sat2,bright2), FORWARD_HUES);    // If we don't have this, the colour fill will flip around. 
        } else {
          leds.fill_gradient(CHSV(hue1,sat1,bright1), CHSV(hue2,sat2,bright2),BACKWARD_HUES);
        }
     }
    if (message[0]==4){
      receivedMessages++;
      if(digitalRead(STOP_PIN))
      {
        currentbrightness=message[1];
        FastLED.setBrightness(message[1]);
        coatOn=true;
      }
      else
      {
        FastLED.setBrightness(0);
        coatOn=false;
      }
    }
    if (message[0]==5){
      receivedMessages++;
      CRGB CRGB1=CRGB( message[1],message[2],message[3]);
      CRGB CRGB2=CRGB( message[4],message[5],message[6]);
      CRGB CRGB3=CRGB( message[7],message[8],message[9]);

      //fill_solid(leds,NUM_LEDS_TOTAL,CRGB(red,green,blue));
      fill_solid( &(leds[START_LEDS_HAT]), NUM_LEDS_HAT, CRGB1 );
      fill_solid( &(leds[START_LEDS_BODY]), NUM_LEDS_BODY, CRGB2 );
      fill_solid( &(leds[START_LEDS_LEGS]), NUM_LEDS_LEGS, CRGB3 );
    }
   if (message[0]==6){  // VU meters
      receivedMessages++;
      CRGB CRGB1=CRGB( message[1],message[2],message[3]);
      CRGB CRGB2=CRGB( message[4],message[5],message[6]);
      CRGB CRGB3=CRGB( message[7],message[8],message[9]);
      int intensity=message[10];
      int numtop=map(intensity,0,255,0, NUM_LEDS_HAT);
      int nummid=map(intensity,0,255,0, NUM_LEDS_BODY);
      int numbottom=map(intensity,0,255,0, NUM_LEDS_LEGS);

      leds.fadeToBlackBy( 20 );
      //fill_solid(leds,NUM_LEDS_TOTAL,CRGB::Black);
      fill_solid( &(leds[START_LEDS_HAT]), numtop, CRGB1 );
      fill_solid( &(leds[START_LEDS_BODY]), nummid, CRGB2 );
      fill_solid( &(leds[START_LEDS_LEGS]), numbottom, CRGB3 );
    }

    if (message[0]==7){
      receivedMessages++;
      CRGB CRGB1=CRGB( message[1],message[2],message[3]);
      CRGB CRGB2=CRGB( message[4],message[5],message[6]);
      CRGB CRGB3=CRGB( message[7],message[8],message[9]);
      byte pos1=message[10];
      byte pos2=message[11];
      byte pos3=message[12];
      ceylonthreesync(CRGB1, CRGB2, CRGB3, pos1, pos2, pos3);
    }

   if (message[0]==8){
      receivedMessages++;
      CRGB CRGB1=CRGB( message[1],message[2],message[3]);
      CRGB CRGB2=CRGB( message[4],message[5],message[6]);
      CRGB CRGB3=CRGB( message[7],message[8],message[9]);
      CRGB CRGB4=CRGB( message[10],message[11],message[12]);
      candycane(CRGB1, CRGB2, CRGB3, CRGB4);
    }

     /*Serial.print(hue1);Serial.print(" ");
     Serial.print(sat1);Serial.print(" ");
      Serial.print(bright1);Serial.print(" ");
    Serial.print(hue2);Serial.print(" ");
     Serial.print(sat2);Serial.print(" ");
      Serial.println(bright2);Serial.print(" ");*/
       /*if (message[30]==1) {
        Serial.print("Message from: ");Serial.println(message[30]);
        }*/
       
       if (millis()-startMillis>1000){
        unsigned long currMillis=millis();
        long rate=1000*receivedMessages/(currMillis-startMillis);
        Serial.print("message rate = ");Serial.print(rate);Serial.println(" messages per second");
         //Serial.print("last message start = ");Serial.println(message[0]);
         if (rate!=0) repeatMessage();
         receivedMessages=0;startMillis=currMillis;
         
      }
      //FastLED.delay(10);
      FastLED.show();

     }
    
}

void candycane(CRGB color1, CRGB color2, CRGB color3, CRGB color4) {
  for (int i=0;i<NUM_LEDS_TOTAL;i++) {
    int curpos=(i) % 4;
    switch (curpos){
      case 0 : {leds[i]=color1; break;}
      case 1 : {leds[i]=color2; break;}
      case 2 : {leds[i]=color3; break;}
      default : {leds[i]=color4;}
    }
  }
  
  //FastLED.delay(100);
}

void chasefour(CRGB color1, CRGB color2, CRGB color3, CRGB color4) {
  for (int i=0;i<NUM_LEDS_TOTAL;i++) {
    int curpos=(i+startcolor) % 7;
    switch (curpos){
      case 0 : {leds[i]=color1; break;}
      case 1 : {leds[i]=color2; break;}
      case 2 : {leds[i]=color3; break;}
      default : {leds[i]=color4;}
    }
  }
  
  FastLED.delay(100);
}

void chasethree(CRGB color1, CRGB color2, CRGB color3) { // Hat, Body, Legs
  fill_solid(leds,NUM_LEDS_TOTAL,CRGB::Black);
  for (int i=0;i<NUM_LEDS_HAT;i++) {
    int curpos=(i+startcolor) % 5;
    if (curpos==0){
      leds[i]=color1; 
    }
  }
  for (int i=START_LEDS_BODY;i<START_LEDS_BODY+NUM_LEDS_BODY;i++) {
    int curpos=(i+startcolor) % 5;
    if (curpos==0){
      leds[i]=color2; 
    }
  }
  for (int i=START_LEDS_LEGS;i<NUM_LEDS_TOTAL;i++) {
    int curpos=(i+startcolor) % 5;
    if (curpos==0){
      leds[i]=color3; 
    }
  }
   
  startcolor+=1;
  if (startcolor>4) startcolor=0;
  FastLED.delay(100);
}


void ceylonthree(CRGB color1, CRGB color2, CRGB color3) { // hat, body, legs
  //fill_solid(leds,NUM_LEDS_TOTAL,CRGB::Black);
  fadeToBlackBy(leds,NUM_LEDS_TOTAL,5);
  int pos = beatsin16(40,0,NUM_LEDS_HAT-1);
  leds[pos] += color1;
  pos=START_LEDS_BODY+beatsin16(50,0,MIDPOINT_BELT-1);
  leds[pos] += color2;
  pos=START_LEDS_BODY+NUM_LEDS_BODY-1-beatsin16(50,0,MIDPOINT_BELT-1);
  leds[pos] += color2;
  pos=START_LEDS_LEGS+beatsin16(45,0,NUM_LEDS_LEGS-1);
  leds[pos] += color3;
  
    
 }

void ceylonthreesync(CRGB color1, CRGB color2, CRGB color3, byte pos1, byte pos2, byte pos3) {
  fadeToBlackBy(leds,NUM_LEDS_TOTAL,5);
  leds[pos1] += color1;
  leds[pos2] += color2;
  leds[pos3] += color3;    
 }


void fill_grad() {
  
  uint8_t starthue = beatsin8(10, 0, 255);
  uint8_t endhue = beatsin8(12, 0, 255);
  
  if (starthue < endhue) {
    leds.fill_gradient(CHSV(starthue,255,255),CHSV(endhue,255,255), FORWARD_HUES);    // If we don't have this, the colour fill will flip around. 
  } else {
    leds.fill_gradient(CHSV(starthue,255,255), CHSV(endhue,255,255), BACKWARD_HUES);
  }
  
} // fill_grad()

void fill_grad2(CHSV starthue, CHSV endhue) {
  //leds=CRGB::Black ; 
  if (starthue < endhue) {
    leds.fill_gradient(starthue, endhue, FORWARD_HUES);    // If we don't have this, the colour fill will flip around. 
  } else {
    leds.fill_gradient(starthue, endhue, BACKWARD_HUES);
  }
  
} // fill_grad()

void repeatMessage()
{
    //message[30]=1;
    radio.stopListening();
    radio.openWritingPipe(addresses[1]); 

    radio.write(&message,sizeof(message),0);
    //Serial.println("repeated a message");
    //message[30]=0;

    radio.openReadingPipe(1,addresses[1]);
    radio.startListening();
}
