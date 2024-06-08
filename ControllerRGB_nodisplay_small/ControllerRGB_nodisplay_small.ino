/*
 * Transmitter code for Westernaires suit arduino systems. 
 * This sketch sends repetitive radio signals indicating the current desired state of the lights. 
 * There are several patterns that are deterined by the state of the switches on the box (in the final design)
 */
#include <Keypad.h>
#define RF24_SPI_SPEED 4000000
#define MESSAGE_WAIT 0
#define FADERATE 10

#include <SPI.h>
#include "RF24.h"

/*       
        ####UPLOAD TO MEGA####
        #     MOSI==>51      #
        #     MISO==>50      #
        #     SCK===>52      #
        # # # # # # # # # # ##
        # CE_pin 7&CSN_pin 8 #
        ######################
*/

/*        
        #UPLOAD TO UNO/NANO##
        #    MOSI==>11      #
        #    MISO==>12      #
        #    SCK===>13      #
        # # # # # # # # # # #
        #CE_pin 7&CSN_pin 8 #
        #####################
*/

//#include <ArduinoBlue.h>

/*FastLED setup*/

#include <FastLED.h>
#define NUM_LEDS 20
/*
#define NUM_LEDS_LEGS 18
#define NUM_LEDS_MID 25
#define NUM_LEDS_BODY 15
#define NUM_LEDS_TOTAL NUM_LEDS_LEGS+NUM_LEDS_MID+NUM_LEDS_BODY
*/
#define NUM_LEDS_LEGS 18
#define NUM_LEDS_HAT 13
#define NUM_LEDS_BODY 25 // Really 15 but the belt is longer and will be divided in two
#define MIDPOINT_BELT 15 // This makes the belt symmetric (hopefully)
#define NUM_LEDS_TOTAL NUM_LEDS_LEGS+NUM_LEDS_HAT+NUM_LEDS_BODY

#define START_LEDS_BODY 0
#define START_LEDS_BODY NUM_LEDS_HAT
#define START_LEDS_LEGS NUM_LEDS_BODY+NUM_LEDS_HAT

#define DATA_PIN 9
#define BRIGHT_PIN A3
#define SENS_PIN A2
#define HUE_PIN A1
#define RATE_PIN A0
#define SOUND_PIN A9
#define SOUND_ON_PIN 2

CRGBArray<NUM_LEDS> leds;

int brightness=0;
int huepot=0;
int satpot=0;
int rate=0;
int Go=1;
uint8_t startcolor=0;

const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

//byte rowPins[ROWS] = {36, 34, 32, 30};
//byte colPins[COLS] = {28, 26, 24, 22};
byte rowPins[ROWS] = {16, 18, 20, 22};
byte colPins[COLS] = {24, 26, 28, 30};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 

RF24 radio(7,8);

byte addresses[][6] = {"Node1","Node2"};
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long interval = 750;           // interval at which to blink (milliseconds)
unsigned long previousBright = 0;        // will store last time LED was updated
unsigned long sentMessages=0;           //variable to store how many messages have been sent and measure send rate
unsigned long startMillis;

int ledState=0;
//byte message[]={0,0,0};

byte message1[31];
byte message2[31];
byte message3[31];

//const int soundPIN =  2;
//const int sensorPIN = A0;

CRGB rgb=CRGB::Black;
CRGB rgb2=CRGB::Black;
CRGB rgb3=CRGB::Black;

uint8_t hue=0;

// Setting up the buetooth module
// Bluetooth TX -> Arduino D10
//const int BLUETOOTH_TX = 5;
// Bluetooth RX -> Arduino D9
//const int BLUETOOTH_RX = 4;
//SoftwareSerial bluetooth(BLUETOOTH_TX, BLUETOOTH_RX);
//ArduinoBlue phone(bluetooth); // pass reference of bluetooth object to ArduinoBlue constructor.

int programstate=1;
int sliderVal, button, sliderId, soundstate;
byte soundlevel;
byte brightnessstate=0;

void setup() {
 
  Serial.begin(115200);
  Serial.println("This is the main control unit");
     // Start bluetooth serial at 9600 bps.
//  bluetooth.begin(9600);

    // delay just in case bluetooth module needs time to "get ready".
//  delay(100);

//  Serial.println("Bluetooth setup complete");
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally

  radio.begin();
  radio.flush_rx();
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MAX); // Use max power on the radio
  radio.setDataRate(RF24_250KBPS); // Lowest bitrate enables largest range
  radio.enableDynamicAck();  // This allows multicast writes that don't expect ACKs to be sent back - increases range and speeds up

    radio.openWritingPipe(addresses[1]);
    radio.openReadingPipe(1,addresses[0]); // not really used on this radio
  radio.stopListening();
  radio.setChannel(76);
  radio.powerUp();

  Serial.print("This radio broadcasting at channel: ");
  Serial.println(radio.getChannel());
  startMillis=millis(); 
  Serial.print(F("Initialized the radio at ms: "));
  Serial.println(startMillis);
 
  ledState=0;
  pinMode(SOUND_ON_PIN, INPUT_PULLUP);
  Serial.print("sound value");
  Serial.println(analogRead(SOUND_PIN));
  
  //programstate=0;
  crossFade(CRGB::Black,CRGB::Red,100);
  crossFade(CRGB::Red,CRGB::Green,100);
  crossFade(CRGB::Green,CRGB::Blue,100);
  
  sendcolortoall(CRGB::White);
   sendcolortoall(rgb);
 
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(100);
  Serial.print("The size of a CRGB object is ");
  Serial.println(sizeof( CRGB));
}




void loop() {
          //radio.setChannel(76);                              // First, stop listening so we can talk.
    unsigned long start_time = micros();    
    unsigned long currentMillis = millis();

    //print the messaging rate
       if (millis()-startMillis>2000){
        unsigned long currMillis=millis();
        long rate=1000*sentMessages/(currMillis-startMillis);
        Serial.print("message rate = ");Serial.print(rate);Serial.println(" messages per second");
        sentMessages=0;startMillis=currMillis;
      }

 
  /*if(!digitalRead(SOUND_ON_PIN)) {soundstate=1;}
  else soundstate=0;
  
  int j, soundlevelint=0;
    for (j=0;j<10;j++){
      soundlevelint+=analogRead(SOUND_PIN);
    }
    int soundlevel2=14*soundlevel + map(soundlevelint*map(analogRead(SENS_PIN),0, 1023, 0, 60),0,6000,0,255);
    soundlevel=constrain(soundlevel2/15,0,255);
            //
    //Serial.print("soundlevel: ");
    //Serial.println(analogRead(SOUND_PIN));
    //Serial.println(analogRead(BRIGHT_PIN));  
   */
  if (currentMillis-previousBright>33)
  {
    if(Go==1) 
    {
      /*if (soundstate)
        {
          if (brightnessstate==0)
            brightness=soundlevel;
          else
            brightness=map(analogRead(BRIGHT_PIN),0, 1023, 0, 255);;
          //Serial.print("soundlevel: ");
          //Serial.println(soundlevel);
        }
      else brightness=map(analogRead(BRIGHT_PIN),0, 1023, 0, 255);*/
      brightness=map(analogRead(BRIGHT_PIN),0, 1023, 0, 255);
    }
    else brightness=0;
    huepot=map(analogRead(HUE_PIN), 0, 1023, 0, 255);
    satpot=255;
    rate=map(analogRead(RATE_PIN), 0, 1023, 0, 255);
    previousBright=currentMillis;
    interval=map(rate,0,255,1023,0);
/*    Serial.print("brightness :");
    Serial.println(brightness);
/*    Serial.print("Hue :");
    Serial.println(huepot);
     Serial.print("Saturation :");
    Serial.println(satpot);
     Serial.print("Rate :");
    Serial.println(rate);*/
     FastLED.setBrightness(map(brightness,0,255,0,64));
    updateBright();
  }
  
  char customKey = customKeypad.getKey();

  if (customKey) {
      Serial.print("pressed ");
      Serial.println(customKey);
      if (customKey == '1') {
        programstate=1;Serial.println("solid red/white/blue");
      }

      if (customKey == '2') {
        programstate=0;Serial.println("red/white/blue alternate accross coats");
      }
     if (customKey == '3') {
        programstate=2;Serial.println("pulsed red/white/blue solid on all coats");
       }
      if (customKey == '4') {
        programstate=3;Serial.println("Rainbow all");
       }
      if (customKey == '5') {
        programstate=4;Serial.println("Rainbow float");
       }
      if (customKey == '6') {
        programstate=5;Serial.println("Rainbow pulsed");
       }
      if (customKey == '7') {
        programstate=6;Serial.println("Ceylons!");
       }
     if (customKey == '8') {
        programstate=7;Serial.println("Alternate RWB pulsed");
       }
     if (customKey == '9') {
        programstate=8;Serial.println("Bouncy");
        }
     if (customKey == 'A') {
        programstate=9;Serial.println("Bouncy RWB");
        }
     if (customKey == 'B') {
        programstate=10;Serial.println("Bouncy on all");
       }
     if (customKey == 'C') {
        programstate=11;Serial.println("Solid Color Bounce");
       }
     if (customKey == 'D') {
        programstate=12;Serial.println("ChaseRWB");
       }
    if (customKey == '*') {
        Go=1;Serial.println("Start");
       }
   if (customKey == '#') {
        Go=0;Serial.println("Stop");
       }




  }
  //Serial.println("here now");
  
  switch (programstate) {
    case 0 : {
      brightnessstate=0;
      if (currentMillis - previousMillis >= interval) {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
      switch (ledState) {
           case 0: {suitcolortoall(CRGB::Red, CRGB::White, CRGB::Blue);break;}
            case 1: {suitcolortoall(CRGB::White, CRGB::Blue, CRGB::Red);break;}
            case 2: {suitcolortoall(CRGB::Blue, CRGB::Red, CRGB::White);break;}
            //case 3: {sendcolortoall(CRGB::Black);   rgb=CRGB::Black;break;}
        //case 3: {{rgb[0]=black[0];rgb[1]=black[1];rgb[2]=black[2];break;}}
        }
       ledState++;
       if (ledState>2) {ledState=0;}
       }
     repeatMessage();
    break;}
/*    case 1 : { // sound driven VU meters

          brightnessstate=1;
          suitVUmeters(CRGB::Red, CRGB::White, CRGB::Blue, soundlevel);
 
          break;

    }*/
    case 1 : { // solid red/white/blue
      suitcolortoall(CRGB::Red, CRGB::White, CRGB::Blue);
      repeatMessage();
      break;
    }
    case 2 : { 
        brightnessstate=0;
        if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
        previousMillis = currentMillis;
        switch (ledState) {
             case 0: {suitcolortoall(CRGB::Red, CRGB::Red, CRGB::Red);break;}
              case 1: {suitcolortoall(CRGB::White, CRGB::White, CRGB::White);break;}
              case 2: {suitcolortoall(CRGB::Blue, CRGB::Blue, CRGB::Blue);break;}
              //case 3: {sendcolortoall(CRGB::Black);   rgb=CRGB::Black;break;}
          //case 3: {{rgb[0]=black[0];rgb[1]=black[1];rgb[2]=black[2];break;}}
          }
         ledState++;
         if (ledState>2) {ledState=0;}
         }
       repeatMessage();
       break;

    }
    case 3 : {    //rainbow
      brightnessstate=0;
      CHSV hsv( hue++, 255, 255);
      hsv2rgb_rainbow( hsv, rgb);
      sendcolortoall(rgb); //delay(1);
    break;
 
    }
    case 4 : {    //floating rainbow
      brightnessstate=0;
      rainbowtoall(); //delay(1);
      break;
   
    }
    case 5 : { // pulsed rainbow
        brightnessstate=0;
        leds.fadeToBlackBy(  FADERATE);
       if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
        previousMillis = currentMillis;

        
         switch (ledState) {
        
          case 0: {for(CRGB & pixel : leds) { pixel+= CRGB::Red; }
                      break;
                      }
          case 1: {for(CRGB & pixel : leds) { pixel+= CRGB::Orange; }
                        break;
                       }
          case 2: {for(CRGB & pixel : leds) { pixel+= CRGB::Yellow; }
                       break;
                      }
         case 3: {for(CRGB & pixel : leds) { pixel+= CRGB::Green; }
                       break;
                      }
         case 4: {for(CRGB & pixel : leds) { pixel+= CRGB::Blue; }
                       break;
                      }
         case 5: {for(CRGB & pixel : leds) { pixel+= CRGB::Purple; }
                       break;
                      }
        }
        ledState++;
        if (ledState>5) ledState=0;
 
        }
        updateCoats();FastLED.show();
       
        break;

    }
  
      case 6 : { // Ceylons
         brightnessstate=1;
         leds.fadeToBlackBy( FADERATE );
          byte pos1 = beatsin16(rate/4,0,NUM_LEDS_HAT-1);
          int localpos1=beatsin16(rate/4,0,NUM_LEDS_HAT-1);
          leds[localpos1] += CRGB::Red;
 
          byte pos2 =START_LEDS_BODY+beatsin16(3+rate/4,0,NUM_LEDS_BODY-1);
          int localpos2=START_LEDS_BODY+beatsin16(3+rate/4,0,NUM_LEDS_BODY-1);
          leds[localpos2] += CRGB::White;

          byte pos3 =START_LEDS_LEGS+beatsin16(10+rate/4,0,NUM_LEDS_LEGS-1);
          int localpos3=START_LEDS_LEGS+beatsin16(10+rate/4,0,NUM_LEDS_LEGS-1);;
          leds[localpos3] += CRGB::Blue;
 
          Ceylons(CRGB::White, CRGB::Red, CRGB::Blue, pos1, pos2, pos3);
          
          FastLED.show();
          
          break;

    }
      case 7 : { // Alternate RWB interval
         brightnessstate=0;
         leds.fadeToBlackBy( FADERATE );
          updateCoats();FastLED.show();
        if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
          previousMillis = currentMillis;
        ledState++;
          if (ledState>2) {ledState=0;}
         alternate3toall(CRGB::Red, CRGB::White, CRGB::Blue, ledState);

        }
            break;

    }
          case 8 : { //Bouncy
           brightnessstate=0;
      bounceblend();break;
    }
         case 9 : { //BouncyRWB
         brightnessstate=0;
        RWBblend();break;
    }
         case 10 : { //BouncyRWB
         brightnessstate=0;
        bounceall();break;
    }
         case 11 : { //BouncyRWB
         brightnessstate=0;

 //            uint8_t pulse = beatsin8( rate, 0, 255,0,0);

         leds.fadeToBlackBy( FADERATE );
              updateCoats();FastLED.show();
            if (currentMillis - previousMillis >= interval) {
            // save the last time you blinked the LED
              previousMillis = currentMillis;
              leds=CHSV(huepot,satpot,255);
            }
            //Pulsecolor();
            break;
    }
         case 12 : { //candycane suits
         brightnessstate=0;
         leds.fadeToBlackBy( FADERATE );
          updateCoats();FastLED.show();
        if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
          previousMillis = currentMillis;
        ledState++;
        if (ledState>3) ledState=0;
        previousMillis = currentMillis;

        
         switch (ledState) {
        
          case 0: {candycane(CRGB::Red,CRGB::White,CRGB::Blue,CRGB::Black);break;}
          case 1: {candycane(CRGB::White,CRGB::Blue,CRGB::Black,CRGB::Red);break;}
          case 2: {candycane(CRGB::Blue,CRGB::Black,CRGB::Red,CRGB::White);break;}
          case 3: {candycane(CRGB::Black,CRGB::Red,CRGB::White,CRGB::Blue);break;}
        }
        //ledState++;
        }
        
 
                
        
        
        break;
    }
}}



void updateCoats()
{
       message1[0]=0;
       message2[0]=1;
    /* for (int j=0;j<10;j++){
        int startnumber=1+j*3;
        message1[startnumber]=leds[j].r; message1[startnumber+1]=leds[j].g; message1[startnumber+2]=leds[j].b;
        message2[startnumber]=leds[j+10].r; message2[startnumber+1]=leds[j+10].g; message2[startnumber+2]=leds[j+10].b;
       }*/ 
      //memmove( &leds2, &leds[0], 20 * sizeof( CRGB) );  
      //if(soundstate) leds.fadeLightBy(soundlevel);

      memmove( &message1[1], &leds[0], 10 * sizeof( CRGB) );
      memmove( &message2[1], &leds[10], 10 * sizeof( CRGB) );   
      rgb=leds[0];
    //Serial.print(redb);Serial.print("/");
    //Serial.print(greenb);Serial.print("/");
    //Serial.println(blueb);
    repeatMessage();

}

void updateBright()
{
       message3[0]=4;
       message3[1]=brightness;
       message3[2]=huepot;
       message3[3]=satpot;
       message3[4]=rate;
   radio.write(&message3,sizeof(message3),1);
   sentMessages++;
   delay(MESSAGE_WAIT);

}

void repeatMessage()
{
   radio.write(&message1,sizeof(message1),1);
   delay(MESSAGE_WAIT);
   sentMessages++;
   radio.write(&message2,sizeof(message2),1);
   delay(MESSAGE_WAIT);
   sentMessages++;
   //radio.write(&message3,sizeof(message3),1);
}


void sendcolortoall(CRGB theColor){
      leds(0,NUM_LEDS-1)=theColor;
      updateCoats();
      FastLED.show();
}


void suitcolortoall(CRGB theColor1, CRGB theColor2, CRGB theColor3){
       message1[0]=5;
       message2[0]=5;
       leds=CRGB::Black;
       leds[0]=theColor1;
       leds[1]=theColor2;
       leds[2]=theColor3;
       message1[1]=theColor1.r;
       message2[1]=theColor1.r;
       message1[2]=theColor1.g;
       message2[2]=theColor1.g;
       message1[3]=theColor1.b;
       message2[3]=theColor1.b;
       message1[4]=theColor2.r;
       message2[4]=theColor2.r;
       message1[5]=theColor2.g;
       message2[5]=theColor2.g;
       message1[6]=theColor2.b;
       message2[6]=theColor2.b;
       message1[7]=theColor3.r;
       message2[7]=theColor3.r;
       message1[8]=theColor3.g;
       message2[8]=theColor3.g;
       message1[9]=theColor3.b;
       message2[9]=theColor3.b;
      repeatMessage();
      FastLED.show();
}

void suitVUmeters(CRGB theColor1, CRGB theColor2, CRGB theColor3, int bright){
       message1[0]=6;
       message2[0]=6;
       leds=CRGB::Black;
       int column1=map(bright, 0 ,255,0,5);
       int column2=map(bright, 0 ,255,5,0);
       int column3=map(bright, 0 ,255,0,5);
       fill_solid(&leds[0],column1,theColor1);
       fill_solid(&leds[5+column2],5-column2,theColor2);
       fill_solid(&leds[10],column3,theColor3);
       
       //leds[0]=theColor1;
       //leds[1]=theColor2;
       //leds[2]=theColor3;
       message1[1]=theColor1.r;
       message2[1]=theColor1.r;
       message1[2]=theColor1.g;
       message2[2]=theColor1.g;
       message1[3]=theColor1.b;
       message2[3]=theColor1.b;
       message1[4]=theColor2.r;
       message2[4]=theColor2.r;
       message1[5]=theColor2.g;
       message2[5]=theColor2.g;
       message1[6]=theColor2.b;
       message2[6]=theColor2.b;
       message1[7]=theColor3.r;
       message2[7]=theColor3.r;
       message1[8]=theColor3.g;
       message2[8]=theColor3.g;
       message1[9]=theColor3.b;
       message2[9]=theColor3.b;
       message1[10]=bright;
       message2[10]=bright;
      repeatMessage();
      FastLED.show();
}

void Ceylons(CRGB theColor1, CRGB theColor2, CRGB theColor3, byte pos1, byte pos2, byte pos3)
{
       message1[0]=7;
       message2[0]=7;
       message1[1]=theColor1.r;
       message2[1]=theColor1.r;
       message1[2]=theColor1.g;
       message2[2]=theColor1.g;
       message1[3]=theColor1.b;
       message2[3]=theColor1.b;
       message1[4]=theColor2.r;
       message2[4]=theColor2.r;
       message1[5]=theColor2.g;
       message2[5]=theColor2.g;
       message1[6]=theColor2.b;
       message2[6]=theColor2.b;
       message1[7]=theColor3.r;
       message2[7]=theColor3.r;
       message1[8]=theColor3.g;
       message2[8]=theColor3.g;
       message1[9]=theColor3.b;
       message2[9]=theColor3.b;
       message1[10]=pos1;
       message2[10]=pos1;
       message1[11]=pos2;
       message2[11]=pos2;
       message1[12]=pos3;
       message2[12]=pos3;
      repeatMessage();
      //FastLED.show();

}

void alternate3toall(CRGB color1, CRGB color2, CRGB color3, int thestart){
      message1[0]=0;
      message2[0]=1;
      //byte redb, greenb, blueb;

      int thecolor=thestart;
      
      for (int j=0;j<20;j++){
        if (thecolor==0) leds[j]=color1;
        if (thecolor==1) leds[j]=color2;
        if (thecolor==2) leds[j]=color3;
        thecolor++; if (thecolor>2) thecolor=0;
       } 
      updateCoats();   FastLED.show();
}


void rainbowtoall(void){
/*      message1[0]=0;
      message2[0]=1;
      byte redb, greenb, blueb;
*/
      leds(0,NUM_LEDS/2 - 1).fill_rainbow(hue+=2);
      leds(NUM_LEDS/2, NUM_LEDS-1) = leds(NUM_LEDS/2-1,0);
      updateCoats();
      FastLED.show();
}



void crossFade(CRGB start, CRGB color, int millistoFade) {
  long theStartFade=millis();

  while ((millis()-theStartFade)<millistoFade){
    long timenow=millis()-theStartFade;
    //fract8 fractTime=ease8InOutCubic(timenow/millistoFade);
    fract8 fractTime=timenow/millistoFade;
    rgb=blend(start,color,fractTime);
      sendcolortoall(rgb);
    }
  sendcolortoall(color);rgb=color;
}


void bounceblend() {
    uint8_t speed1 = beatsin8( 50, 0, 255);
    uint8_t speed2 = beatsin8( 100, 0, 255);
    CHSV startclr = blend(CHSV(0,255,speed2) , CHSV(160,255,speed2), speed1);
    CHSV middlecolor = CHSV(0,0,speed2);
    CHSV endclr = blend(CHSV(160,255,speed2), CHSV(0,255,speed2) , speed1);
    //fill_gradient_RGB(leds, 0, startclr, NUM_LEDS-1, endclr);
    leds(0,NUM_LEDS/2).fill_gradient(startclr, middlecolor, SHORTEST_HUES);
    leds(NUM_LEDS/2,NUM_LEDS).fill_gradient(middlecolor, endclr, SHORTEST_HUES);
   FastLED.show();
    updateCoats();
}

void RWBblend() {
    uint8_t speedR = beatsin8( 30, 0, 255,0,0);
    uint8_t speedW = beatsin8( 60, 0, 255,0,63);
    uint8_t speedB = beatsin8( 30, 0, 255,0,127);
    CRGB bColor = blend(CRGB::Black , CRGB::White, speedW);
    bColor=blend(bColor,CRGB::Blue,speedB);
    bColor=blend(bColor,CRGB::Red,speedR);
    leds=bColor;
   FastLED.show();
    updateCoats();
}

void Pulsecolor() {
    uint8_t pulse = beatsin8( rate, 0, 255,0,0);

     leds=CHSV(huepot,satpot,pulse);
   FastLED.show();
    updateCoats();
}

void Chasefour() {
  for (int i=0;i<NUM_LEDS;i++) {
    int curpos=(i+startcolor) % 8;
    switch (curpos){
      case 0 : {leds[i]=CRGB::Red; break;}
      case 1 : {leds[i]=CRGB::White; break;}
      case 2 : {leds[i]=CRGB::Blue; break;}
      default : {leds[i]=CRGB::Black;}
    }
  }
  if (millis()>previousMillis+interval){
    previousMillis=millis();
    startcolor+=1;
    if (startcolor>7) startcolor=0;

  }
   message1[0]=2;
   message2[0]=2;
   message1[1]=255;
   message2[1]=255;
   message1[2]=0;
   message2[2]=0;
   message1[3]=0;
   message2[3]=0;
   message1[4]=255;
   message2[4]=255;
   message1[5]=255;
   message2[5]=255;
   message1[6]=255;
   message2[6]=255;
   message1[7]=0;
   message2[7]=0;
   message1[8]=0;
   message2[8]=0;
   message1[9]=255;
   message2[9]=255;
   message1[10]=0;
   message2[10]=0;
   message1[11]=0;
   message2[11]=0;
   message1[12]=0;
   message2[12]=0;
   message1[13]=startcolor;
   message2[13]=startcolor;
   
   repeatMessage();

}



void bounceall() {
    uint8_t starthue = beatsin8(20, 0, 255);
    uint8_t endhue = beatsin8(24, 0, 255);
       message1[0]=3;
       message2[0]=3;
       leds=CRGB::Black;
         if (starthue < endhue) {
          leds.fill_gradient(CHSV(starthue,255,255), CHSV(endhue,255,255), FORWARD_HUES);    // If we don't have this, the colour fill will flip around. 
        } else {
          leds.fill_gradient(CHSV(starthue,255,255), CHSV(endhue,255,255),BACKWARD_HUES);
        }
       message1[1]=starthue;
       message2[1]=starthue;
       message1[2]=255;
       message2[2]=255;
       message1[3]=255;
       message2[3]=255;
       message1[4]=endhue;
       message2[4]=endhue;
       message1[5]=255;
       message2[5]=255;
       message1[6]=255;
       message2[6]=255;
       
       
       FastLED.show();
   
      rgb=leds[0];
    //Serial.print(redb);Serial.print("/");
    //Serial.print(greenb);Serial.print("/");
    //Serial.println(blueb);
    repeatMessage();

    
}

void candycane(CRGB color1, CRGB color2, CRGB color3, CRGB color4) {
       message1[0]=8;
       message2[0]=8;
       message1[1]=color1.r;
       message2[1]=color1.r;
       message1[2]=color1.g;
       message2[2]=color1.g;
       message1[3]=color1.b;
       message2[3]=color1.b;
       message1[4]=color2.r;
       message2[4]=color2.r;
       message1[5]=color2.g;
       message2[5]=color2.g;
       message1[6]=color2.b;
       message2[6]=color2.b;
       message1[7]=color3.r;
       message2[7]=color3.r;
       message1[8]=color3.g;
       message2[8]=color3.g;
       message1[9]=color3.b;
       message2[9]=color3.b;
       message1[10]=color4.r;
       message2[10]=color4.r;
       message1[11]=color4.g;
       message2[11]=color4.g;
       message1[12]=color4.b;
       message2[12]=color4.b;
       
       repeatMessage();
    for (int i=0;i<NUM_LEDS;i++) {
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
