// made by me: http://www.synthbror.com
// but includes borrowed code from  http://www.kerrywong.com/2012/07/25/code-for-mcp4821-mcp4822/
// and this project https://little-scale.blogspot.com/2019/01/12-gate-16-cv-usb-midi-interface-bom.html

#include <MIDI.h>        // access to serial (5 pin DIN) MIDI
#include <USBHost_t36.h> // access to USB MIDI devices (plugged into 2nd USB port)
#include <SPI.h>
USBHost myusb;
USBHub hub1(myusb);
USBHub hub2(myusb);
USBHub hub3(myusb);
USBHub hub4(myusb);
MIDIDevice midi01(myusb);
MIDIDevice midi02(myusb);
MIDIDevice midi03(myusb);
MIDIDevice midi04(myusb);
MIDIDevice midi05(myusb);
MIDIDevice midi06(myusb);
MIDIDevice midi07(myusb);
MIDIDevice midi08(myusb);
MIDIDevice midi09(myusb);
MIDIDevice midi10(myusb);
MIDIDevice * midilist[10] = {
  &midi01, &midi02, &midi03, &midi04, &midi05, &midi06, &midi07, &midi08, &midi09, &midi10
};

#define debug Serial
#define num_dac 12
const int num_cs = num_dac / 2;
const unsigned int maximum_notes = num_dac;

const int LED_pin = 13;  //pin of LED
const int MOSI_pin(28);
const int SCK_pin(27);
int cs_pin[num_cs] =  {2, 3, 4, 5, 6, 7}; // chip select pins
const int trimpot_pin;
const int pitchbend_value_positive = 1200;
const int pitchbend_value_negative = -1200;
int pitchbend_value;
const int mcpGain[] = {0x0, 0x0, 0x2}; //both mcpGain[0] and mcpGain[1] will be valid for x1 gain on DAC, mcpGain[2] for x2
const int mcpChan[] =  {0, 0, 1}; //both mcpChan[0] and mcpChan[1] will be valid for channel 1, mcpChan[2] for channel2
const int mcpShutdown[] = {0, 1, 1};

int gateIndex[num_dac];
bool gateMode = true;
bool unisonMode =  false;
const unsigned int gateVoltage = 4095;
unsigned int dacNumber = 0;
unsigned int dacNoteVoltage_min = 0;
unsigned int dacNoteVoltage_max = 4000;
unsigned int midiNoteRange = 48; //4 volts = 4 octaves in CV (1v/oct) and 4 octaves = 48 midi notes
unsigned int midiOffset = 36;
unsigned int polyIndex[num_dac];
//store previous mcp transmit
byte prevDac_chan = 0;
byte prevDac_gain = 0;
byte prevDac_shutdown = 0;
unsigned int prevDac_dacNote = 0;
unsigned int prevDac_dacNumber = 0;
//stored and unused functions
//byte clock_tick;
//byte clock_value;
//byte play_flag;
//byte play_tick;
//float voltage_range = 4.024 * 1200;
//const float offset_pitch = 48; //Key 48 on midi-keyboard will be at dac voltage 0.0
void setup() {
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();
  SPI.setMOSI(MOSI_pin);
  SPI.setSCK(SCK_pin);
  for (int i = 0; i < num_cs; i ++) {
    pinMode(cs_pin[i], OUTPUT);
    digitalWriteFast(cs_pin[i], HIGH);
    //    writeGate(i, HIGH);
    delay(50);
  }
  delay(100);
  myusb.begin();
  debug.begin(115200);
  debug.println("started");
  if (gateMode == true)
  {
    dacNumber = 1;
  }
  for (int i = 1; i <= num_dac; i ++) {
    transmitDac(mcpGain[2], mcpShutdown[1], 0000, i); //set DACs to 0 voltage
  }
}

void loop() {
  // //high-res triangular wave
  // for (int i=0; i < 4096; i+=32)
  // {
  //  //

  //(0, GAIN_2, 1, i);
  //  transmitDac(i);
  // }
  //}
  digitalWrite(13, HIGH);
  for (int port = 0; port < 10; port++) {
    if (midilist[port]->read()) {
      uint8_t type =       midilist[port]->getType();
      uint8_t data1 =      midilist[port]->getData1();
      uint8_t data2 =      midilist[port]->getData2();
      uint8_t channel =    midilist[port]->getChannel();
      const uint8_t *sys = midilist[port]->getSysExArray();
      midiHandle(type, data1, data2, channel);
    }
  }

}

void midiHandle(uint8_t type, uint8_t data1, uint8_t data2, uint8_t channel)
{
  midi::MidiType midiType = (midi::MidiType)type;
       int midiNote = data1 - midiOffset;
               int midiVelocity = data2;
  switch (midiType)
  {
    case midi::NoteOn:
      {  
        if (unisonMode==false)
        {
        handlePoly(midiNote,  midiVelocity);
        debug.print("["); debug.print(channel); debug.print(" | "); debug.print(type); debug.print(" | "); debug.print(data1); debug.print(" | "); debug.print(data2);     debug.println("]");
        break;
      }
      if (unisonMode==true)
      {

          unisonDac(mcpGain[2], mcpShutdown[1], midiNote);
          break;
      }
      break;
      }
      
    case midi::NoteOff:
      {
        if (unisonMode==0)
        {
        gateOff(midiNote);
        break;
        }
                if (unisonMode==1)
        {
        gateOff(midiNote);
        break;
        }
         break;
      }
  }
}

void unisonDac(byte gain, byte shutdown,  int midiNote) //
{
    unsigned int dacNote = map(midiNote + pitchbend_value, 0, midiNoteRange, dacNoteVoltage_min, dacNoteVoltage_max);

 for (int dacNumber = 1; dacNumber <= num_dac; dacNumber++)
 {
    if (gateMode==true)
 {
  
    if (dacNumber % 2)
  {
   // debug.println("dacNumber is odd number");
  transmitDac(mcpGain[2], mcpShutdown[1], dacNote, dacNumber);
  }  

  if ( (dacNumber % 2) == 0)
  {
    transmitDac(mcpGain[2], mcpShutdown[1], gateVoltage, dacNumber);
    gateIndex[dacNumber] = midiNote;
    polyIndex[dacNumber] = gateVoltage; //occupy the gate DAC in poly-memory
    debug.println("Send gate voltage to channel B....");
  }
 }
  
  polyIndex[dacNumber] = dacNote;
  debug.print("debug: cs_pin[");
  debug.print(cs_pin[dacNumber]);
      debug.print("]");
          debug.print(" - ");
              debug.print(dacNumber);
    debug.print("/");
  debug.println(num_cs);
 }
}

void gateOff(int midiNote)
{
  for (int dacSearch = 1; dacSearch <= num_dac; dacSearch++)
    if (gateIndex[dacSearch] == midiNote)
    {
       gateIndex[dacSearch] = 0000;
      transmitDac(mcpGain[2], mcpShutdown[1], 0000, dacSearch);
    //  transmitDac(mcpGain[2], mcpShutdown[1], 0000, dacSearch - 1); 
      polyIndex[dacSearch] = 4090; //fool the poly memory to believe the GATE of this DAC has a high note so it doesnt get overrid by new poly notes
      polyIndex[dacSearch - 1] = 0;
      //This 'for' loop searches for a matching midinote in the gateIndex to see if it triggered a gate earlier and it was stored
      debug.println("Found midiNote match when scanning the gate index, turning gate off");
    }
}
void handlePoly(unsigned int midiNote, unsigned int midiVelocity)
{

  unsigned int dacVelocity = midiVelocity;
  if (gateMode == true)
  {
    dacNumber += 2;
  }
  if (gateMode == false)
  {
    dacNumber++;
  }

  if (dacNumber > num_dac)
  {
    dacNumber = 1;
  }
  if (polyIndex[dacNumber] > 0) //if occupied DAC
  {
    debug.println("------------------------------------------------");
    for (int polySearch = 1; polySearch <= num_dac; polySearch++)
    {
      if (polyIndex[polySearch] == 0)
      {
        dacNumber = polySearch;
        debug.println("*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-");
        debug.print("DAC #"); debug.print(polySearch);  debug.print("  *** EMPTY **** [");  debug.print(polyIndex[polySearch]); debug.println("]");
        debug.println("*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-*-*_*-_*-");
        debug.println(dacNumber);
      }
      else {
        debug.print("DAC #"); debug.print(polySearch);  debug.print(" OCCUPIED WITH [");  debug.print(polyIndex[polySearch]); debug.println("]");
        debug.println("------------------------------------------------");
      }
    }
  }

  unsigned int dacNote = map(midiNote + pitchbend_value, 0, midiNoteRange, dacNoteVoltage_min, dacNoteVoltage_max);
  int gain = mcpGain[2];
  int shutdown = mcpShutdown[1];
  transmitDac(gain, shutdown,  dacNote, dacNumber);
  polyIndex[dacNumber] = dacNote;
  if (gateMode == true)
  {
    transmitDac(gain, shutdown,  gateVoltage, dacNumber + 1);
    gateIndex[dacNumber + 1] = midiNote;
    polyIndex[dacNumber + 1] = gateVoltage; //occupy the gate DAC in poly-memory
    debug.println("Send gate voltage to channel B....");
  }
  debug.print("[ midiNote=");
  debug.print(midiNote);
  debug.print(" ]");
  debug.print("[ dacNote=");
  debug.print(dacNote);
  debug.print(") dacNumber:");
  debug.println(dacNumber);

}



//void OnPitchChange (byte channel, int pitch_change) {
//  if (channel < 9) {
//    pitchbend_value[channel - 1] = map(pitch_change, 0, 16383, pitchbend_value_negative, pitchbend_value_positive);
//    writeDAC(cs_pin - channel + 1, 1, constrain(map((pitch_values[channel - 1] - offset_pitch) * 100.0 + pitchbend_value[channel - 1], 0.0, voltage_range, 0.0, 4095.0), 0.0, 4095.0));
//  }
//}


void transmitDac(byte gain, byte shutdown,  unsigned int dacNote, int dacNumber) //
{

  byte mcpChannel;
  if (dacNumber % 2)
  {
   // debug.println("dacNumber is odd number");
    mcpChannel = mcpChan[1];
        debug.println("***********");
  debug.print("OSCILLATOR #");
    debug.println(dacNumber/2+1);
        debug.println("***********");
  }

  if ( (dacNumber % 2) == 0)
  {
  //  debug.println("dacNumber is even number, might be the triggered gate?");
    mcpChannel = mcpChan[2];
  }

  int chipSelect = map(dacNumber, 1, num_dac, 0, num_dac * 0.5 - 1);
  byte lowByte = dacNote &  0xff;
  byte highByte = ((dacNote >> 8) & 0xff) | mcpChannel << 7 | gain << 5 | shutdown << 4;
  digitalWriteFast(cs_pin[chipSelect], LOW);
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  digitalWriteFast(cs_pin[chipSelect], HIGH);
//  if (dacNote < gateVoltage)
//  {
    prevDac(gain, shutdown, dacNote, dacNumber);
    debug.print("transmit to DAC #"); debug.print(dacNumber); debug.print(" Channel:"); debug.print(mcpChannel); debug.print(", MCP gain="); debug.print(gain); debug.print(", MCP shutdown=");
    debug.print(shutdown);  debug.print(", Voltage="); debug.print(dacNote); debug.print(", chipselect = cs_pin[");
    debug.print(cs_pin[chipSelect]); debug.println("]");
//  }


}

void prevDac(byte gain, byte shutdown,  unsigned int dacNote, int dacNumber) //byte gain, byte shutdown,
{ 
  prevDac_gain = gain;
  prevDac_dacNote = dacNote;
  prevDac_shutdown = shutdown;
}
