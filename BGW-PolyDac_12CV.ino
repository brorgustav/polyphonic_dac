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

unsigned int selectDac = 0;
unsigned int dacVoltage_min = 0;
unsigned int dacVoltage_max = 4000;
unsigned int midiNoteRange = 48; //4 volts = 4 octaves in CV (1v/oct) and 4 octaves = 48 midi notes
unsigned int midiOffset = 36;
unsigned int polyNote[maximum_notes];
//store previous mcp transmit
byte prevDac_chan = 0;
byte prevDac_gain = 0;
byte prevDac_shutdown = 0;
unsigned int prevDac_dacNote = 0;
unsigned int prevDac_selectDac = 0;
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
}

void loop() {
  // //high-res triangular wave
  // for (int i=0; i < 4096; i+=32)
  // {
  //  //transmitDac(0, GAIN_2, 1, i);
  //  transmitDac(i);
  // }
  //}
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
  switch (midiType)
  {
    case midi::NoteOn:
      {
        int midiNote = data1 - midiOffset;
        int midiVelocity = data2;
        handlePoly(midiNote,  midiVelocity);
        debug.print("["); debug.print(channel); debug.print(" | "); debug.print(type); debug.print(" | "); debug.print(data1); debug.print(" | "); debug.print(data2);     debug.println("]");
        break;
      }
    case midi::NoteOff:
      {
        break;
      }
  }
}

void handlePoly(unsigned int midiNote, unsigned int midiVelocity)
{
  debug.print("[ midiNote=");
  debug.print(midiNote);
  debug.print(" ]");
  unsigned int dacNote = map(midiNote + pitchbend_value, 0, midiNoteRange, dacVoltage_min, dacVoltage_max);
  debug.print("[ dacNote=");
  debug.print(dacNote);
  debug.println(" ]");
  unsigned int dacVelocity = midiVelocity;
  selectDac++;
  if (selectDac > num_dac)
  {
    selectDac = 1;
  }
  polyNote[selectDac] = dacNote;
  int gain = mcpGain[2];
  int shutdown = mcpShutdown[1];
  transmitDac(gain, shutdown,  dacNote, selectDac);
  debug.print("void handlePoly(unsigned int");
  debug.print(dacNote);
  debug.print(", ");
  debug.print("unsigned int ");
  debug.print(midiVelocity);
  debug.print(") selectDac:");
  debug.println(selectDac);

}


//void OnPitchChange (byte channel, int pitch_change) {
//  if (channel < 9) {
//    pitchbend_value[channel - 1] = map(pitch_change, 0, 16383, pitchbend_value_negative, pitchbend_value_positive);
//    writeDAC(cs_pin - channel + 1, 1, constrain(map((pitch_values[channel - 1] - offset_pitch) * 100.0 + pitchbend_value[channel - 1], 0.0, voltage_range, 0.0, 4095.0), 0.0, 4095.0));
//  }
//}


void transmitDac(byte gain, byte shutdown,  unsigned int dacNote, int selectDac) //
{

  int mcpChannel;
  if (selectDac % 2)
  {
    debug.println("selectDac is odd number");
    mcpChannel = mcpChan[1];
  }

  if ( (selectDac % 2) == 0)
  {
    debug.println("selectDac is even number");
    mcpChannel = mcpChan[2];
  }

  int chipSelect = map(selectDac, 1, num_dac, 0, num_dac * 0.5 - 1);
  byte lowByte = dacNote &  0xff;
  byte highByte = ((dacNote >> 8) & 0xff) | mcpChannel << 7 | gain << 5 | shutdown << 4;
  digitalWriteFast(cs_pin[chipSelect], LOW);
  SPI.transfer(highByte);
  SPI.transfer(lowByte);
  digitalWriteFast(cs_pin[chipSelect], HIGH);
  prevDac(gain, shutdown, dacNote, selectDac);

  debug.print("transmitDac(mcpChannel="); debug.print(mcpChannel); debug.print(", gain="); debug.print(gain); debug.print(", shutdown=");
  debug.print(shutdown);  debug.print(", dacNote="); debug.print(dacNote); debug.print(", chipselect = cs_pin[");
  debug.print(cs_pin[chipSelect]); debug.println("]");
}

void prevDac(byte gain, byte shutdown,  unsigned int dacNote, int selectDac) //byte gain, byte shutdown,
{
  prevDac_gain = gain;
  prevDac_dacNote = dacNote;
  prevDac_shutdown = shutdown;

  debug.println("prevDac() - stored prev");
}
