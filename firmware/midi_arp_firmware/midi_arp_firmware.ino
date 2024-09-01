#include <MIDI.h>

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

const uint8_t CLOCK_IN_PIN = 32;
const uint8_t RESET_IN_PIN = 6;
const uint8_t CV_OUT_PIN = A21;
const uint8_t GATE_OUT_PIN = 12;
const uint8_t TRIGGER_OUT_PIN = 4;
const uint8_t HOLD_LED_OUT_PIN = 2;

const uint8_t MAX_NOTE = 60;
const double MAX_VOLTS_OUT = 5.36;

uint8_t notesOnSorted[128];
uint8_t notesOnCount;

int arp;
int16_t arpIndex;
int8_t arpDirection;

double pitchBend;
bool holdMode;

// Software clock
bool softwareClockEnabled;
uint64_t prevTime;
double unprocessedTime;
double secondsPerPhase;
bool clockEdge;

// Hardware clock
bool prevExtClock;

bool triggerOn;

enum Arp {
  ARP_FORWARD = 0,
  ARP_REVERSE = 1,
  ARP_UP_DOWN = 2,
  ARP_UP_DOWN_REPEAT = 3,
  ARP_RANDOM_NOTE = 4,
  ARP_RANDOM_WALK = 5,
  ARP_WRAPPING_RANDOM_WALK = 6,
  ARP_WRAPPING_RANDOM_WALK_MOSTLY_RIGHT = 7,
  ARP_COUNT = 8 // Must be last
};

void fullReset() {
  notesOnCount = 0;
  arpIndex = 0;
  pitchBend = 0.0;
  arpDirection = 1;
  holdMode = false;
}

void setup() {
  fullReset();

  arp = ARP_REVERSE;

  pinMode(CLOCK_IN_PIN, INPUT_PULLUP);
  pinMode(RESET_IN_PIN, INPUT_PULLUP);
  pinMode(GATE_OUT_PIN, OUTPUT);
  pinMode(TRIGGER_OUT_PIN, OUTPUT);
  pinMode(HOLD_LED_OUT_PIN, OUTPUT);

  // TODO: Implement these
  digitalWrite(TRIGGER_OUT_PIN, HIGH);
  digitalWrite(HOLD_LED_OUT_PIN, LOW);

  digitalWrite(GATE_OUT_PIN, HIGH);
  analogWriteResolution(12);
  analogWrite(CV_OUT_PIN, 0);
  prevExtClock = !digitalRead(CLOCK_IN_PIN);

  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandlePitchBend(handlePitchBend);
  MIDI.setHandleControlChange(handleControlChange);
  MIDI.setHandleSystemReset(handleSystemReset);
  MIDI.begin();
  Serial1.begin(31250, SERIAL_8N1_RXINV);

  softwareClockEnabled = false;
  prevTime = millis();
  clockEdge = false;
  unprocessedTime = 0.0;
  secondsPerPhase = 0.07;
  pinMode(LED_BUILTIN, OUTPUT);

  triggerOn = false;
}

void forward() {
  arpIndex++;
  arpIndex %= notesOnCount;
}

void reverse() {
  if (notesOnCount != 0)
    arpIndex--;
  if (arpIndex < 0)
    arpIndex = notesOnCount - 1;
}

void upDown() {
  arpIndex += arpDirection;
  if (arpIndex >= notesOnCount - 1) {
    arpIndex = notesOnCount - 1;
    arpDirection = -1;
  }
  if (arpIndex <= 0) {
    arpIndex = 0;
    arpDirection = 1;
  }
}

void upDownRepeat() {
  arpIndex += arpDirection;
  if (arpIndex >= notesOnCount) {
    arpIndex = notesOnCount - 1;
    arpDirection = -1;
  }
  if (arpIndex < 0) {
    arpIndex = 0;
    arpDirection = 1;
  }
}

void randomNote() {
  uint8_t value = random(0, notesOnCount - 1);
  // Don't do same one twice in a row
  if (value >= arpIndex) {
    value++;
  }
  arpIndex = value;
}

void randomWalk() {
  if (random(0, 2)) {
    arpIndex++;
    if (arpIndex > notesOnCount - 1)
      arpIndex = notesOnCount - 1;
  } else {
    arpIndex--;
    if (arpIndex < 0)
      arpIndex = 0;
  }
}

void wrappingRandomWalk() {
  if (random(0, 2)) {
    forward();
  } else {
    reverse();
  }
}

void wrappingRandomWalkMostlyRight() {
  if (random(0, 100) < 60) {
    forward();
  } else {
    reverse();
  }
}

void doArp(int a) {
  switch (a) {
    case ARP_FORWARD: forward(); break;
    case ARP_REVERSE: reverse(); break;
    case ARP_UP_DOWN: upDown(); break;
    case ARP_UP_DOWN_REPEAT: upDownRepeat(); break;
    case ARP_RANDOM_NOTE: randomNote(); break;
    case ARP_RANDOM_WALK: randomWalk(); break;
    case ARP_WRAPPING_RANDOM_WALK: wrappingRandomWalk(); break;
    case ARP_WRAPPING_RANDOM_WALK_MOSTLY_RIGHT: wrappingRandomWalkMostlyRight(); break;
    default:
      // Forgot to implement
      break;
  }
}

void loop() {
  MIDI.read();

  if (notesOnCount == 0) {
    arpIndex = 0;
    digitalWrite(GATE_OUT_PIN, HIGH); // Gate output is inverted
    digitalWrite(TRIGGER_OUT_PIN, HIGH);
  } else {
    writeCV();
    digitalWrite(GATE_OUT_PIN, LOW);
    digitalWrite(TRIGGER_OUT_PIN, !triggerOn);
  }

  bool reset = !digitalRead(RESET_IN_PIN);
  if (reset) {
    arpIndex = 0;
    arpDirection = 1;
    digitalWrite(LED_BUILTIN, LOW);
    writeCV();
    return;
  }

  if (softwareClockEnabled) {
    uint64_t time = millis();
    uint64_t passedTime = time - prevTime;
    prevTime = time;
    unprocessedTime += passedTime / 1000.0;
    while (unprocessedTime > secondsPerPhase) {
      if (clockEdge) {
        clockRising();
      } else {
        clockFalling();
      }
      clockEdge = !clockEdge;
      unprocessedTime -= secondsPerPhase;
    }
  } else {
    bool clock = !digitalRead(CLOCK_IN_PIN);
    if (clock && !prevExtClock)
      clockRising();
    if (!clock && prevExtClock)
      clockFalling();
    prevExtClock = clock;
  }
}

void clockRising() {
  if (notesOnCount > 1)
    doArp(arp);
  else if (notesOnCount == 1)
    arpIndex = 0;

  triggerOn = true;
}

void clockFalling() {
  triggerOn = false;
}

void writeCV() {
  double note = notesOnSorted[arpIndex];
  note = min(note, 12 * 5);
  note += pitchBend;

  // 3947.206 is the output required for the CV to be 5V
  double outputFor5V = 3947.206;
  analogWrite(CV_OUT_PIN, (int) ((double) note / 60.0 * outputFor5V));
}

void handleNoteOn(byte channel, byte note, byte velocity) {
  // if (note >= 60 && note < 60 + ARP_COUNT) {
  //   arp = note - 60;
  // }

  for (uint8_t i = 0; i < notesOnCount; i++) {
    uint8_t currentNote = notesOnSorted[i];
    
    if (currentNote == note) {
      // Note already on
      return;
    }

    if (currentNote > note) {
      // Move everything after down by one
      uint8_t after = notesOnCount - i;
      memmove(&notesOnSorted[i + 1], &notesOnSorted[i], after);

      notesOnSorted[i] = note;
      notesOnCount++;

      if (arpIndex >= i)
        arpIndex++;

      return;
    }
  }

  // Note is highest, add to end of list
  notesOnSorted[notesOnCount] = note;
  notesOnCount++;
}

void handleNoteOff(byte channel, byte note, byte velocity) {
  for (uint8_t i = 0; i < notesOnCount; i++) {
    uint8_t currentNote = notesOnSorted[i];
    if (currentNote == note) {
      // Remove this note
      uint8_t after = notesOnCount - i - 1;
      if (after > 0)
        memmove(&notesOnSorted[i], &notesOnSorted[i + 1], after);
      notesOnCount--;

      if (arpIndex > i)
        arpIndex--;
    }
  }
}

void handlePitchBend(byte channel, int bend) {
  pitchBend = (double) bend / 8192.0 * 2.0;
}

void handleControlChange(byte channel, byte control, byte value) {
  Serial.print("CC ");
  Serial.print(channel);
  Serial.print(" ");
  Serial.print(control);
  Serial.print(" ");
  Serial.println(value);

  if (control == 16 || control == 74) {
    float arpPct = (float) value / 128.0f;
    int prevArp = arp;
    arp = (int) (arpPct * ARP_COUNT);

    if (prevArp != arp) {
      Serial.print("Arp: ");
      Serial.println(arp);
    }
  } else if (control == 80 || control == 23) {
    // Toggle hold mode
    holdMode ^= (value == 127);
  }
}

void handleSystemReset() {
  fullReset();
}
