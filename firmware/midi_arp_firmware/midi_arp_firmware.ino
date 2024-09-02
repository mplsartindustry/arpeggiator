/*

    mplsartindustry/arpeggiator
    Copyright (c) 2024 held jointly by the individual authors.

    This file is part of mplsartindustry/arpeggiator.

    mplsartindustry/arpeggiator is free software: you can redistribute
    it and/or modify it under the terms of the GNU General Public License
    as published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    mplsartindustry/arpeggiator is distributed in the hope that it will
    be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
    of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with mplsartindustry/arpeggiator.  If not, please see
    <http://www.gnu.org/licenses/>.

*/

#include <MIDI.h>
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

const uint8_t PIN_CLOCK_IN = 32;
const uint8_t PIN_RESET_IN = 6;
const uint8_t PIN_GATE_OUT = 12;
const uint8_t PIN_TRIGGER_OUT = 4;
const uint8_t PIN_CV_OUT = A21;
const uint8_t PIN_HOLD_LED = 2;

const uint8_t CC_ARP_KNOB_A = 16;
const uint8_t CC_ARP_KNOB_B = 74;
const uint8_t CC_ORDER_KNOB_A = 17;
const uint8_t CC_ORDER_KNOB_B = 71;
const uint8_t CC_HOLD_TOGGLE_BUTTON_A = 23;
const uint8_t CC_HOLD_TOGGLE_BUTTON_B = 80;
const uint8_t CC_CLEAR_BUTTON_A = 22;
const uint8_t CC_CLEAR_BUTTON_B = 81;

const double CV_OUT_FOR_5V = 3947.206;

// MIDI notes are from 0-127
const uint8_t MAX_NOTES = 128;

struct DownNoteSet {
  uint8_t notes[MAX_NOTES];
  uint8_t noteCount;

  DownNoteSet(): noteCount(0) {}

  void addNote(uint8_t note) {
    // Check if note is already contained
    for (uint8_t i = 0; i < noteCount; i++) {
      if (notes[i] == note) {
        return;
      }
    }

    notes[noteCount] = note;
    noteCount++;
  }

  void removeNote(uint8_t note) {
    for (uint8_t i = 0; i < noteCount; i++) {
      if (notes[i] == note) {
        noteCount--;
        
        // Fill in the gap
        if (i < noteCount) {
          notes[i] = notes[noteCount];
        }
      }
    }
  }

  void clear() {
    noteCount = 0;
  }
};

struct NoteIndices {
  int8_t sorted;
  int8_t ordered;
};

struct PlayingNoteSet {
  uint8_t sorted[MAX_NOTES];
  uint8_t ordered[MAX_NOTES];
  uint8_t noteCount;

  PlayingNoteSet(): noteCount(0) {}

  // Returns the indices the note was added, or -1 if it was already in the set
  NoteIndices addNote(uint8_t note) {
    NoteIndices indices;
    indices.sorted = addSorted(note);
    indices.ordered = addOrdered(note);

    if (indices.sorted >= 0) {
      noteCount++;
    }

    return indices;
  }

  // Returns the indices of the note in the set, or -1 if it wasn't there
  NoteIndices removeNote(uint8_t note) {
    NoteIndices indices;
    indices.sorted = remove(note, sorted);
    indices.ordered = remove(note, ordered);

    if (indices.sorted >= 0) {
      noteCount--;
    }

    return indices;
  }

  void clear() {
    noteCount = 0;
  }

private:
  int8_t addSorted(uint8_t note) {
    for (uint8_t i = 0; i < noteCount; i++) {
      uint8_t currentNote = sorted[i];

      if (currentNote == note) {
        // Note already on
        return -1;
      }

      if (currentNote > note) {
        // Insert note before currentNote

        // Move everything after forward by one
        uint8_t notesAfter = noteCount - i;
        memmove(&sorted[i + 1], &sorted[i], notesAfter);

        // Put the note in the new gap
        sorted[i] = note;

        return i;
      }
    }

    // Note is the highest, put it at the end
    sorted[noteCount] = note;
    return noteCount;
  }

  int8_t addOrdered(uint8_t note) {
    // Check if note is already contained
    for (uint8_t i = 0; i < noteCount; i++) {
      if (ordered[i] == note) {
        return -1;
      }
    }

    ordered[noteCount] = note;
    return noteCount;
  }

  int8_t remove(uint8_t note, uint8_t *notes) {
    for (uint8_t i = 0; i < noteCount; i++) {
      if (notes[i] == note) {
        // Move all notes after this back by one
        uint8_t notesAfter = noteCount - i - 1;
        if (notesAfter > 0) {
          memmove(&notes[i], &notes[i + 1], notesAfter);
        }

        return i;
      }
    }

    return -1;
  }
};

enum Arp : uint8_t {
  ARP_FORWARD = 0,
  ARP_REVERSE,
  ARP_UP_DOWN,
  ARP_UP_DOWN_REPEAT,
  ARP_RANDOM_NOTE,
  ARP_RANDOM_WALK,
  ARP_WRAPPING_RANDOM_WALK,
  ARP_WRAPPING_RANDOM_WALK_MOSTLY_RIGHT,
  ARP_COUNT // Must be last
};

enum Order : uint8_t {
  ORDER_SORTED = 0,
  ORDER_ORDERED,
  ORDER_COUNT // Must be last
};

DownNoteSet notesOn;
PlayingNoteSet notesPlaying;

uint8_t arp;
uint8_t order;

int8_t arpIndex;
int8_t arpDirection;
bool holdMode;

bool prevClock;
double pitchBend;

void setup() {
  pinMode(PIN_CLOCK_IN, INPUT_PULLUP);
  pinMode(PIN_RESET_IN, INPUT_PULLUP);
  pinMode(PIN_GATE_OUT, OUTPUT);
  pinMode(PIN_TRIGGER_OUT, OUTPUT);
  pinMode(PIN_HOLD_LED, OUTPUT);
  analogWriteResolution(12);

  onSystemReset();

  MIDI.setHandleSystemReset(onSystemReset);
  MIDI.setHandleNoteOn(onNoteOn);
  MIDI.setHandleNoteOff(onNoteOff);
  MIDI.setHandlePitchBend(onPitchBend);
  MIDI.setHandleControlChange(onControlChange);
  MIDI.begin();
  Serial1.begin(31250, SERIAL_8N1_RXINV); // Input transistor inverts RX
}

void onSystemReset() {
  arp = ARP_FORWARD;
  order = ORDER_SORTED;
  arpDirection = 1;
  arpIndex = 0;
  holdMode = false;
  prevClock = false;
  pitchBend = 0.0;

  notesOn.clear();
  notesPlaying.clear();

  digitalWrite(PIN_GATE_OUT, HIGH);
  digitalWrite(PIN_TRIGGER_OUT, HIGH);
  digitalWrite(PIN_HOLD_LED, LOW);
  analogWrite(PIN_CV_OUT, 0);
}

void onNoteOn(byte channel, byte note, byte velocity) {
  // Note off can be sent as a note on with 0 velocity
  if (velocity == 0) {
    onNoteOff(channel, note, 0);
    return;
  }

  // Clear held notes if this is the first note down
  if (holdMode && notesOn.noteCount == 0) {
    notesPlaying.clear();
  }

  notesOn.addNote(note);

  NoteIndices indices = notesPlaying.addNote(note);
  int8_t index = -1;
  switch (order) {
    case ORDER_SORTED: index = indices.sorted; break;
    case ORDER_ORDERED: index = indices.ordered; break;
  }

  if (index >= 0 && arpIndex >= index) {
    // Arp was on a note after this one, move it back to that note
    arpIndex++;
  }
}

void onNoteOff(byte channel, byte note, byte velocity) {
  notesOn.removeNote(note);

  // Keep notes on if holding
  if (holdMode) {
    return;
  }

  NoteIndices indices = notesPlaying.removeNote(note);
  int8_t index = -1;
  switch (order) {
    case ORDER_SORTED: index = indices.sorted; break;
    case ORDER_ORDERED: index = indices.ordered; break;
  }

  if (index >= 0 && arpIndex > index) {
    // Arp was on a note after this one, move it back to that note
    arpIndex--;
  }
}

void onPitchBend(byte channel, int bend) {
  pitchBend = (double) bend / 8192.0 * 2.0;
}

void onControlChange(byte channel, byte control, byte value) {
  switch (control) {
    case CC_ARP_KNOB_A:
    case CC_ARP_KNOB_B: {
      float arpPct = (float) value / 128.0f;
      arp = (uint8_t) (arpPct * ARP_COUNT);
      break;
    }

    case CC_ORDER_KNOB_A:
    case CC_ORDER_KNOB_B: {
      float orderPct = (float) value / 128.0f;
      order = (uint8_t) (orderPct * ORDER_COUNT);
      break;
    }
    
    case CC_HOLD_TOGGLE_BUTTON_A:
    case CC_HOLD_TOGGLE_BUTTON_B: {
      bool pressed = (value == 127);
      holdMode ^= pressed;
      if (!holdMode) {
        // Release all notes
        notesOn.clear();
        notesPlaying.clear();
      }
      break;
    }

    case CC_CLEAR_BUTTON_A:
    case CC_CLEAR_BUTTON_B:
      notesOn.clear();
      notesPlaying.clear();
      break;
  }
}

void loop() {
  MIDI.read();

  // Inputs are inverted
  bool clock = !digitalRead(PIN_CLOCK_IN);
  bool reset = !digitalRead(PIN_RESET_IN);

  bool playing = notesPlaying.noteCount > 0;

  if (playing) {
    if (reset) {
      switch (arp) {
        case ARP_REVERSE:
          arpIndex = notesPlaying.noteCount - 1;
          break;
        default:
          arpIndex = 0;
          break;
      }
    } else if (clock && !prevClock) {
      // Clock rising
      switch (arp) {
        case ARP_FORWARD: arpForward(); break;
        case ARP_REVERSE: arpReverse(); break;
        case ARP_UP_DOWN: arpUpDown(); break;
        case ARP_UP_DOWN_REPEAT: arpUpDownRepeat(); break;
        case ARP_RANDOM_NOTE: arpRandomNote(); break;
        case ARP_RANDOM_WALK: arpRandomWalk(); break;
        case ARP_WRAPPING_RANDOM_WALK: arpWrappingRandomWalk(); break;
        case ARP_WRAPPING_RANDOM_WALK_MOSTLY_RIGHT: arpWrappingRandomWalkMostlyRight(); break;
      }
    }

    int8_t index = notesPlaying.noteCount > 1 ? arpIndex : 0;
    double note = 0.0;
    switch (order) {
      case ORDER_SORTED: note = notesPlaying.sorted[index]; break;
      case ORDER_ORDERED: note = notesPlaying.ordered[index]; break;
    }

    // Write CV output
    note = min(note, 60.0);
    note += pitchBend;
    int cvOut = (int) map(note, 0.0, 60.0, 0.0, CV_OUT_FOR_5V);
    analogWrite(PIN_CV_OUT, cvOut);
  }

  prevClock = clock;

  digitalWrite(PIN_GATE_OUT, !playing);
  digitalWrite(PIN_TRIGGER_OUT, !(playing && clock));
  digitalWrite(PIN_HOLD_LED, holdMode);
}

void arpForward() {
  arpIndex++;
  arpIndex %= notesPlaying.noteCount;
}

void arpReverse() {
  if (notesPlaying.noteCount != 0)
    arpIndex--;
  if (arpIndex < 0)
    arpIndex = notesPlaying.noteCount - 1;
}

void arpUpDown() {
  arpIndex += arpDirection;
  if (arpIndex >= notesPlaying.noteCount - 1) {
    arpIndex = notesPlaying.noteCount - 1;
    arpDirection = -1;
  }
  if (arpIndex <= 0) {
    arpIndex = 0;
    arpDirection = 1;
  }
}

void arpUpDownRepeat() {
  arpIndex += arpDirection;
  if (arpIndex >= notesPlaying.noteCount) {
    arpIndex = notesPlaying.noteCount - 1;
    arpDirection = -1;
  }
  if (arpIndex < 0) {
    arpIndex = 0;
    arpDirection = 1;
  }
}

void arpRandomNote() {
  uint8_t value = random(0, notesPlaying.noteCount - 1);
  // Don't do same one twice in a row
  if (value >= arpIndex) {
    value++;
  }
  arpIndex = value;
}

void arpRandomWalk() {
  if (random(0, 2)) {
    arpIndex++;
  } else {
    arpIndex--;
  }
  arpIndex = constrain(arpIndex, 0, notesPlaying.noteCount - 1);
}

void arpWrappingRandomWalk() {
  if (random(0, 2)) {
    arpForward();
  } else {
    arpReverse();
  }
}

void arpWrappingRandomWalkMostlyRight() {
  if (random(0, 100) < 60) {
    arpForward();
  } else {
    arpReverse();
  }
}
