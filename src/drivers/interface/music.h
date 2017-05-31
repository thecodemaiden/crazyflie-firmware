#ifndef __CF2_MUSIC__
#define __CF2_MUSIC__
/**
 * Credit to http://tny.cz/e525c1b2 for supplying the tones
 */
#define OFF 0
#define C0 16
#define Db0 17
#define D0  18
#define Eb0 19
#define E0  20
#define F0  21
#define Gb0 23
#define G0  24
#define Ab0 25
#define A0 27
#define Bb0 29
#define B0  30
#define C1  32
#define Db1 34
#define D1  36
#define Eb1 38
#define E1  41
#define F1  43
#define Gb1 46
#define G1  49
#define Ab1 51
#define A1 55
#define Bb1 58
#define B1  61
#define C2  65
#define Db2 69
#define D2  73
#define Eb2 77
#define E2  82
#define F2  87
#define Gb2 92
#define G2  98
#define Ab2 103
#define A2 110
#define Bb2 116
#define B2  123
#define C3  130
#define Db3 138
#define D3  146
#define Eb3 155
#define E3  164
#define F3  174
#define Gb3 185
#define G3  196
#define Ab3 207
#define A3 220
#define Bb3 233
#define B3  246
#define C4  261
#define Db4 277
#define D4  293
#define Eb4 311
#define E4  329
#define F4  349
#define Gb4 369
#define G4  392
#define Ab4 415
#define A4 440
#define Bb4 466
#define B4  493
#define C5  523
#define Db5 554
#define D5  587
#define Eb5 622
#define E5  659
#define F5  698
#define Gb5 739
#define G5  783
#define Ab5 830
#define A5 880
#define Bb5 932
#define B5  987
#define C6  1046
#define Db6 1108
#define D6  1174
#define Eb6 1244
#define E6  1318
#define F6  1396
#define Gb6 1479
#define G6  1567
#define Ab6 1661
#define A6 1760
#define Bb6 1864
#define B6  1975
#define C7  2093
#define Db7 2217
#define D7  2349
#define Eb7 2489
#define E7  2637
#define F7  2793
#define Gb7 2959
#define G7  3135
#define Ab7 3322
#define A7 3520
#define Bb7 3729
#define B7  3951
#define C8  4186
#define Db8 4434
#define D8  4698
#define Eb8 4978
/* Duration of notes */
#define W  1  // 1/1
#define H  2  // 1/2
#define Q  4  // 1/4
#define E  8  // 1/8
#define S  16 // 1/16
#define ES 6
/* End markers */
#define STOP {0xFE, 0}
#define REPEAT {0xFF, 0}

#define MAX_NOTE_LENGTH 80

typedef const struct {
  uint16_t tone;
  uint16_t duration;
} Note;

typedef const struct {
  uint32_t bpm;
  uint32_t delay;
  Note notes[MAX_NOTE_LENGTH];
} Melody;
#endif
