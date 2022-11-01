#define D0  ((0<<5) + 22)
#define D1  ((0<<5) + 23)
#define D2  ((0<<5) + 10)
#define D3  ((0<<5) + 11)
#define D4  ((1<<5) + 10)
#define D5  ((1<<5) + 11)
#define D6  ((0<<5) + 20)
#define D7  ((0<<5) + 21)
#define D8  ((0<<5) + 16)
#define D9  ((0<<5) + 17)
#define D10 ((0<<5) + 19)
#define D11 ((0<<5) + 08)
#define D12 ((0<<5) + 09)
#define D13 ((1<<5) + 23)
#define D14 ((1<<5) + 22)

#define OutSet(input) (PORT_IOBUS->Group[input>>5].OUTSET.reg = 1<<(input & 0x1f))  // Make high
#define OutClr(input) (PORT_IOBUS->Group[input>>5].OUTCLR.reg = 1<<(input & 0x1f))  // Make low
#define OutTgl(input) (PORT_IOBUS->Group[input>>5].OUTTGL.reg = 1<<(input & 0x1f))  // toggle value

#define DIRSet(input) (PORT_IOBUS->Group[input>>5].DIRSET.reg = 1<<(input & 0x1f))  // Make input
#define DIRClr(input) (PORT_IOBUS->Group[input>>5].DIRCLR.reg = 1<<(input & 0x1f))  // Make output
#define DIRTgl(input) (PORT_IOBUS->Group[input>>5].DIRTGL.reg = 1<<(input & 0x1f))  // toggle in/out

#define Read(input) ((PORT_IOBUS->Group[input>>5].IN.reg) >> (input & 0x1f) & 1)    // Read pin


void setup() {
  Serial.begin(1000000);

  DIRClr(D0);
}

void loop() {
  int v;
  unsigned long start;

  start = micros();
  for (unsigned int i = 0; i < 1000000; i++){
    v = Read(D0);
  }
  Serial.print("fast: ");
  Serial.print  (1.0 / ((micros() - start) / 1000000.0));

  start = micros();
  for (unsigned int i = 0; i < 1000000; i++){
    v = digitalRead(0);
  }
  Serial.print(", slow: ");
  Serial.println(1.0 / ((micros() - start) / 1000000.0));
}