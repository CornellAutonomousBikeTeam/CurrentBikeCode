#define A 0
#define B 1
#define mkpin(port, pin) ((port<<5) + pin)
#define D0  mkpin(A, 22)
#define D1  mkpin(A, 23)
#define D2  mkpin(A, 10)
#define D3  mkpin(A, 11)
#define D4  mkpin(B, 10)
#define D5  mkpin(B, 11)
#define D6  mkpin(A, 20)
#define D7  mkpin(A, 21)
#define D8  mkpin(A, 16)
#define D9  mkpin(A, 17)
#define D10 mkpin(A, 19)
#define D11 mkpin(A, 08)
#define D12 mkpin(A, 09)
#define D13 mkpin(B, 23)
#define D14 mkpin(B, 22)
#define D15 mkpin(A, 02)
#define D16 mkpin(B, 02)
#define D17 mkpin(B, 03)
#define D18 mkpin(A, 04)
#define D19 mkpin(A, 05)
#define D20 mkpin(A, 06)
#define D21 mkpin(A, 07)

#define OutSet(input) (PORT_IOBUS->Group[input>>5].OUTSET.reg = 1<<(input & 0x1f))  // Make high
#define OutClr(input) (PORT_IOBUS->Group[input>>5].OUTCLR.reg = 1<<(input & 0x1f))  // Make low
#define OutTgl(input) (PORT_IOBUS->Group[input>>5].OUTTGL.reg = 1<<(input & 0x1f))  // toggle value

#define Input(input)  (PORT_IOBUS->Group[input>>5].DIRSET.reg = 1<<(input & 0x1f))  // Make input
#define Output(input) (PORT_IOBUS->Group[input>>5].DIRCLR.reg = 1<<(input & 0x1f))  // Make output
// #define DIRTgl(input) (PORT_IOBUS->Group[input>>5].DIRTGL.reg = 1<<(input & 0x1f))  // toggle in/out

#define Read(input)  ((PORT_IOBUS->Group[input>>5].IN.reg) >> (input & 0x1f) & 1)   // Read pin


void setup() {
  Serial.begin(1000000);

  Input(D0);
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