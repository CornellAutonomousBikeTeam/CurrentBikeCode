

void setup() {
  // put your setup code here, to run once:
//copy the float to fixed code/ fixed to float code
//before set up declare two values (float vvalues) = 16.2 , 8.995
//nothin in loop method
//convert 16.2 to fixed, print it, covert the fixed value back to float and find the difference
  float bob = 16.2; 
  float y = 8.995;
  Serial.begin(9600);


  while(!Serial);

  
  
  //float startMicros = micros();

  //Serial.println(startMicros);
  //int16_t test_fixed = floattofixed(y);
  //Serial.println(test_fixed);
  //float currentMicros= micros();
  //Serial.println(currentMicros);
  //Serial.print("floattofixed time diff:"), Serial.println(currentMicros - startMicros);
/*
  startMicros = micros();
  Serial.println(startMicros);
  float test_float = fixedtofloat(test_fixed);
  Serial.println(test_float);
  currentMicros= micros();
  Serial.println(currentMicros);
  Serial.print("fixedtofloat time diff:"), Serial.println(currentMicros - startMicros);*/

  //int a = 66;
  //int b = 55;

  float c = -5.8976;
  float d = -7.9874;
/*
  float startMicros = micros();
  int opp = a * b * a * b;
  float currentMicros = micros();
  Serial.println(opp);
  Serial.println((currentMicros - startMicros)*1000);

  startMicros = micros();
  float opp_float = c * d * c * d;
  currentMicros = micros();
  Serial.println(opp_float);
  Serial.println((currentMicros - startMicros)*1000);
 */

 //  int16_t converted_d = floattofixed(d);
 //  int16_t converted_c = floattofixed(c);

 //  float fin_convert = fixedtofloat(converted_d - converted_c);
 //  Serial.println(fin_convert);

 float w = fixedtofloat(multiply_fixed(floattofixed(d),  floattofixed(c)));
 Serial.println(floattofixed (d));
 Serial.println(floattofixed (c));
 Serial.println("Fixed point multiplication");
 Serial.println(w);
 Serial.println("Float multiplication");
 Serial.println(d * c);

 for (float i = 0; i < 43; i+=0.5){ 
  float x = fixedtofloat(multiply_fixed(floattofixed(i),  floattofixed(i)));
   if (x < 0){
      Serial.println(i);  
      Serial.println(x);
      break;
   }
   
 }
}


int16_t floattofixed(float x)
{
  //int y = (int)x;
  //return y*16;
  return (int16_t)(x * 16);
}    

float fixedtofloat(int16_t x)
{
  //float y = (float)x;
  //return y
  return (x)/16.0;
} 

int16_t multiply_fixed(int16_t d, int16_t c)
{ 
  return (int16_t) ((((int32_t) d) * ((int32_t) c)) >> 4);
}

  



void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println("hello loop");
 //delay(5000);
}
//copy the float to fixed code/ fixed to float code
//before set up declare two values (float vvalues) = 16.2 , 8.995
//nothin in loop method
//convert 16.2 to fixed, print it, covert the fixed value back to float and find the difference