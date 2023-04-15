#define FIXED_POINT_SHIFT 10  // Shift value for fixed-point arithmetic (must be a power of 2)
#define FIXED_POINT_ONE (1 << FIXED_POINT_SHIFT)  // Constant representing the value 1 in fixed-point arithmetic

int divide_fixed_point(int dividend, int divisor) {
  long result = ((long)dividend << FIXED_POINT_SHIFT) / divisor;  // Multiply dividend by 2^SHIFT before division
  return (int)result;  // Convert result back to fixed-point format with SHIFT decimal places
}

void setup() {
  Serial.begin(9600);  // Initialize serial communication for output
}

void loop() {
  int dividend = 100;
  int divisor = 3;
  
  int quotient = divide_fixed_point(dividend * FIXED_POINT_ONE, divisor);  // Multiply dividend by 2^SHIFT before division
  Serial.print(dividend);
  Serial.print(" / ");
  Serial.print(divisor);
  Serial.print(" = ");
  Serial.println((float)quotient / FIXED_POINT_ONE);  // Convert quotient back to floating-point format with SHIFT decimal places
  delay(1000);
}

/*
The FIXED_POINT_SHIFT constant determines the number of decimal places used in fixed-point arithmetic. In this example, I used 10 decimal places, so FIXED_POINT_SHIFT 
is set to 10.

The FIXED_POINT_ONE constant represents the value 1 in fixed-point arithmetic. This is calculated by shifting a 1 bit left by FIXED_POINT_SHIFT places.

The divide_fixed_point() function takes two integers as inputs: dividend and divisor. It multiplies dividend by 2^SHIFT before performing division using long integer arithmetic. 
The result is then converted back to fixed-point format with SHIFT decimal places and returned as an integer.

In the loop() function, we set dividend and divisor to arbitrary values (100 and 3, respectively). I then called divide_fixed_point() with dividend * FIXED_POINT_ONE as the
numerator, since I wanted to perform division on fixed-point numbers. I printed the result as a floating-point number with SHIFT decimal places by dividing the quotient by 
FIXED_POINT_ONE before printing it to the serial monitor.

I added a delay of 1 second using the delay() function to slow down for serial monitor output(adjustable).
*/ 