float PI = 3.14159;
float C_1 = 1;
float K_1 = 1;
float TAU = 1;

/* ======== USER INTERFACE TEMPLATE ============= */
/*  Insert below the code for your scaling, saturation block, and controllers.*/

/*  Variables may be declared on the box border, as shown for the input 
"Tms" and the output "BallPosn". Variables can also be declared inline as was done for "Temp1". */

float eGearAng;

/*  Shift registers permit previous values of variables to be saved.
The output variable "e" is wired to a shift register input on the For Loop border.
The inputs "e1" and "e2"are wired  to the corresponding shift register outputs.
"e1" holds the value of "e" from the previous iteration and "e2" holds the value of "e1" from  the previous iteration. */

/* Place your sensor SCALING here */
/* NO scaling is provided for the demo */
BallPosn = posV ;  /* V to V */
// Gear angle in radians
ServoAng = (angV - 4.78) / (3.65 - 4.78) * (PI/2) - (PI/4); /* V to V */
/* SCALING end */

if (Loop  < 3) {   /* all shift registers cleared after 3rd iteration; this statement initializes the shift registers */
  u = e = ThRef = posV= angV =ServoAng= BallPosn= 0;
} else {
  if (Manual) {   /*manual motor voltage control*/
    u = MotV;
  } else {    /*control algorithm*/
  
    /* CAUTION: DO NOT load the output of a nonlinear block (e.g., saturator, offset) into a SHIFT REGISTER, 
    to avoid introducing a nonlinearity into your controller loop. Create separate variables to hold nonlinear values.*/

    /* Place your outer loop BALL POSITION CONTROLLER below */
    BallPosn = 0; // REMOVE this line when the ball is being used on the beam

    /* Place your gear angle SATURATOR below */

    /* Place your inner loop GEAR ANGLE CONTROLLER below */
    u = ServoAng - ref;
  }
}
    
/* ThRef, ThRef1, e, e1 are present, but not used in this demo.  
However, they will be necessary (at a minimum) when the controllers will be implemented. */