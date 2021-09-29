float C_1 = 15;
float K_1 = 1;
float TAU = 1;
float PI = 3.14159;
float V_neg_45 = 4.765;
float V_pos_45 = 3.66;

/*  Insert below the code for your scaling, saturation block, and
   controllers.*/
/*  Shift registers permit previous values of variables to be saved.
The output variable "e" is wired to a shift register input on the For Loop
border.
The inputs "e1" and "e2"are wired  to the corresponding shift register
outputs.
 "e1" holds the value of "e" from the previous iteration and "e2" holds the
value of "e1" from  the previous iteration. */

/* Place your sensor SCALING here */
/* NO scaling is provided for the demo */

BallPosn = posV; /* V to V */

// Gear angle in radians
ServoAng = (angV - V_neg_45) / (V_pos_45 - V_neg_45) * (PI / 2) -
           (PI / 4); /* V to V */

/**************** SCALING END ****************/

if (Loop < 3) {
  /* all shift registers cleared after 3rd iteration; this
                    statement initializes the shift registers */
  u = e = ThRef = posV = angV = ServoAng = BallPosn = 0;
} else {
  if (Manual) {/*manual motor voltage control*/
    u = MotV;
  } else {/*control algorithm*/
    ThRef = ref;

    /* CAUTION: DO NOT load the output of a nonlinear block (e.g., saturator,
    offset) into a SHIFT REGISTER,
    to avoid introducing a nonlinearity into your controller loop. Create
    separate variables to hold nonlinear values.*/

    /* Place your outer loop BALL POSITION CONTROLLER below */
    BallPosn = 0; // REMOVE this line when the ball is being used on the beam

    /* Place your gear angle SATURATOR below */
    // if (ThRef > PI / 4) {
    //   ThRef = PI / 4;
    // } else if (ThRef < -PI / 4) {
    //   ThRef = -PI / 4;
    // }

    /* Place your inner loop GEAR ANGLE CONTROLLER below */
    u = -C_1 * (ThRef - ServoAng);
  }
}

/* ThRef, ThRef1, e, e1 are present, but not used in this demo.
However, they will be necessary (at a minimum) when the controllers will be
implemented. */
