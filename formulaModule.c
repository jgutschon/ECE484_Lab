// C_1 = ( -1.204931478672254 * z + 1.032681063701449) / (z -0.960962180870512)
float A_0 = 1.871384310801432;
float A_1 = -2.090858526061850;
float B_0 = 1.0;
float B_1 = -0.945709135734579;
float V_neg_45 = 4.765;
float V_pos_45 = 3.650;
float BALL_LEFT_V = 3.255;
float BALL_RIGHT_V = 7.315;
float BEAM_LENGTH = 417;
float LEVER_END_RADIUS = 25.4;

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

// Ball position in mm
BallPosn = BEAM_LENGTH / (BALL_RIGHT_V - BALL_LEFT_V) * (posV - BALL_LEFT_V); 

// Gear angle in radians
ServoAng = (angV - V_neg_45) / (V_pos_45 - V_neg_45) * (pi / 2) -
           (pi / 4);

/**************** SCALING END ****************/

if (Loop < 3) {
  /* all shift registers cleared after 3rd iteration; this
                    statement initializes the shift registers */
  u = e = ThRef = posV = angV = ServoAng = BallPosn = 0;
} else {
  if (Manual) {/*manual motor voltage control*/
    u = MotV;
  } else { /*control algorithm*/
    ThRef = ref;
    
    float phi = asin(LEVER_END_RADIUS / BEAM_LENGTH * sin(ThRef));

    /* CAUTION: DO NOT load the output of a nonlinear block (e.g., saturator,
    offset) into a SHIFT REGISTER,
    to avoid introducing a nonlinearity into your controller loop. Create
    separate variables to hold nonlinear values.*/

    /* Place your outer loop BALL POSITION CONTROLLER below */
    // BallPosn = phi; // REMOVE this line when the ball is being used on the beam

    /* Place your gear angle SATURATOR below */
    if (ThRef > 0.7) {
      ThRef = 0.7;
    } else if (ThRef < -0.7) {
      ThRef = -0.7;
    }

    e = ThRef - ServoAng;

    /* Place your inner loop GEAR ANGLE CONTROLLER below */
    u = 1 / B_0 * (-B_1 * u1 + A_0 * e + A_1 * e1);
  }
}

/* ThRef, ThRef1, e, e1 are present, but not used in this demo.
However, they will be necessary (at a minimum) when the controllers will be
implemented. */
