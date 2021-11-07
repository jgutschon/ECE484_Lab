// Controller Coefficients Constants. Controller is given by:
// C_1 = (A_0 * z + A_1)/(B_0 * z + B_1)
float A_0 = -1.204931478672254;
float A_1 = 1.032681063701449;
float B_0 = 1.000000000000000;
float B_1 = -0.960962180870512;

// Measurement Scaling Constants
float V_neg_45 = 4.765;
float V_pos_45 = 3.650;
float BALL_LEFT_V = 3.290;
float BALL_RIGHT_V = 7.320;
float BEAM_LENGTH = 0.417;

// Measured Constants
float K_2 = 0.0573;

// Constraint Constants
float THETA_MAX = 0.7;
float THETA_MIN = -0.7;

// Ball Position in m
BallPosn = BEAM_LENGTH / (BALL_RIGHT_V - BALL_LEFT_V) * (posV - BALL_LEFT_V);
// Gear Angle in RAD
ServoAng = (angV - V_neg_45) / (V_pos_45 - V_neg_45) * (pi / 2) - (pi / 4);

if (Loop < 3) {
  // All shift registers cleared after 3rd iteration. This statement
  // initializes the shift registers
  u = e = ThRef = posV = angV = ServoAng = BallPosn = 0;
} else {
  if (Manual) {
    // MANUAL MOTOR VOLTAGE CONTROL
    u = MotV;
  } else {
    // CONTROL ALGORITHM
    ThRef = ref;
    float phi = K_2 * ThRef;

    /* CAUTION: DO NOT load the output of a nonlinear block (e.g., saturator,
    offset) into a SHIFT REGISTER,
    to avoid introducing a nonlinearity into your controller loop. Create
    separate variables to hold nonlinear values.*/

    /* Place your outer loop BALL POSITION CONTROLLER below */

    // GEAR ANGLE SATURATOR
    if (ThRef > THETA_MAX) {
      ThRef = THETA_MAX;
    } else if (ThRef < THETA_MIN) {
      ThRef = THETA_MIN;
    }

    // INNER LOOP GEAR ANGLE CONTROLLER
    e = ThRef - ServoAng;
    u = 1 / B_0 * (-B_1 * u1 + A_0 * e + A_1 * e1);
  }
}
