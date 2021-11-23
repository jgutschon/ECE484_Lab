// Controller Coefficients Constants. Controller is given by:
// C_1 = (A1_0 * z + A1_1)/(B1_0 * z + B1_1)
float A1_0 = -1.204931478672254;
float A1_1 = 1.032681063701449;
float B1_0 = 1.000000000000000;
float B1_1 = -0.960962180870512;

// C_LEAD_2 = (A_LEAD_0 * z + A_LEAD_1)/ B_LEAD_0 * z + B_LEAD_1)
float A_LEAD_0 = -7.000000000000000;
float A_LEAD_1 = 6.997553059949510;
float B_LEAD_0 = 1.000000000000000;
float B_LEAD_1 = -0.997503122397460;

// PART C CONTROLLER
// C_2 = (A2_0 * z^3 + A2_1 * z^2 + A2_2 * z + A2_3)
//       / (B2_0 * z^3 + B2_1 * z^2 + B2_2 * z + B2_3)
// float A2_0 = -0.295404446074768;
// float A2_1 = 0.295232270026272;
// float A2_2 = 0.295404414174432;
// float A2_3 = -0.295232301926608;
// float B2_0 = 1.000000000000000;
// float B2_1 = -2.979638314061477;
// float B2_2 = 2.959401373706974;
// float B2_3 = -0.979763059645498;

// PART E CONTROLLER
// float A2_0 = 0.000000000000000;
// float A2_1 = -0.110664622404449;
// float A2_2 = 0.221261353288049;
// float A2_3 = -0.110596745599437;
// float B2_0 = 1.000000000000000;
// float B2_1 = -2.991121832548782;
// float B2_2 = 2.982272333037067;
// float B2_3 = -0.991150500488285;

// Measurement Scaling Constants
float V_neg_45 = 4.765;
float V_pos_45 = 3.650;
float BALL_LEFT_V = 3.290;
float BALL_RIGHT_V = 7.320;
float RADIUS_BALL = 0.0127;
float BEAM_LENGTH = 0.417 - 2 * RADIUS_BALL;

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
  u = e = ePos = ThRef = posV = angV = ServoAng = BallPosn = 0;
} else {
  if (Manual) {
    // MANUAL MOTOR VOLTAGE CONTROL
    u = MotV;
  } else {
    // CONTROL ALGORITHM

    /* CAUTION: DO NOT load the output of a nonlinear block (e.g., saturator,
    offset) into a SHIFT REGISTER,
    to avoid introducing a nonlinearity into your controller loop. Create
    separate variables to hold nonlinear values.*/

    /* Place your outer loop BALL POSITION CONTROLLER below */
    ePos = ref - BallPosn;

    // LEAD CONTROLLER METHOD
    // ThRef = 1 / B_LEAD_0 * (-B_LEAD_1 * ThRef1 + A_LEAD_0 * ePos + A_LEAD_1 *
    // ePos1);
    ThRef = 1 / B2_0 *
            (-B2_1 * ThRef1 - B2_2 * ThRef2 - B2_3 * ThRef3 + A2_0 * ePos +
             A2_1 * ePos1 + A2_2 * ePos2 + A2_3 * ePos3);

    float ThRefSaturated = ThRef;
    // GEAR ANGLE SATURATOR
    if (ThRefSaturated > THETA_MAX) {
      ThRefSaturated = THETA_MAX;
    } else if (ThRefSaturated < THETA_MIN) {
      ThRefSaturated = THETA_MIN;
    }

    // INNER LOOP GEAR ANGLE CONTROLLER
    e = ThRefSaturated - ServoAng;
    u = 1 / B1_0 * (-B1_1 * u1 + A1_0 * e + A1_1 * e1);
  }
}
