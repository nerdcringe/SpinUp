/*
    THIS IS A FILE FOR DECLARING ALL YOUR FUNCTIONS. YOU NEED TO DECLARE THEM HERE SO YOU CAN USE THEM IN YOUR AUTONOMOUS AND OTHER PLACES
    You NEED to have this file that has all the functions you plan on REFERENCING IN OTHER FILES declared here so you can actually refernce them in other files like autonomous.cpp 
      You dont need to do anything more than declare the function signature you'll actually implement these functions in functions.cpp
  NOTE: Essentially you only need to put functions in here that you plan on calling outside the functions.cpp file If you want to create functions that you then use in your 
    main.cpp then you should list it below
  NOTE: These functions are functions you plan on referencing in other files
        Recommended Approach:
          I would recommend that you put all function signatures in here and then in functions.cpp you can actually implement those functions.
*/


// BASIC MOVEMENT
void goDistance(double distance, double speed);
void goTime(double numSeconds, double speed);
void turnDistanceL(double distance, double speed);
void turnDistanceR(double distance, double speed);
void curveL(double distance, double Lspeed, double Rspeed);
void curveR(double distance, double Lspeed, double Rspeed);

// INERTIAL SENSOR MOVEMENT
void driveStraight(double distance, double speed); // maintain heading
void turnAngleR(double degrees, double speed);
void turnAngleL(double degrees, double speed);
