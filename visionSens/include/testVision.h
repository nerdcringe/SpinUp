/*vex-vision-config:begin*/
#include "vex.h"
vex::vision::signature REDGOAL = vex::vision::signature (1, 7307, 8615, 7960, -591, -147, -368, 5, 0);
vex::vision::signature BLUEGOAL = vex::vision::signature (2, -4133, -3289, -3712, 8065, 10013, 9040, 5, 0);
vex::vision::signature SIG_3 = vex::vision::signature (3, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_4 = vex::vision::signature (4, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_5 = vex::vision::signature (5, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_6 = vex::vision::signature (6, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision::signature SIG_7 = vex::vision::signature (7, 0, 0, 0, 0, 0, 0, 2.5, 0);
vex::vision VisionSensor = vex::vision (vex::PORT14, 50, REDGOAL, BLUEGOAL, SIG_3, SIG_4, SIG_5, SIG_6, SIG_7);
/*vex-vision-config:end*/