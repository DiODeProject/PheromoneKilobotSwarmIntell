/*  Authors: Mohamed S. Talamali (mstalamali1@sheffield.ac.uk) and Andreagiovanni Reina (a.reina@sheffield.ac.uk)
 *
 *  If you use this code for scientific experiment, please cite: M.S. Talamali et al. Swarm Intelligence (2019)
 *
 *  Copyright University of Sheffield, 2019
 */

#include "kilolib.h"
#include "stdlib.h"
#include "math.h"

#define STRAIGHT_TIME 320
#define TURN_RAND_TIME 128
#define MIN_TIME_BEFORE_SPIN 20
#define DEPOSIT_PHEROMONE_STEP 128
#define ticks_per_rotation_degree 0.7

// Enum for boolean flags
typedef enum {
    false = 0,// !< false
    true = 1, // !< true
} bool;

// Enum for different motion types
typedef enum {
    STOP = 0,
    FORWARD,
    TURN_LEFT,
    TURN_RIGHT,
} motion_t;

// Enum for different behaviour types
typedef enum {
    SEARCH_FOOD = 0,
    GO_HOME,
    FOLLOW_PH,
    TURN_KILOBOT,
    AVOID_OBSTACLE,
} behave_t;

motion_t current_motion;
message_t message;
behave_t behave;
behave_t behave_previous;

uint32_t last_pheromone_deposit_ticks;
uint32_t last_motion_update = 0;
uint32_t last_motion_ticks;
uint32_t timesToTurn = 0;
uint32_t countLost = 0;

uint8_t backup_LED = 0;
/* Variable to store LED colour before runtime identification: 0 OFF, 1 RED, 2 GREEN, 3 BLUE */

bool runtime_identification = false; /* when true the runtime identification is ongoing */
bool turn180completed = false;
bool updated_home_angle;
bool atPheromone;
bool atHome;
bool atFood;
bool atObstacle; // at the border of an obstacle or the arena
bool turning;
bool goforward;

bool AvoidingObstacle=false;

float best_pheromone_angle = 0;

bool FL; // obstacle sensor in front left antenae
bool FR; // obstacle sensor in front right antenae
bool L; // obstacle sensor in left antenae
bool R; // obstacle sensor in right antenae


int angleToHome; /* received as "round(a/45)" and computed to a value between 0 and 360 */
double foodQuality;
double v_max=0.0;
double alpha=0.0;

uint32_t travelTime=0; // in ticks
double travelTime_max=100; // in seconds

// Motion Control Function
void set_motion(motion_t new_motion){
    /* Only takes an an action if the motion is being changed */
    if (current_motion != new_motion){
        current_motion = new_motion;

        if (current_motion == STOP){
            set_motors(0, 0);
        }
        else if (current_motion == FORWARD){
            spinup_motors();
            set_motors(kilo_straight_left, kilo_straight_right);
        }
        else if (current_motion == TURN_LEFT){
            spinup_motors();
            set_motors(kilo_turn_left, 0);
        }
        else if (current_motion == TURN_RIGHT){
            spinup_motors();
            set_motors(0, kilo_turn_right);
        }
    }
}

// Initilization Function
void setup(){
    srand(rand_hard());
    last_motion_ticks = rand() % STRAIGHT_TIME;
    /*
    printf("FOR DEBUG, reset last_motion_ticks\n");
    srand(123456);
    last_motion_ticks = 0;
    behave = GO_HOME;
    */
    behave = SEARCH_FOOD;
    atHome = false;
    atFood = false;
    atObstacle = false;
    atPheromone = false;
    turning = false;
    goforward = false;
    angleToHome = 0;
    current_motion = FORWARD;
    updated_home_angle = false;
    foodQuality = 0;
    FL = false;
    FR = false;
    L = false;
    R = false;
}

// Random Motion Function
void set_random_motion(){
    switch( current_motion ) {
    case TURN_LEFT:
    case TURN_RIGHT:
        if( kilo_ticks > last_motion_ticks + timesToTurn ) {
            /* Move Forward */
            last_motion_ticks = kilo_ticks;
            set_motion(FORWARD);
        }
        break;

    case FORWARD:
        if( kilo_ticks > last_motion_ticks + STRAIGHT_TIME ) {
            /* Random Turn */
            last_motion_ticks = kilo_ticks;

            if( rand() % 2 ) { set_motion(TURN_LEFT); }
            else { set_motion(TURN_RIGHT); }

            timesToTurn = (rand() % TURN_RAND_TIME) + 1;
        }
        break;

    case STOP:
    default:
        set_motion(STOP);
    }
}

// Pheromone Quality Check Function
void checkPheroQuality(){
    /* Probability of depositing pheromone */
    double probability = exp(alpha*(foodQuality-v_max)/foodQuality)*100;
    uint8_t randNum = rand() % 100;

    /* Print pheromone */
    if(probability > randNum){
        if (!runtime_identification) { set_color(RGB(0, 0, 3)); }
        backup_LED = 3; // 0 OFF, 1 RED, 2 GREEN, 3 BLUE
    }
    /* Do not print pheromone */
    else{
        if (!runtime_identification) { set_color(RGB(0, 0, 0)); }
        backup_LED = 0; // 0 OFF, 1 RED, 2 GREEN, 3 BLUE
    }
}

// Return Home Function
void returnHome(){
    if(kilo_ticks > last_pheromone_deposit_ticks + DEPOSIT_PHEROMONE_STEP){
        /* Pheromone Deposit Rate */
        checkPheroQuality();
        last_pheromone_deposit_ticks = kilo_ticks;
    }
    if(current_motion != FORWARD && kilo_ticks > last_motion_ticks + timesToTurn){
        /* Ignore messages if turning */
        timesToTurn = 0;
        set_motion(FORWARD);
    }
    else{
        /* If robot is facing home with a margin of +-30 degrees, move straight forward
           Otherwise, turn right for angles > 180 deg, and left for angles < 180 deg */
        if (updated_home_angle && kilo_ticks > last_motion_ticks + MIN_TIME_BEFORE_SPIN){
            updated_home_angle = false;
            if ( angleToHome < 30 || angleToHome > 330 ){
                set_motion(FORWARD);
            }
            else if (angleToHome == 180){
                if (rand() % 2) { set_motion(TURN_LEFT); }
                else { set_motion(TURN_RIGHT); }
            }
            else if (angleToHome < 180){
                set_motion(TURN_LEFT);
            }
            else{
                angleToHome = 360 - angleToHome;
                set_motion(TURN_RIGHT);
            }
            timesToTurn = (uint32_t)(angleToHome * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;
        }
    }
}

// Avoid Obstacle Function


// Find Minimum Function
float min(float a, float b){
    if (a < b) return a;
    else return b;
}

// Approximately Equal Function
bool almostEq(float a, float b){
    if( fabs(a-b) < 0.001) return true;
    else return false;
}

// Message Parsing Function
void message_rx(message_t *m, distance_measurement_t *d){
    bool new_message = false;
    int sa_payload = 0;
    uint8_t sa_type = 0;

    if (m->type == 0) {
        // unpack message
        int id1 = m->data[0] << 2 | (m->data[1] >> 6);
        int id2 = m->data[3] << 2 | (m->data[4] >> 6);
        int id3 = m->data[6] << 2 | (m->data[7] >> 6);
        if (id1 == kilo_uid) {
            // unpack type
            sa_type = m->data[1] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((m->data[1]&0b11) << 8) | (m->data[2]);
            new_message = true;
        }
        if (id2 == kilo_uid) {
            // unpack type
            sa_type = m->data[4] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((m->data[4]&0b11)  << 8) | (m->data[5]);
            new_message = true;
        }
        if (id3 == kilo_uid) {
            // unpack type
            sa_type = m->data[7] >> 2 & 0x0F;
            // unpack payload
            sa_payload = ((m->data[7]&0b11)  << 8) | (m->data[8]);
            new_message = true;
        }
    }
    else if (m->type == 120) { // initial identification
        int id = (m->data[0] << 8) | m->data[1];
        if (id == kilo_uid) {
            set_color(RGB(0,0,3));
        } else {
            set_color(RGB(3,0,0));
        }
    }
    else if (m->type == 119) { // runtime identification
        int id = (m->data[0] << 8) | m->data[1];
        if (id >= 0){ // runtime identification ongoing
            runtime_identification = true;
            if (id == kilo_uid) {
                set_color(RGB(0,0,3));
            } else {
                set_color(RGB(3,0,0));
            }
        }
        else { // runtime identification ended
            runtime_identification = false;
            // restore the LED colour (0 OFF, 1 RED, 2 GREEN, 3 BLUE)
            if (backup_LED == 0)
                set_color(RGB(0,0,0));
            else if (backup_LED == 1)
                set_color(RGB(3,0,0));
            else if (backup_LED == 2)
                set_color(RGB(0,3,0));
            else if (backup_LED == 3)
                set_color(RGB(0,0,3));
        }
    }

    if (new_message){

        float MaxDistToHome = 0;
        int angleRx = 0;
        int ObsRx = 0;
        float aux_phAngle = 0;
        angleToHome = sa_type;
        angleToHome = angleToHome*45;
        updated_home_angle = true;

        // Storing information about pheromone
        atPheromone = false;
        best_pheromone_angle = -1;

        //Check if it is at food, at home, or at obstacle
        atFood = false;
        atHome = false;
        atObstacle = false;
        FL = false;
        FR = false;
        L = false;
        R = false;

        // Check if kilobot is at the border of an obstacle or the arena
        if (sa_payload >= 768){
            sa_payload = sa_payload - 768;
            atObstacle = true;
            //Save the bits that represent obstacle zones
            ObsRx = (sa_payload >> 4) & 0b0000001111;
            //            ObsRx = (sa_payload >> 4);

            //Check which zones have obstacles present
            if (ObsRx >= 8){ //obstacle present in right antenae
                ObsRx = ObsRx - 8;
                R = true;
            }
            if (ObsRx >= 4){ //obstacle present in front right antenae
                ObsRx = ObsRx - 4;
                FR = true;
            }

            if (ObsRx >= 2){ //obstacle present in front left antenae
                ObsRx = ObsRx - 2;
                FL = true;
            }
            if (ObsRx >= 1){ //obstacle present in left antenae
                ObsRx = ObsRx - 1;
                L = true;
            }
            if(!AvoidingObstacle){
                behave_previous=behave;
                behave=AVOID_OBSTACLE;
            }
        }
        else if (sa_payload >= 512){
            sa_payload = sa_payload - 512;
            //Save the quality of the food source
            foodQuality = sa_payload >> 4;
            if(foodQuality>v_max){
                v_max=foodQuality;
            }
            atFood = true;
        }
        //If not at food, maybe at home
        else if (sa_payload >= 256){
            sa_payload = sa_payload - 256;
            if( (sa_payload >> 4) > v_max ){
                v_max= (double) (sa_payload >> 4);
            }
            atHome = true;
        }
        else if(!AvoidingObstacle){ // If the agent is not at the border of an obstacle the information trasmitted relates to the pherozones.
            //Save the bits that represent pherozones
            angleRx = (sa_payload >> 4) & 0b0000001111;

            //Find the angle farther from home
            if (angleRx >= 8){
                atPheromone = true;
                aux_phAngle = 67.5;
                angleRx = angleRx - 8;
                float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
                if( diff > MaxDistToHome){
                    MaxDistToHome = diff;
                    best_pheromone_angle = aux_phAngle;
                }
            }
            if (angleRx >= 4){
                atPheromone = true;
                aux_phAngle = 22.5;
                angleRx = angleRx - 4;
                float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
                if ( almostEq(diff,MaxDistToHome) ){
                    if (rand()%2) best_pheromone_angle = aux_phAngle;
                } else if( diff > MaxDistToHome){
                    MaxDistToHome = diff;
                    best_pheromone_angle = aux_phAngle;
                }
            }
            if (angleRx >= 2){
                atPheromone = true;
                aux_phAngle = 360-22.5;
                angleRx = angleRx - 2;
                float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
                if ( almostEq(diff,MaxDistToHome) ){
                    if (rand()%2) best_pheromone_angle = aux_phAngle;
                } else if( diff > MaxDistToHome){
                    MaxDistToHome = diff;
                    best_pheromone_angle = aux_phAngle;
                }
            }
            if (angleRx >= 1){
                atPheromone = true;
                aux_phAngle = 360-67.5;
                angleRx = angleRx - 1;
                float diff = min(360 - fabs(aux_phAngle - angleToHome), fabs(aux_phAngle - angleToHome));
                if ( almostEq(diff,MaxDistToHome) ){
                    if (rand()%2) best_pheromone_angle = aux_phAngle;
                } else if( diff > MaxDistToHome){
                    MaxDistToHome = diff;
                    best_pheromone_angle = aux_phAngle;
                }
            }
        }
    }
}

void avoidObstacle(){

    if(!AvoidingObstacle){

        // 45 DEGREE LEFT TURN SCENARIOS
        if(L && !FL &&  !FR && !R) {

            timesToTurn = (uint32_t)(0 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            set_motion(TURN_LEFT); // turn right until obstacle not detected
            AvoidingObstacle=true;
        }

        // 90 DEGREE LEFT TURN SCENARIOS
        else if ( (L && FL &&  !FR && !R) ||
                  (!L && FL &&  !FR && !R) ) {

            timesToTurn = (uint32_t)(45 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            set_motion(TURN_LEFT); // turn right until obstacle not detected
            AvoidingObstacle=true;
        }


        // 135 DEGREE LEFT TURN SCENARIOS
        else if( (L && FL &&  FR && !R) ||
                 (L && !FL &&  FR && !R) ) {

            timesToTurn = (uint32_t)(90 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            set_motion(TURN_LEFT); // turn right until obstacle not detected
            AvoidingObstacle=true;
        }

        // 45 DEGREE RIGHT TURN SCENARIOS
        else if (R && !FR &&  !FL && !L) {

            timesToTurn = (uint32_t)(0 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            set_motion(TURN_RIGHT); // turn left until obstacle not detected
            AvoidingObstacle=true;
        }

        // 90 DEGREE RIGHT TURN SCENARIOS
        else if ( (R && FR &&  !FL && !L) ||
                  (!R && FR &&  !FL && !L) ) {

            timesToTurn = (uint32_t)(45 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            set_motion(TURN_RIGHT); // turn left until obstacle not detected
            AvoidingObstacle=true;
        }

        // 135 DEGREE RIGHT TURN SCENARIOS
        else if( (R && FR &&  FL && !L) ||
                 (R && !FR &&  FL && !L) ) {

            timesToTurn = (uint32_t)(90 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            set_motion(TURN_RIGHT); // turn left until obstacle not detected
            AvoidingObstacle=true;
        }

        // 180 DEGREE TURN SCENARIOS
        else if ((!L && FL &&  FR && !R) ||
                 (L && !FL &&  !FR && R) ||
                 (L && FL &&  FR && R) ||
                 (L && FL &&  !FR && R) ||
                 (R && FR &&  !FL && L) ) {

            timesToTurn = (uint32_t)(135 * ticks_per_rotation_degree);
            last_motion_ticks = kilo_ticks;

            if(rand()%2){
                set_motion(TURN_LEFT); // turn left until obstacle not detected
            }
            else {
                set_motion(TURN_RIGHT);
            }
            AvoidingObstacle=true;
        }
    }
    else{

        if(!goforward){
            if(kilo_ticks > last_motion_ticks+timesToTurn){
                last_motion_ticks=kilo_ticks;
                set_motion(FORWARD); // turn right until obstacle not detected
                goforward = true;
            }
        }
        else{
            if (kilo_ticks>last_motion_ticks+(STRAIGHT_TIME/4)){
                goforward = false;
                AvoidingObstacle=false;
                if(behave_previous==GO_HOME){
                    behave = behave_previous;
                }
                else{
                    behave= SEARCH_FOOD;
                }
            }
        }

    }
}
void loop(){
    switch (behave){
    case SEARCH_FOOD: {
        set_random_motion();
        if(atFood){ // if food found
            travelTime=kilo_ticks; // start counting from when you arrive to food
            atFood = false;
            //Go at home leaving pheromone
            checkPheroQuality();
            last_pheromone_deposit_ticks = kilo_ticks;
            behave = GO_HOME;
            set_motion(STOP);
        }
        else if(atPheromone){ //Check if pheromone found
            if (!runtime_identification) set_color(RGB(3, 0, 0));
            backup_LED = 1; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
            behave = FOLLOW_PH;
            last_motion_ticks = kilo_ticks - MIN_TIME_BEFORE_SPIN - 1;
        }
        else { // Green, random movement, searching food
            if (!runtime_identification) set_color(RGB(0, 3, 0));
            backup_LED = 2; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)
        }
        break;
    }
    case GO_HOME: {
        if(!atHome){
            //If there is food available go to collect it
            returnHome();
        }else{
            //At Home
            atHome = false;
            travelTime=kilo_ticks-travelTime;
            atObstacle = false; //set obstacle check to false
            if (!runtime_identification) set_color(RGB(3, 0, 0));
            backup_LED = 1; // (0 - OFF, 1 - RED, 2 - GREEN, 3 - BLUE)

            /* Probability of depositing pheromone */
            double beta=1/(alpha+1);
            double probability = ( pow(beta,2.0) * exp( beta* ( travelTime- travelTime_max*30 ) / sqrt(travelTime) ) )*100;

            uint8_t randNum = rand() % 100;

            /* Print pheromone */
            if(probability > randNum){
                behave = SEARCH_FOOD;
            }
            else{
                //Turn 180 degrees to follow the PH
                behave = TURN_KILOBOT;
                timesToTurn = (uint32_t)(180 * ticks_per_rotation_degree);
                last_motion_ticks = kilo_ticks;
                turn180completed = false;
                if (rand()%2) set_motion(TURN_LEFT); else set_motion(TURN_RIGHT);
            }
        }
        break;
    }

    case AVOID_OBSTACLE: {	// dedicate obstacle avoidance behaviour

        if(atObstacle || AvoidingObstacle){
            avoidObstacle();
        }
        else{
            //            behave = behave_previous;
        }

        break;
    }

    case TURN_KILOBOT: {
        if(atFood){
            travelTime=kilo_ticks; // start counting from when you arrive to food
            atFood = false;
            checkPheroQuality();
            last_pheromone_deposit_ticks = kilo_ticks;
            behave = GO_HOME;
            set_motion(STOP);
        } else {
            if(kilo_ticks > last_motion_ticks + timesToTurn){
                //After turning 180 go forward to find the path
                set_motion(FORWARD);
                last_motion_ticks = kilo_ticks;
                timesToTurn = 15;
                if (turn180completed){
                    behave = SEARCH_FOOD;
                } else {
                    turn180completed = true;
                }
            }
        }
        //If it was lost but now it finds ph, check where to go
        if (atPheromone){
            behave = FOLLOW_PH;
            turn180completed = false;
            last_motion_ticks = kilo_ticks - MIN_TIME_BEFORE_SPIN - 1;
        }
        break;
    }
    case FOLLOW_PH: {
        if(atFood){
            travelTime=kilo_ticks; // start counting from when you arrive to food
            atFood = false;
            checkPheroQuality();
            last_pheromone_deposit_ticks = kilo_ticks;
            behave = GO_HOME;
            set_motion(STOP);
        } else {
            // If pheromone is detected
            if(atPheromone && kilo_ticks > last_motion_ticks + MIN_TIME_BEFORE_SPIN){
                atPheromone = false;
                //If the angle is smaller than 180 degrees is faster to go left
                if (best_pheromone_angle < 180){
                    set_motion(TURN_LEFT);
                } else {
                    //If the angle is bigger than 180 degrees is faster to go right
                    best_pheromone_angle = 360 - best_pheromone_angle;
                    set_motion(TURN_RIGHT);
                }
                timesToTurn = (uint32_t)(best_pheromone_angle * ticks_per_rotation_degree);
                last_motion_ticks = kilo_ticks;
                countLost = kilo_ticks;
            } else { // if I didn't receive atPheromone message

                // I keep turning and when I finished I try to move in the direction opposite to home
                if (kilo_ticks > last_motion_ticks + timesToTurn) {
                    //Turn 180 degrees to follow the PH
                    behave = TURN_KILOBOT;
                    turn180completed = false;
                    timesToTurn = (uint32_t)(180 * ticks_per_rotation_degree);

                    last_motion_ticks = kilo_ticks;
                    // select opposite direction than before
                    if (current_motion == TURN_LEFT)
                        set_motion(TURN_RIGHT);
                    else
                        set_motion(TURN_LEFT);
                }

                // if I don't receive information from ARK about Pheromone for more than 5 seconds, I declare myself as Lost and I start random walk! :-(
                if (kilo_ticks > countLost + 160) {
                    behave = SEARCH_FOOD;
                }
            }
        }
        break;
    }
    }
}

int main(){
    kilo_init();
    kilo_message_rx = message_rx;
    kilo_start(setup, loop);

    return 0;
}
