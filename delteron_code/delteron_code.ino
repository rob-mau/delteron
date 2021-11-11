//-----------------------------------------------------------------------------------------------
//  INCLUSIONE LIBRERIE
//-----------------------------------------------------------------------------------------------
#include <AccelStepper.h>
#include <MultiStepper.h>
//-----------------------------------------------------------------------------------------------
//  DEFINIZIONE COSTANTI
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//  costanti geometriche del robot (usate nella funzione inverse_kinematics())
//-----------------------------------------------------------------------------------------------
#define H               45.0      // [mm]   - z-offset
#define D               60.0      // [mm]   - dist. di un vertice del dell'end-eff. dal suo asse
#define DELTA           0.3927    // [rad]  - angolo di inclinazione dei bracci (22,5°)
#define LEAD            1.25      // [mm]   - passo della vite
//-----------------------------------------------------------------------------------------------
//  costanti dei motori stepper
//-----------------------------------------------------------------------------------------------
#define VEL             1600.0    // [half-step/s] (4.0 revolution/s)
#define V_HOME          2000.0    // [half-step/s] (5.0 revolution/s)
#define A_HOME          5000.0    // [half-step/s^2]
#define STEP_TO_DEG     0.9       // [degree/half-step]
//-----------------------------------------------------------------------------------------------
//  etichette dei pin
//-----------------------------------------------------------------------------------------------
#define STP_1           2
#define STP_2           3
#define STP_3           4

#define DIR_1           5
#define DIR_2           6
#define DIR_3           7

#define EN              8

#define LIM_1           9
#define LIM_2           10
#define LIM_3           11
//-----------------------------------------------------------------------------------------------
//  costanti ausiliarie
//-----------------------------------------------------------------------------------------------
#define NUM_CHARS       100     // numero di caratteri per la variabile "received_chars"
#define BAUD_RATE       9600    // [bit/s]
//-----------------------------------------------------------------------------------------------
//  DEFINIZIONE VARIABILI GLOBALI
//-----------------------------------------------------------------------------------------------
AccelStepper axis_1(1, STP_1, DIR_1);
AccelStepper axis_2(1, STP_2, DIR_2);
AccelStepper axis_3(1, STP_3, DIR_3);

MultiStepper multi_axis;

struct point {
  float x;
  float y;
  float z;
} pt;

struct joint {
  float m1;
  float m2;
  float m3;
} th;

char  received_chars[NUM_CHARS];
bool  new_data = false;
long  num_G = 0;
//-----------------------------------------------------------------------------------------------
//  DICHIARAZIONE FUNZIONI
//-----------------------------------------------------------------------------------------------
void  recv_with_start_end_markers();
void  parse_data();
void  read_number(char*);
void  show_parsed_data();
void  homing();
void  run_to_position();
point deformation_correction(point);
joint inverse_kinematics(point);
//-----------------------------------------------------------------------------------------------
//  MAIN
//-----------------------------------------------------------------------------------------------
void setup() {

    pinMode(LIM_1, INPUT_PULLUP);
    pinMode(LIM_2, INPUT_PULLUP);
    pinMode(LIM_3, INPUT_PULLUP);
    pinMode(EN, OUTPUT);
    digitalWrite(EN, LOW);

    delay(50);  // attesa per il "risveglio" dei driver

    multi_axis.addStepper(axis_1);
    multi_axis.addStepper(axis_2);
    multi_axis.addStepper(axis_3);

    Serial.begin(BAUD_RATE);

    Serial.println("Homing . . .");
    homing();
    Serial.println("Homing Completed");
    delay(1000);
    Serial.println("Start!");
}

void loop() {

    receive_data();
    if (new_data == true) {
        parse_data();
        show_parsed_data();
        new_data = false;

        run_to_position();
        Serial.println("Positioned");
    }
}
//-----------------------------------------------------------------------------------------------
//  DEFINIZIONE FUNZIONI
//-----------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------
//  homing(): posiziona i motori nella configurazione "home"
//-----------------------------------------------------------------------------------------------
void homing() {

long axis_1_pos = -1;
long axis_2_pos = -1;
long axis_3_pos = -1;

    axis_1.setAcceleration(A_HOME);
    axis_2.setAcceleration(A_HOME);
    axis_3.setAcceleration(A_HOME);

    axis_1.setMaxSpeed(V_HOME);
    axis_2.setMaxSpeed(V_HOME);
    axis_3.setMaxSpeed(V_HOME);

    while (digitalRead(LIM_1) || digitalRead(LIM_2) || digitalRead(LIM_3)) {
        if (digitalRead(LIM_1)) {
            axis_1.moveTo(axis_1_pos);
            axis_1.run();
            axis_1_pos--;
        }
        if (digitalRead(LIM_2)) {
            axis_2.moveTo(axis_2_pos);
            axis_2.run();
            axis_2_pos--;
        }
        if (digitalRead(LIM_3)) {
            axis_3.moveTo(axis_3_pos);
            axis_3.run();
            axis_3_pos--;
        }
    }
    delay(10);

    axis_1.setCurrentPosition(0);
    axis_2.setCurrentPosition(0);
    axis_3.setCurrentPosition(0);

    axis_1_pos = 1;
    axis_2_pos = 1;
    axis_3_pos = 1;

    while (!digitalRead(LIM_1) || !digitalRead(LIM_2) || !digitalRead(LIM_3)) {
        if (!digitalRead(LIM_1)) {
            axis_1.moveTo(axis_1_pos);
            axis_1.run();
            axis_1_pos++;
        }
        if (!digitalRead(LIM_2)) {
            axis_2.moveTo(axis_2_pos);
            axis_2.run();
            axis_2_pos++;
        }
        if (!digitalRead(LIM_3)) {
            axis_3.moveTo(axis_3_pos);
            axis_3.run();
            axis_3_pos++;
        }
    }
    delay(10);

    axis_1.setCurrentPosition(0);
    axis_2.setCurrentPosition(0);
    axis_3.setCurrentPosition(0);

    axis_1.setMaxSpeed(VEL);
    axis_2.setMaxSpeed(VEL);
    axis_3.setMaxSpeed(VEL);
}
//-----------------------------------------------------------------------------------------------
//  run_to_position(): posiziona i motori dove richiesto
//-----------------------------------------------------------------------------------------------
void run_to_position(){

point pt_corr;
long  pos[3];

    pt_corr = deformation_correction(pt);
    th = inverse_kinematics(pt_corr);

    // conversion: [rad] --> [half-step]
    pos[0] = th.m1 * RAD_TO_DEG / STEP_TO_DEG;
    pos[1] = th.m2 * RAD_TO_DEG / STEP_TO_DEG;
    pos[2] = th.m3 * RAD_TO_DEG / STEP_TO_DEG;

    multi_axis.moveTo(pos);
    multi_axis.runSpeedToPosition();
}
//-----------------------------------------------------------------------------------------------
//  deformation_correction(): corregge alcuni effetti dovuti a deformazione del robot
//-----------------------------------------------------------------------------------------------
point deformation_correction(point in){

point out;
float ang = 0.18;       // rotation angle
float s_x = 1.0;        // scale factor x-axis
float s_y = 0.95;       // scale factor y-axis

    out.x = cos(ang) * in.x - sin(ang) * in.y;
    out.y = cos(ang) * in.y + sin(ang) * in.x;
    out.z = in.z;

    out.y *= s_x;
    out.y *= s_y;

    return out;
}
//-----------------------------------------------------------------------------------------------
//  inverse_kinematics(): risolve il problema di cinematica inversa. Prende in input
//                        la posizione dell'end-effector "in" e restituisce in output
//                        il valore degli angoli di giunto "out".
//-----------------------------------------------------------------------------------------------
joint inverse_kinematics(point in){

joint out;
float x, y, z;

    x = in.x;
    y = in.y;
    z = in.z - H;

    out.m1 = -2 * PI / LEAD * (z - tan(DELTA) * (D + y));
    out.m2 = -2 * PI / LEAD * (z - tan(DELTA) * (D - sqrt(3) * x / 2 - y / 2));
    out.m3 = -2 * PI / LEAD * (z - tan(DELTA) * (D + sqrt(3) * x / 2 - y / 2));

    return out;
}
//-----------------------------------------------------------------------------------------------
//  receive_data(): memorizza in "received_chars" i caratteri letti sulla seriale
//-----------------------------------------------------------------------------------------------
void receive_data() {

static bool recv_in_progress = false;
static byte index = 0;
char start_marker = 'G';
char end_marker = '\n';
char rc;

    while (Serial.available() > 0 && new_data == false) {
        rc = Serial.read();
        if (recv_in_progress == true) { 
            if (rc != end_marker) {
                received_chars[index] = rc;
                index++;
                if (index >= NUM_CHARS) index = NUM_CHARS - 1;
            }
            else {
                received_chars[index] = '\0'; // termina la stringa
                recv_in_progress = false;
                index = 0;
                new_data = true;
            }
        }        
        else if (rc == start_marker) {
            recv_in_progress = true;
            received_chars[index] = rc;
            index++;
        }
    }
}
//-----------------------------------------------------------------------------------------------
//  parse_data(): effettua il parsing della stringa ricevuta
//-----------------------------------------------------------------------------------------------
void parse_data() {

char  temp_chars[NUM_CHARS];      // stringa temporanea usata per il parsing
char* strtok_index;               // è usato da strtok() come un indice
char  del[] = " ";
int   i;

    strcpy(temp_chars, received_chars);
    strtok_index = strtok(temp_chars, del);
    read_number(strtok_index);

    for (i = 0; i < 3; i++) {
        strtok_index = strtok(NULL, del);
        read_number(strtok_index);
    }
}
//-----------------------------------------------------------------------------------------------
//  read_number(): estrapola il valore numerico contenuto nel frammento di stringa in input
//-----------------------------------------------------------------------------------------------
void read_number(char* str) {

    switch (str[0]) {
        case 'G':
            str[0] = ' ';
            num_G = atoi(str);
            break;

        case 'X':
            str[0] = ' ';
            pt.x = atof(str);
            break;

        case 'Y':
            str[0] = ' ';
            pt.y = atof(str);
            break;

        case 'Z':
            str[0] = ' ';
            pt.z = atof(str);
            break;

        default:
            break;
      } 
}
//-----------------------------------------------------------------------------------------------
//  show_parsed_data(): stampa i valori numerici estrapolati col processo di parsing
//-----------------------------------------------------------------------------------------------
void show_parsed_data() {

    Serial.print("X: ");
    Serial.println(pt.x, 3);

    Serial.print("Y: ");
    Serial.println(pt.y, 3);

    Serial.print("Z: ");
    Serial.println(pt.z, 3);
}
