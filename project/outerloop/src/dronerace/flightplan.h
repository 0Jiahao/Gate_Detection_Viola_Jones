
#define RadOfDeg(a) a/180.0*3.14159

#define UPPER_GATE 0
#define LOWER_GATE 1


extern int flagHighOrLowGate;
struct dronerace_fp_struct
{
  // Current Gate Position
  int gate_nr;
  float gate_x;
  float gate_y;
  float gate_alt;
  float gate_psi;

  // Current Navigation Target
  float x_set;
  float y_set;
  float psi_set;
  float alt_set;
};


# define MAX_DETECTION 2

struct JungleGate
{
    bool flagJungleGateDetected;
    int numJungleGateDetection;
    float jungleGateHeight;
    float sumJungleGateHeight;
    float jungleGateDetection[MAX_DETECTION];
    bool flagInJungleGate;
    long long timeStartJungleGate;
};


// Variables
extern struct dronerace_fp_struct dr_fp;

#define MAX_GATES 7
#define  REGULAR 0
//#define NORMAL 0
#define JUNGLE 1
#define VIRTUAL 2
#define DIAL 3

#define BRAKE 1
#define NO_BRAKE 0

struct dronerace_flightplan_item_struct
{
    float x;
    float y;
    float alt;
    float psi;
    int type;
    int brake;
};

extern const struct dronerace_flightplan_item_struct gates[MAX_GATES];
extern const struct dronerace_flightplan_item_struct waypoints[MAX_GATES];
// Functions
extern void flightplan_reset(void);
extern void flightplan_run(void);
extern struct JungleGate jungleGate;
extern void resetJungleGate(void);
