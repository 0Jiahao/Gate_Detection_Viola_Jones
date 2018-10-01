

struct dronerace_control_struct
{
  // States
  float psi_ref;    ///< maintain a rate limited speed set-point to smooth heading changes

  // Outputs to inner loop
  float phi_cmd;
  float theta_cmd;
  float psi_cmd;
  float alt_cmd;
};

extern struct dronerace_control_struct dr_control;


extern void control_run(float dt,float *desired_phi,float *desired_theta,float *desired_psi,float *desired_alt);

extern void control_reset(void);

