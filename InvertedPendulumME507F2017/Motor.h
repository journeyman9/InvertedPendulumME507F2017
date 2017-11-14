#ifndef MOTOR_H
#define MOTOR_H

class Motor{
	public:
		Motor(int omegam_measured, int I_actuator);
	private:
		volatile int xm_set;
		volatile int xm_error;
		volatile int xm_measured;
		volatile int radius;
		volatile int omegam_set;
		volatile int vm_set;
		volatile int omegam_measured;
		volatile int anglem_measured;
		volatile int Tset;
		volatile int Im_set;
		volatile int Voltm_set;
		volatile int K_T;
		volatile int I_motor;
		volatile int I_error;
		volatile int I_actuator;
		volatile int V_R;
		volatile int K_p;
		volatile int V_E;
		volatile int K_v;
		volatile int V_m;
		volatile int i_error_sum;
};

#endif



