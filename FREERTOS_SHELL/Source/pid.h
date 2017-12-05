
#ifndef _PID_H_
#define _PID_H_

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( int16_t dt, int16_t max, int16_t min, int16_t Kp, int16_t Kd, int16_t Ki );

        // Returns the manipulated variable given a setpoint and current process value
        int16_t calculate( int16_t setpoint, int16_t pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif
