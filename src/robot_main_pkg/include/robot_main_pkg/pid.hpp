#ifndef PID_H
#define PID_H

#include <math.h>

class PID
{
private:
    float min_val_, max_val_;
    float KP, KI, KD;

    float kp, kpT, ki, kiT, kd, kdT;

    float angular_vel_Prev;

    struct e
    {
        float proportional;
        float integral;
        float derivative;
        float u;
        float preveious;
    } err;

    struct heading_param
    {
        float kp;
        float ki;
        float kd;
    } headingParams;

    struct base_param
    {
        float kp;
        float ki;
        float kd;
    } baseParams;

    float lowpass_filt = 0;
    float lowpass_prev = 0;
    float radian = 0;
    float eProportional;
    float deg2target = 0;

    float encPrev;

    float angular_vel_Filt = 0;
    float angular_vel = 0;

    float PPR = 0;

    float error_integral = 0;
    float error_previous = 0;

    float eIntegral = 0;
    float prevError = 0;

public:
    void setBaseParam(float kp_, float ki_, float kd_)
    {
        baseParams.kp = kp_;
        baseParams.ki = ki_;
        baseParams.kd = kd_;
    };

    void setHeadingParam(float kp_, float ki_, float kd_)
    {
        headingParams.kp = kp_;
        headingParams.ki = ki_;
        headingParams.kd = kd_;
    };

    void ppr_total(float total_ppr)
    {
        PPR = total_ppr;
    }

    float control_base(float error, float speed)
    {
        err.proportional = error;
        err.integral += err.proportional;

        float eDerivative = (err.proportional - err.preveious);
        err.preveious = err.proportional;

        float u = baseParams.kp * err.proportional + baseParams.ki * err.integral + baseParams.kd * eDerivative;
        //     float u = KP * err.proportional + KI * err.integral + KD * eDerivative;


        return std::clamp(u, -speed, speed);
    }

    // float control_base(float error, float deltaT)
    // {
    //     err.proportional = error;
    //     err.integral += err.proportional * deltaT;

    //     float eDerivative = (err.proportional - err.preveious) / deltaT;
    //     err.preveious = err.proportional;

    //     float u = KP * err.proportional + KI * err.integral + KD * eDerivative;

    //     return fmax(min_val_, fmin(u, max_val_));
    // }

    float control_base_rotation(float error, float speed)
    {
        err.proportional = error;

        if (err.proportional > 180)
        {
            err.proportional -= 360;
        }
        else if (err.proportional < -180)
        {
            err.proportional += 360;
        }
        err.integral += err.proportional ;
        // err.integral += err.proportional * deltaT;

        // float eDerivative = (err.proportional - err.preveious) / deltaT;
        float eDerivative = (err.proportional - err.preveious) ;
        err.preveious = err.proportional;

        float uT = headingParams.kp * err.proportional + headingParams.ki * err.integral + headingParams.kd * eDerivative;
        // float u = KP * err.proportional + KI * err.integral + KD * eDerivative;

        // return fmax(min_val_, fmin(u, max_val_));
        return std::clamp(uT, -speed, speed);
    }

    float get_filt_vel()
    {
        return angular_vel_Filt;
    }
};

#endif