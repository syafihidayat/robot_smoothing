#include <math.h>
class Convertion
{
private:
    
public:
    
    float toDeg(float radian)
    {
        return radian * 180 / M_PI;
    }
    float toRad(float radian)
    {
        return radian * M_PI / 180;
    }
    struct Quaternion
    {
        double w, x, y, z;
    };

    void quat_to_eular(Quaternion q, double &yaw, double &pitch, double &roll)
    {
        const double w2 = q.w * q.w;
        const double x2 = q.x * q.x;
        const double y2 = q.y * q.y;
        const double z2 = q.z * q.z;
        const double unitLength = w2 + x2 + y2 + z2; // Normalised == 1, otherwise correction divisor.
        const double abcd = q.w * q.x + q.y * q.z;
        const double eps = 1e-7; // TODO: pick from your math lib instead of hardcoding.
        if (abcd > (0.5 - eps) * unitLength)
        {
            yaw = 2 * atan2(q.y, q.w);
            pitch = M_PI;
            roll = 0;
        }
        else if (abcd < (-0.5 + eps) * unitLength)
        {
            yaw = -2 * ::atan2(q.y, q.w);
            pitch = -M_PI;
            roll = 0;
        }
        else
        {
            const double adbc = q.w * q.z - q.x * q.y;
            const double acbd = q.w * q.y - q.x * q.z;
            yaw = ::atan2(2 * adbc, 1 - 2 * (z2 + x2));
            pitch = ::asin(2 * abcd / unitLength);
            roll = ::atan2(2 * acbd, 1 - 2 * (y2 + x2));
        }
    }

    void euler_to_quat(float roll, float pitch, float yaw, float *q)
    {
        float cy = cos(yaw * 0.5);
        float sy = sin(yaw * 0.5);
        float cp = cos(pitch * 0.5);
        float sp = sin(pitch * 0.5);
        float cr = cos(roll * 0.5);
        float sr = sin(roll * 0.5);

        q[0] = cy * cp * cr + sy * sp * sr;
        q[1] = cy * cp * sr - sy * sp * cr;
        q[2] = sy * cp * sr + cy * sp * cr;
        q[3] = sy * cp * cr - cy * sp * sr;
    }
};