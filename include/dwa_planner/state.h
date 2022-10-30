#ifndef __STATE_H
#define __STATE_H
 
class State
{
    public:
        State(double, double, double, double, double);

        double x;
        double y;
        double heading;
        double velocity;
        double omega;
};

#endif //__STATE_H