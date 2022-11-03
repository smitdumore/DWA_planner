#ifndef __STATE_H
#define __STATE_H
 
class State
{
    public:

        double x;
        double y;
        double heading;
        double velocity;
        double omega;

        //constructor
        State(double x, double y, double heading, double velocity, double omega){
            this->x = x;
            this->y = y;
            this->heading = heading;
            this->velocity = velocity;
            this->omega = omega;
        }

};

#endif //__STATE_H