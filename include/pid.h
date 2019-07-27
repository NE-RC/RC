#ifndef PID
#define PID

/**
 * Simple PID + FeedForward Controller.
**/
class PIDController{

  private:
    double kP, kI, kD, kF;
    double integral_error, prev_error;
    double target;
    bool has_target;

  public:
    PIDController(double kP, double kI, double kD, double kF);
    PIDController();

    void set_gains(double kP, double kI, double kD, double kF);
    void set_target(double target);
    double update(double current);
    void reset();
};

#endif
