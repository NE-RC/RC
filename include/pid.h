#ifndef PID
#define PID

#include <algorithm>

/**
 * Simple PID + FeedForward Controller.
**/
class PIDController{

  private:
    double kP, kI, kD, kF;
    double integral_error, prev_error;
    double target;
    double min_output, max_output;
    double tolerance;
    bool has_target, at_target;

  public:
    PIDController(double kP, double kI, double kD, double kF);
    PIDController();

    void SetGains(double kP, double kI, double kD, double kF);
    void SetTarget(double target);
    void SetOutputBounds(double min, double max);
    void SetTolerance(double tolerance);
    double Update(double current);
    void Reset();
    bool Done() const;
};

#endif
