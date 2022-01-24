class PID
{
private:
    double kp;
    double ki;
    double kd;
    double err_thresh;

    double last_err;
    double target;
    double error_sum;

public:
    PID(double& p, double& i, double& d, double& e_t);
    void setCoeffs(double p, double i, double d, double e_t);

    double get_kp();
    double get_ki();
    double get_kd();
    double get_target();

    void update_target(double new_target);
    double update(double measure, double dt);
    bool check_arrived();
};
