

class KalmanFuse
{
private:
    float P[4];     //< covariance matrix
    float angle;    //< angle state
    float rate;     // input rate
    float angle_z;  //< pseudo absolute measurement
    float Q_a, Q_b; //< process noise for states
    float bias;     //< bias of the input rate
    float R_z;      //< measurement noise

public:
    /**
     * @brief Construct a new Kalman Filter object
     * 
     * @param angle_  : initial condition
     * @param p_angle : uncertainty on initial condition
     * @param Q_a_    : process noise for the angle state
     * @param Q_b_    : process noise for the bias state
     * @param R_z_    : measurement noise
     */
    KalmanFuse(float angle_, float p_angle, float Q_a_, float Q_b_, float R_z_);
    /**
     * @brief Get the new angle
     * 
     * @param angle_z : angle from pseudo absolute measurements
     * @param prevRate : input rate to be integrated
     * @param dt : time step
     * @return float : the new angle 
     */
    float getAngle(float angle_z, float prevRate, float dt);
    /**
     * @brief set the new angle
     * 
     * @param angle : angle in radians
     */
    void setAngle(float);
};

KalmanFuse::KalmanFuse(float angle_, float p_angle, float Q_a_, float Q_b_, float R_z_)
{
    Q_a = Q_a_;
    Q_b = Q_b_;
    R_z = R_z_;
    angle = angle_;
    P[0] = p_angle;
    P[1] = 0.0f;
    P[2] = 0.0f;
    P[3] = 0.0f;

    bias = 0.0f;
}

float KalmanFuse::getAngle(float angle_z, float prevRate, float dt)
{
    // Prediction Step

    rate = prevRate - bias;
    angle += rate * dt;

    P[0] += dt * (dt * P[3] - P[1] - P[2] + Q_a);
    P[1] -= dt * P[3];
    P[2] = P[1];
    P[3] += Q_b * dt;

    //Correction Step

    float y = angle_z - angle;
    float S = P[0] + R_z;
    //Kalman Gains
    float K1 = P[0] / S;
    float K2 = P[1] / S;

    angle += K1 * y;
    bias += K2 * y;

    // Update Covariance matrix

    float p0 = P[0];
    float p1 = P[1];
    P[0] -= K1 * p0;
    P[1] -= K1 * p1;
    P[2] -= K2 * p0;
    P[3] -= K2 * p1;

    return angle;
}

void KalmanFuse::setAngle(float angle_) { angle = angle_; };
