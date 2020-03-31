class Kalman
{
private:
    float Q_angle;
    float Q_bias;
    float R_measure;

    float P00 = 0.1, P01 = 0.1, P10 = 0.1, P11 = 0.1;
    float rate, angle;
    float K0, K1;
    float bias = 0.0;

public:
    Kalman(float Q_angle=0.001, float Q_bias=0.003, float R_measure=0.03);
    ~Kalman();

    float get_angle(float newRate, float newAngle, float dt);
    void set_angle(float initAngle);
};

Kalman::Kalman(float QAngle=0.001, float QBias=0.003, float RMeasure=0.03) {
    Q_angle = QAngle;
    Q_bias = QBias;
    R_measure = RMeasure;
}

Kalman::~Kalman() {}

void Kalman::set_angle(float initAngle) {
    angle = initAngle;
}

float Kalman::get_angle(float newRate, float newAngle, float dt) {
    //Time update step 1
    rate = newRate - bias;
    angle += rate * dt;

    //Time update step 2
    P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
    P01 -= dt * P11;
    P10 -= dt * P11;
    P11 += Q_bias * dt;

    //Measurement update step 1
    float S = P00 + R_measure;
    K0 = P00/S;
    K1 = P10/S;

    //Measurement update step 2
    float y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;

    //Measurement update step 3
    P00 -= K0 * P00;
    P01 -= K0 * P01;
    P10 -= K1 * P00;
    P11 -= K1 * P01;

    return angle;
}
