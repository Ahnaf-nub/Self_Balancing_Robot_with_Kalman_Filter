class KalmanFilter {
public:
  KalmanFilter() {
    Q_angle = 0.001;
    Q_bias = 0.003;
    R_measure = 0.03;

    angle = 0;
    bias = 0;
    P[0][0] = 0;
    P[0][1] = 0;
    P[1][0] = 0;
    P[1][1] = 0;
  }

  float getAngle(float newAngle, float newRate, float dt) {
    // Step 1
    rate = newRate - bias;
    angle += dt * rate;

    // Step 2
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Step 3
    float S = P[0][0] + R_measure;
    
    // Step 4
    float K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Step 5
    float y = newAngle - angle;

    // Step 6
    angle += K[0] * y;
    bias += K[1] * y;

    // Step 7
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }

private:
  float Q_angle;
  float Q_bias;
  float R_measure;

  float angle;
  float bias;
  float rate;

  float P[2][2];
};
