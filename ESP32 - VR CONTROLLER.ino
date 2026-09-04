#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float biasGx = 0.0;
float biasGy = 0.0;
float biasGz = 0.0;

// Orientacao filtrada
float rollFiltro = 0.0f;
float pitchFiltro = 0.0f;

float yawGyro = 0.0f;

bool filtroInicializado = false;

unsigned long tempoAnterior = 0;

float normalizarAngulo(float angulo) {

    while (angulo > 180.0f) {
        angulo -= 360.0f;
    }

    while (angulo < -180.0f) {
        angulo += 360.0f;
    }

    return angulo;
}

void calibrarGiroscopio() {
    const int N = 1000;

    long somaGx = 0;
    long somaGy = 0;
    long somaGz = 0;

    Serial.println("Mantenha o MPU6050 completamente parado...");
    delay(2000);

    Serial.println("Calibrando giroscopio...");

    for (int i = 0; i < N; i++) {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        somaGx += gx;
        somaGy += gy;
        somaGz += gz;

        delay(2);
    }

    biasGx = (float)somaGx / N;
    biasGy = (float)somaGy / N;
    biasGz = (float)somaGz / N;

    Serial.println("Calibracao concluida!");

    Serial.print("Bias RAW Gx: ");
    Serial.println(biasGx);

    Serial.print("Bias RAW Gy: ");
    Serial.println(biasGy);

    Serial.print("Bias RAW Gz: ");
    Serial.println(biasGz);

    Serial.println();

    delay(2000);
}

void setup() {
    Serial.begin(115200);
    delay(3000);

    Wire.begin(21, 22); // SDA e SCL

    Serial.println("Iniciando o MPU6050...");

    mpu.initialize();

    if (mpu.testConnection()) {
        Serial.println("MPU6050 conectado com sucesso!");
    }
    else {
        Serial.println("Erro ao conectar ao MPU6050");
        while (1);
    }
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);

    calibrarGiroscopio();
    tempoAnterior = micros();
}

void loop() {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float ax_ms2, ay_ms2, az_ms2;
    float gx_dps, gy_dps, gz_dps;

     float roll_acc, pitch_acc;

    mpu.getMotion6(
        &ax, &ay, &az,
        &gx, &gy, &gz
    );

    const float FATOR_ACEL = 9.80665 / 8192.0;
    const float FATOR_GIRO = 1.0f / 32.8f;

    ax_ms2 = ax * FATOR_ACEL;
    
    float ay_corrigido = (ay - 409.0f) * 0.998f;
    ay_ms2 = ay_corrigido * FATOR_ACEL;

    float az_corrigido = (az + 1157.0f)*0.9813f;
    az_ms2 = az_corrigido * FATOR_ACEL;


    gx_dps = (gx - biasGx) * FATOR_GIRO;
    gy_dps = (gy - biasGy) * FATOR_GIRO;
    gz_dps = (gz - biasGz) * FATOR_GIRO;

    unsigned long tempoAtual = micros();

    float dt = (tempoAtual - tempoAnterior)/1000000.0f;
    tempoAnterior = tempoAtual;  
    
    
    roll_acc= atan2(ay_ms2, az_ms2) * 180.0f / PI;

    pitch_acc= atan2(
        -ax_ms2, sqrt(ay_ms2 * ay_ms2 + az_ms2 * az_ms2)
    ) * 180.0f / PI;


    if(!filtroInicializado){
        rollFiltro = roll_acc;
        pitchFiltro = pitch_acc;

        yawGyro = 0.0f;
        filtroInicializado = true;
    }

    const float ALPHA = 0.98f;

    float rollPrevisto = rollFiltro + gx_dps * dt;
    float erroRoll = 
        normalizarAngulo(roll_acc - rollPrevisto);

    rollFiltro = 
        normalizarAngulo(rollPrevisto + (1.0f - ALPHA) * erroRoll);


    float pitchPrevisto = pitchFiltro + gy_dps * dt;
    pitchFiltro = ALPHA * pitchPrevisto + (1.0f - ALPHA) * pitch_acc;

    yawGyro += gz_dps * dt;
    yawGyro = normalizarAngulo(yawGyro);


    Serial.print("Aceleracao (X, Y, Z): ");
    Serial.print(ax_ms2); Serial.print(" m/s^2, ");
    Serial.print(ay_ms2); Serial.print(" m/s^2, ");
    Serial.print(az_ms2); Serial.print(" m/s^2");

    Serial.print(" | Giro (X, Y, Z): ");
    Serial.print(gx_dps); Serial.print(" dps, ");
    Serial.print(gy_dps); Serial.print(" dps, ");
    Serial.print(gz_dps); Serial.print(" dps");

    Serial.print(" | Roll ACC: "); Serial.print(roll_acc);
    Serial.print(" Roll FILTRO: "); Serial.print(rollFiltro);

    Serial.print(" | Pitch ACC: "); Serial.print(pitch_acc);
    Serial.print(" Pitch FILTRO: "); Serial.print(pitchFiltro);

    Serial.print(" | Yaw Gyro: ");
    Serial.println(yawGyro);


    delay(10);
}
