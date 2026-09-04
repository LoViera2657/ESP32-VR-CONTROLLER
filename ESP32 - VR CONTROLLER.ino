#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float biasGx = 0.0;
float biasGy = 0.0;
float biasGz = 0.0;

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

    calibrarGiroscopio();
}

void loop() {

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float ax_ms2, ay_ms2, az_ms2;
    float gx_dps, gy_dps, gz_dps;

    mpu.getMotion6(
        &ax, &ay, &az,
        &gx, &gy, &gz
    );

    const float FATOR_ACEL = 9.80665 / 16384.0;
    const float FATOR_GIRO = 1.0 / 131.0;

    ax_ms2 = ax * FATOR_ACEL;
    ay_ms2 = ay * FATOR_ACEL;

    float az_corrigido = (az + 2120.0f)*0.9813f;
    az_ms2 = az_corrigido * FATOR_ACEL;

    gx_dps = (gx - biasGx) * FATOR_GIRO;
    gy_dps = (gy - biasGy) * FATOR_GIRO;
    gz_dps = (gz - biasGz) * FATOR_GIRO;


    Serial.print("Aceleracao (X, Y, Z): ");
    Serial.print(ax_ms2); Serial.print(" m/s^2, ");
    Serial.print(ay_ms2); Serial.print(" m/s^2, ");
    Serial.print(az_ms2); Serial.print(" m/s^2");

    Serial.print(" | Giro (X, Y, Z): ");
    Serial.print(gx_dps); Serial.print(" dps, ");
    Serial.print(gy_dps); Serial.print(" dps, ");
    Serial.print(gz_dps); Serial.println(" dps");

    delay(100);
}
