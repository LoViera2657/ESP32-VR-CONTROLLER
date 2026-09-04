#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup(){
    Serial.begin(115200);
    delay(2000);
    Wire.begin(21, 22);  //SDA e SCL

    Serial.println("Iniciando o MPU6050...");
    mpu.initialize();

    if(mpu.testConnection()){
        Serial.println("MPU6050 conectado com sucesso!");
    }
    else{
        Serial.println("Erro ao conectar ao MPU6050");
        while(1);
    }

}

void loop(){
    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    float ax_ms2, ay_ms2, az_ms2;
    float gx_dps, gy_dps, gz_dps;

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    const float FATOR_ACEL = 9.80665 / 16384.0;
    const float FATOR_GIRO = 1.0 / 131.0;

    ax_ms2 = ax * FATOR_ACEL;
    ay_ms2 = ay * FATOR_ACEL;
    az_ms2 = az * FATOR_ACEL;

    gx_dps = gx * FATOR_GIRO;
    gy_dps = gy * FATOR_GIRO;
    gz_dps = gz * FATOR_GIRO;

    
    Serial.print("\033[2J\033[H");

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
