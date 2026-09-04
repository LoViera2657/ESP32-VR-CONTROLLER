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

    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    Serial.print("\033[2J\033[H");

    Serial.print("Aceleracao (X, Y, Z): ");
    Serial.print(ax); Serial.print(", ");
    Serial.print(ay); Serial.print(", ");
    Serial.print(az);
    Serial.print(" | Giro (X, Y, Z): ");
    Serial.print(gx); Serial.print(", ");
    Serial.print(gy); Serial.print(", ");
    Serial.println(gz);
    

    delay(100);
}
