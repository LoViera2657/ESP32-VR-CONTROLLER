#include <Wire.h>
#include <MPU6050.h>


// ============================================================
// OBJETOS
// ============================================================

MPU6050 mpu;


// ============================================================
// CONSTANTES DO MPU6050
// ============================================================

const float G = 9.80665f;

// Acelerometro configurado em ±4 g
const float FATOR_ACEL = G / 8192.0f;

// Giroscopio configurado em ±1000 °/s
const float FATOR_GIRO = 1.0f / 32.8f;


// ============================================================
// BOTAO RECENTER
// ============================================================

const int PIN_RECENTER = 25;

bool estadoAnteriorRecenter = HIGH;

unsigned long ultimoRecenter = 0;

const unsigned long DEBOUNCE_RECENTER = 50;


// ============================================================
// CALIBRACAO DO ACELEROMETRO
// ============================================================

// Eixo X
const float OFFSET_AX = 4.0f;
const float GANHO_AX  = 1.0105f;

// Eixo Y
const float OFFSET_AY = 380.0f;
const float GANHO_AY  = 1.0053f;

// Eixo Z
const float OFFSET_AZ = -1157.0f;
const float GANHO_AZ  = 0.9813f;


// ============================================================
// CALIBRACAO DO GIROSCOPIO
// ============================================================

float biasGx = 0.0f;
float biasGy = 0.0f;
float biasGz = 0.0f;


// ============================================================
// QUATERNION
//
// q0 = w
// q1 = x
// q2 = y
// q3 = z
// ============================================================

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

bool quaternionInicializado = false;


// ============================================================
// TEMPO
// ============================================================

unsigned long tempoAnterior = 0;


// ============================================================
// NORMALIZACAO DO QUATERNION
// ============================================================

void normalizarQuaternion()
{
    float norma = sqrt(
        q0 * q0 +
        q1 * q1 +
        q2 * q2 +
        q3 * q3
    );

    if (norma > 0.0f)
    {
        q0 /= norma;
        q1 /= norma;
        q2 /= norma;
        q3 /= norma;
    }
}


// ============================================================
// CALIBRACAO DO GIROSCOPIO
// ============================================================

void calibrarGiroscopio()
{
    const int N = 1000;

    long somaGx = 0;
    long somaGy = 0;
    long somaGz = 0;

    Serial.println("Mantenha o MPU6050 completamente parado...");
    delay(2000);

    Serial.println("Calibrando giroscopio...");

    for (int i = 0; i < N; i++)
    {
        int16_t ax, ay, az;
        int16_t gx, gy, gz;

        mpu.getMotion6(
            &ax, &ay, &az,
            &gx, &gy, &gz
        );

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


// ============================================================
// INICIALIZACAO DO QUATERNION
//
// Roll e pitch sao obtidos pela gravidade.
// Yaw inicia em zero porque o MPU6050 nao possui magnetometro.
// ============================================================

void inicializarQuaternion(
    float rollGraus,
    float pitchGraus
)
{
    float roll  = rollGraus  * DEG_TO_RAD;
    float pitch = pitchGraus * DEG_TO_RAD;
    float yaw   = 0.0f;

    float cr = cos(roll * 0.5f);
    float sr = sin(roll * 0.5f);

    float cp = cos(pitch * 0.5f);
    float sp = sin(pitch * 0.5f);

    float cy = cos(yaw * 0.5f);
    float sy = sin(yaw * 0.5f);

    q0 =
        cr * cp * cy +
        sr * sp * sy;

    q1 =
        sr * cp * cy -
        cr * sp * sy;

    q2 =
        cr * sp * cy +
        sr * cp * sy;

    q3 =
        cr * cp * sy -
        sr * sp * cy;

    normalizarQuaternion();

    quaternionInicializado = true;
}


// ============================================================
// MADGWICK
//
// Durante aceleracoes fortes, se o acelerometro nao for
// considerado confiavel, usa somente o giroscopio.
// ============================================================

void atualizarMadgwick(
    float gx,
    float gy,
    float gz,
    float ax,
    float ay,
    float az,
    float dt,
    bool accelConfiavel
)
{
    const float BETA = 0.08f;

    // Madgwick usa rad/s
    gx *= DEG_TO_RAD;
    gy *= DEG_TO_RAD;
    gz *= DEG_TO_RAD;


    // --------------------------------------------------------
    // MODO GYRO-ONLY
    // --------------------------------------------------------

    if (!accelConfiavel)
    {
        float qDot0 =
            0.5f * (-q1 * gx - q2 * gy - q3 * gz);

        float qDot1 =
            0.5f * (q0 * gx + q2 * gz - q3 * gy);

        float qDot2 =
            0.5f * (q0 * gy - q1 * gz + q3 * gx);

        float qDot3 =
            0.5f * (q0 * gz + q1 * gy - q2 * gx);

        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;

        normalizarQuaternion();

        return;
    }


    // --------------------------------------------------------
    // NORMALIZACAO DO ACELEROMETRO
    // --------------------------------------------------------

    float normaAcel = sqrt(
        ax * ax +
        ay * ay +
        az * az
    );

    if (normaAcel <= 0.0f)
    {
        return;
    }

    ax /= normaAcel;
    ay /= normaAcel;
    az /= normaAcel;


    // --------------------------------------------------------
    // TERMOS AUXILIARES
    // --------------------------------------------------------

    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;

    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;

    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;

    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;


    // --------------------------------------------------------
    // GRADIENTE
    // --------------------------------------------------------

    float s0 =
        _4q0 * q2q2 +
        _2q2 * ax +
        _4q0 * q1q1 -
        _2q1 * ay;

    float s1 =
        _4q1 * q3q3 -
        _2q3 * ax +
        4.0f * q0q0 * q1 -
        _2q0 * ay -
        _4q1 +
        _8q1 * q1q1 +
        _8q1 * q2q2 +
        _4q1 * az;

    float s2 =
        4.0f * q0q0 * q2 +
        _2q0 * ax +
        _4q2 * q3q3 -
        _2q3 * ay -
        _4q2 +
        _8q2 * q1q1 +
        _8q2 * q2q2 +
        _4q2 * az;

    float s3 =
        4.0f * q1q1 * q3 -
        _2q1 * ax +
        4.0f * q2q2 * q3 -
        _2q2 * ay;


    float normaGradiente = sqrt(
        s0 * s0 +
        s1 * s1 +
        s2 * s2 +
        s3 * s3
    );

    if (normaGradiente > 0.0f)
    {
        s0 /= normaGradiente;
        s1 /= normaGradiente;
        s2 /= normaGradiente;
        s3 /= normaGradiente;
    }


    // --------------------------------------------------------
    // DERIVADA DO QUATERNION
    // --------------------------------------------------------

    float qDot0 =
        0.5f * (-q1 * gx - q2 * gy - q3 * gz)
        - BETA * s0;

    float qDot1 =
        0.5f * (q0 * gx + q2 * gz - q3 * gy)
        - BETA * s1;

    float qDot2 =
        0.5f * (q0 * gy - q1 * gz + q3 * gx)
        - BETA * s2;

    float qDot3 =
        0.5f * (q0 * gz + q1 * gy - q2 * gx)
        - BETA * s3;


    // --------------------------------------------------------
    // INTEGRACAO
    // --------------------------------------------------------

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    normalizarQuaternion();
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    pinMode(
        PIN_RECENTER,
        INPUT_PULLUP
    );

    delay(3000);

    Wire.begin(21, 22);

    Serial.println("Iniciando o MPU6050...");

    mpu.initialize();

    if (!mpu.testConnection())
    {
        Serial.println("Erro ao conectar ao MPU6050");

        while (true)
        {
            delay(1000);
        }
    }

    Serial.println("MPU6050 conectado com sucesso!");

    // Configuracoes adequadas para casting rapido
    mpu.setFullScaleGyroRange(
        MPU6050_GYRO_FS_1000
    );

    mpu.setFullScaleAccelRange(
        MPU6050_ACCEL_FS_4
    );

    calibrarGiroscopio();

    tempoAnterior = micros();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // --------------------------------------------------------
    // 1. LEITURA RAW
    // --------------------------------------------------------

    int16_t ax, ay, az;
    int16_t gx, gy, gz;

    mpu.getMotion6(
        &ax, &ay, &az,
        &gx, &gy, &gz
    );


    // --------------------------------------------------------
    // 2. CALIBRACAO + CONVERSAO DO ACELEROMETRO
    // --------------------------------------------------------

    float axCorrigido =
        (ax - OFFSET_AX) * GANHO_AX;

    float ayCorrigido =
        (ay - OFFSET_AY) * GANHO_AY;

    float azCorrigido =
        (az - OFFSET_AZ) * GANHO_AZ;


    float ax_ms2 =
        axCorrigido * FATOR_ACEL;

    float ay_ms2 =
        ayCorrigido * FATOR_ACEL;

    float az_ms2 =
        azCorrigido * FATOR_ACEL;


    // --------------------------------------------------------
    // 3. CALIBRACAO + CONVERSAO DO GIROSCOPIO
    // --------------------------------------------------------

    float gx_dps =
        (gx - biasGx) * FATOR_GIRO;

    float gy_dps =
        (gy - biasGy) * FATOR_GIRO;

    float gz_dps =
        (gz - biasGz) * FATOR_GIRO;


    // --------------------------------------------------------
    // 4. DELTA DE TEMPO
    // --------------------------------------------------------

    unsigned long tempoAtual =
        micros();

    float dt =
        (tempoAtual - tempoAnterior)
        / 1000000.0f;

    tempoAnterior =
        tempoAtual;


    // --------------------------------------------------------
    // 5. CONFIABILIDADE DO ACELEROMETRO
    // --------------------------------------------------------

    float moduloAcel = sqrt(
        ax_ms2 * ax_ms2 +
        ay_ms2 * ay_ms2 +
        az_ms2 * az_ms2
    );

    bool accelConfiavel =
        moduloAcel > 0.8f * G &&
        moduloAcel < 1.2f * G;


    // --------------------------------------------------------
    // 6. ROLL E PITCH INICIAIS PELA GRAVIDADE
    // --------------------------------------------------------

    float rollAcc =
        atan2(
            ay_ms2,
            az_ms2
        ) * RAD_TO_DEG;

    float pitchAcc =
        atan2(
            -ax_ms2,
            sqrt(
                ay_ms2 * ay_ms2 +
                az_ms2 * az_ms2
            )
        ) * RAD_TO_DEG;


    // --------------------------------------------------------
    // 7. INICIALIZACAO DO QUATERNION
    // --------------------------------------------------------

    if (
        !quaternionInicializado &&
        accelConfiavel
    )
    {
        inicializarQuaternion(
            rollAcc,
            pitchAcc
        );
    }


    // --------------------------------------------------------
    // 8. MADGWICK
    // --------------------------------------------------------

    if (quaternionInicializado)
    {
        atualizarMadgwick(
            gx_dps,
            gy_dps,
            gz_dps,

            ax_ms2,
            ay_ms2,
            az_ms2,

            dt,
            accelConfiavel
        );
    }


    // --------------------------------------------------------
    // 9. RECENTER
    // --------------------------------------------------------

    bool estadoRecenter =
        digitalRead(PIN_RECENTER);

    if (
        estadoAnteriorRecenter == HIGH &&
        estadoRecenter == LOW &&
        millis() - ultimoRecenter >
            DEBOUNCE_RECENTER
    )
    {
        Serial.println("R");

        ultimoRecenter =
            millis();
    }

    estadoAnteriorRecenter =
        estadoRecenter;


    // --------------------------------------------------------
    // 10. ENVIO SERIAL
    // --------------------------------------------------------

    Serial.print("Q,");
    Serial.print(q0, 6);
    Serial.print(",");
    Serial.print(q1, 6);
    Serial.print(",");
    Serial.print(q2, 6);
    Serial.print(",");
    Serial.println(q3, 6);


    delay(10);
}
