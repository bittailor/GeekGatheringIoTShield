#include <I2Cdev.h>
#include <Wire.h>
#include <SPI.h>
#include <dht.h>
#include <PWMServo.h>
#include <BtMqttSn.h>
#include <MPU6050_6Axis_MotionApps20.h>

//-----

#define TEAM_ID 4

//-----

#define xstr(s) str(s)
#define str(s) #s

#define CLIENT_NODE_ID (TEAM_ID)
#define CLIENT_ID "Geek-Team-" xstr(TEAM_ID)
#define TOPIC_BASE "GG/Iot/" xstr(TEAM_ID) "/"

#define TOPIC_POTI TOPIC_BASE "Poti"
#define TOPIC_PHOTO TOPIC_BASE "Photo"
#define TOPIC_TEMPERATURE TOPIC_BASE "Temperature"
#define TOPIC_HUMIDITY TOPIC_BASE "Humidity"
#define TOPIC_BUTTON_A TOPIC_BASE "Button/A"
#define TOPIC_BUTTON_B TOPIC_BASE "Button/B"
#define TOPIC_GYRO_YAW TOPIC_BASE "Gyro/Yaw"
#define TOPIC_GYRO_PITCH TOPIC_BASE "Gyro/Pitch"
#define TOPIC_GYRO_ROLL TOPIC_BASE "Gyro/Roll"

#define TOPIC_BUZZER TOPIC_BASE "Buzzer"
#define TOPIC_SERVO TOPIC_BASE "Servo"
#define TOPIC_RGB_LED TOPIC_BASE "RgbLed"

#define GATEWAY_NODE_ID 0

#define RETRY_CONNECT_DELAY 10000

#define CHIP_ENABLE 7
#define CHIP_SELECT 8
#define RF_CHANNEL 50

#define DHT22_PIN 4

#define PIN_POTI 0
#define PIN_PHOTO 1

#define PIN_BUZZER 10
#define PIN_SERVO 9

#define PIN_RGB_RED 3
#define PIN_RGB_GREEN 5
#define PIN_RGB_BLUE 6

#define RGB_BLACK 0x000000
#define RGB_DARK_ORANGE 0xFF8C00
#define RGB_RED 0x0F0000
#define RGB_GREEN 0x000F00

//-----
char sConvertbuffer[10] = {0};

//-----


template<typename T, int N> inline size_t sizeOfArray(const T (&pArray)[N]) {
    return N;
}

//-----

class DigitalInputPublisher {
    public:
        DigitalInputPublisher(uint8_t iPin, const char* iTopic)
        : mPin(iPin), mTopic(iTopic), mState(false) {
        }

        void begin() {
            pinMode(mPin, INPUT_PULLUP);
        }

        void publish(MqttSnClient& iClient) {
            bool state = digitalRead(mPin) == LOW;
            if(state != mState) {
                mState = state;
                iClient.publish(mTopic, state ? "1" : "0");
            }
        }

    private:
        uint8_t mPin;
        const char* mTopic;
        bool mState;

};

//-----

class AnalogInputPublisher {


    public:

    AnalogInputPublisher(uint8_t iPin, const char* iTopic, int iThreshold)
    : mPin(iPin), mTopic(iTopic), mThreshold(iThreshold), mLastPublishedValue(0) {
    }

    void publish(MqttSnClient& iClient, bool iForcePublish)
    {
        int value = analogRead(mPin);
        if(iForcePublish || abs(value-mLastPublishedValue) > mThreshold) {
          mLastPublishedValue = value;
          iClient.publish(mTopic, itoa(value,sConvertbuffer,10));;
        }

    }

    private:
    uint8_t mPin;
    const char* mTopic;
    int mThreshold;
    int mLastPublishedValue;
};

//-----

class MpuPublisher {
    public:

        MpuPublisher()
        : mThreshold(1.0 * M_PI / 180) {
        }

        void begin() {
            Wire.begin();
            TWBR = 24;
            mMpu.initialize();
            uint8_t devStatus = mMpu.dmpInitialize();

            mMpu.setXGyroOffset(220);
            mMpu.setYGyroOffset(76);
            mMpu.setZGyroOffset(-85);
            mMpu.setZAccelOffset(1688);

            if (devStatus == 0) {
                mMpu.setDMPEnabled(true);
                mPacketSize = mMpu.dmpGetFIFOPacketSize();
            } else {
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                Serial << F("MPU failure : ")  << devStatus << endl;
            }
        }

        void loop(){
            uint8_t mpuIntStatus = mMpu.getIntStatus();
            uint16_t fifoCount = mMpu.getFIFOCount();

            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                Serial << F("MPU reset fifo") << endl;
                mMpu.resetFIFO();
            } else if (mpuIntStatus & 0x02) {
                // wait for correct available data length
                while (fifoCount < mPacketSize) fifoCount = mMpu.getFIFOCount();

                mMpu.getFIFOBytes(mFifoBuffer, mPacketSize);

                Quaternion q;
                VectorFloat gravity;
                mMpu.dmpGetQuaternion(&q, mFifoBuffer);
                mMpu.dmpGetGravity(&gravity, &q);
                mMpu.dmpGetYawPitchRoll(mYawPitchRoll, &q, &gravity);
            }
        }

        void publish(MqttSnClient& iClient, bool iForcePublish)
        {
            const char* topics[] = {TOPIC_GYRO_YAW,TOPIC_GYRO_PITCH, TOPIC_GYRO_ROLL};
            for (int i = 0 ; i < sizeOfArray(mYawPitchRoll) ; i++ ) {
                if(iForcePublish || (abs(mYawPitchRoll[i]-mLastPublishedYawPitchRoll[i]) > mThreshold) ) {
                    mLastPublishedYawPitchRoll[i] = mYawPitchRoll[i];
                    iClient.publish(topics[i], dtostrf(mYawPitchRoll[i] * 180/M_PI,4,2,sConvertbuffer));
                }
            }
        }

    private:
        MPU6050 mMpu;
        uint8_t mFifoBuffer[64];
        uint16_t mPacketSize;
        float mThreshold;
        float mYawPitchRoll[3];
        float mLastPublishedYawPitchRoll[3];
};

//-----

MqttSnClient client;
dht DHT;
unsigned long lastConnect;

DigitalInputPublisher sButtons[] = {
    DigitalInputPublisher(A2, TOPIC_BUTTON_A),
    DigitalInputPublisher (A3, TOPIC_BUTTON_B)
};

AnalogInputPublisher sAnalogSensors[] = {
    AnalogInputPublisher(PIN_POTI,TOPIC_POTI,5),
    AnalogInputPublisher(PIN_PHOTO,TOPIC_PHOTO,10)
};

MpuPublisher sMpuPublisher;

PWMServo sServo;


void setup() {

    Serial.begin(115200);
    //Serial << endl << endl << endl << F("*** Geek Gathering IoT Shield ***") << endl;
    Serial << F(" - Team ID                   = ") << TEAM_ID << endl;

    pinMode(PIN_BUZZER, OUTPUT);
    analogWrite(PIN_BUZZER, 0);

    pinMode(PIN_RGB_RED, OUTPUT);
    pinMode(PIN_RGB_GREEN, OUTPUT);
    pinMode(PIN_RGB_BLUE, OUTPUT);
    setRgbLed(RGB_DARK_ORANGE);


    for(int i = 0 ; i < sizeOfArray(sButtons) ; ++i) {
        sButtons[i].begin();
    }

    client.begin(CHIP_ENABLE, CHIP_SELECT, CLIENT_NODE_ID, GATEWAY_NODE_ID, CLIENT_ID, RF_CHANNEL, callback);
    analogWrite(PIN_BUZZER, 0);

    sMpuPublisher.begin();

    sServo.attach(SERVO_PIN_A);

    Serial << F("Try connect ...") << endl;
    connect();

    unsigned long now = millis();
    lastConnect = now;

}

void loop() {
    if(client.loop()) {
        sMpuPublisher.loop();
        publish();
    }
    else {
        setRgbLed(RGB_RED);
        Serial << F("Connection lost try reconnect ...") << endl;
        connect();
        lastConnect = millis();
    }
}


void connect() {
    while (!client.connect()) {
        setRgbLed(RGB_RED);
        Serial << F("... connect failed, reset client ... ") << endl;
        client.end();
        Serial << F("... and retry connect after delay @ ") <<  (millis() + RETRY_CONNECT_DELAY) << F(" ...") << endl;
        client.begin(CHIP_ENABLE, CHIP_SELECT, CLIENT_NODE_ID, GATEWAY_NODE_ID, CLIENT_ID, RF_CHANNEL, callback);
        analogWrite(PIN_BUZZER, 0);
        delay(RETRY_CONNECT_DELAY);
        Serial << F("... retry connect ...") << endl;
    }
    setRgbLed(RGB_GREEN);
    Serial << F("... connected") << endl;
    client.subscribe(TOPIC_BUZZER);
    client.subscribe(TOPIC_SERVO);
    client.subscribe(TOPIC_RGB_LED);
}


void publish() {

    static unsigned long sNextMediumLoop = 0;
    static unsigned long sNextSlowLoop = 0;
    unsigned long now = millis();
    bool mediumLoop = sNextMediumLoop < now;
    bool slowLoop = sNextSlowLoop < now;

    for(int i = 0 ; i < sizeOfArray(sButtons) ; ++i) {
        sButtons[i].publish(client);
    }

    if(mediumLoop || slowLoop) {
        sNextMediumLoop = now + 100;
        for(int i = 0 ; i < sizeOfArray(sAnalogSensors) ; ++i) {
            sAnalogSensors[i].publish(client,slowLoop);
        }
        sMpuPublisher.publish(client,slowLoop);
    }

    if(slowLoop) {
        sNextSlowLoop = now + 3000;
        publishDth();
    }
}

void publishDth() {
    int answer = DHT.read22(DHT22_PIN);
    if(answer != DHTLIB_OK) {
        switch (answer)
        {
            case DHTLIB_ERROR_CHECKSUM:  Serial << F("DHT Checksum error") << endl; return;
            case DHTLIB_ERROR_TIMEOUT :  Serial << F("DHT Time out error") << endl; return;
        }
    }
    //Serial << "publishDth : " << TOPIC_TEMPERATURE << " : " << DHT.temperature << endl;
    //Serial << "publishDth : " << TOPIC_HUMIDITY << " : " << DHT.humidity << endl;

    client.publish(TOPIC_TEMPERATURE, dtostrf(DHT.temperature,4,2,sConvertbuffer));
    client.publish(TOPIC_HUMIDITY, dtostrf(DHT.humidity,4,2,sConvertbuffer));
}


void callback(const char* iTopic, const char* iData) {
    //Serial << "callback : " << iTopic << " : " << iData << endl;
    if(strcmp(iTopic, TOPIC_BUZZER)==0) {
        handleBuzzer(iData);
    }
    if(strcmp(iTopic, TOPIC_SERVO)==0) {
        handleServo(iData);
    }
    if(strcmp(iTopic, TOPIC_RGB_LED)==0) {
        handleRgbLed(iData);
    }
}

void handleBuzzer(const char* iData) {
    int volume = atoi(iData);
    //Serial << "Buzzer to = " << volume << endl;
    analogWrite(PIN_BUZZER, volume);
}


void handleServo(const char* iData) {
    int position = atoi(iData);
    //Serial << "Servo to = " << position << endl;
    sServo.write(position);
    delay(15);
}

void handleRgbLed(const char* iData) {
    //Serial << "RGB to = " << iData << endl;
    if(strlen(iData) != 7 ) {
        //Serial << "Invalid (!=7) hex color: " << iData << endl;
        return;
    }
    if(iData[0] != '#' ) {
        //Serial << "Invalid ([0]!='#') hex color: " << iData << endl;
        return;
    }
    long colors = strtol(iData+1,NULL,16);
    setRgbLed(colors);
}

void setRgbLed(long colors) {
  analogWrite(PIN_RGB_BLUE, 0xFF - (colors & 0xff));
  colors = colors >> 8;
  analogWrite(PIN_RGB_GREEN, 0xFF - (colors & 0xff));
  colors = colors >> 8;
  analogWrite(PIN_RGB_RED, 0xFF -  (colors & 0xff));
}
