#include <I2Cdev.h>
#include <Wire.h>
#include <SPI.h>
#include <dht.h>
#include <PWMServo.h>
#include <BtMqttSn.h>
#include <MPU6050_6Axis_MotionApps20.h>


//-----

#define TEAM_ID 5

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
#define TOPIC_GYRO_ROW TOPIC_BASE "Gyro/Row"

#define TOPIC_BUZZER TOPIC_BASE "Buzzer"
#define TOPIC_SERVO TOPIC_BASE "Servo"
#define TOPIC_RGB_LED TOPIC_BASE "RgbLed"

#define GATEWAY_NODE_ID 0

#define DHT22_READ_INTERVAL 5000
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


//-----
char sConvertbuffer[20] = {0};
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
                iClient.publish(mTopic, state ? "ON" : "OFF");
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
    : mPin(iPin), mTopic(iTopic), mThreshold(iThreshold), mLastValue(0) {
    }

    void publish(MqttSnClient& iClient, bool iOnlyWhenChanged)
    {
        int value = analogRead(mPin);
        if(iOnlyWhenChanged && abs(value-mLastValue) < mThreshold) {
            return;
        }
        mLastValue = value;
        iClient.publish(mTopic, itoa(value,sConvertbuffer,10));
    }

    private:
    uint8_t mPin;
    const char* mTopic;
    int mThreshold;
    int mLastValue;
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

PWMServo sServo;

MPU6050 sMpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void setup() {

    Wire.begin();
    TWBR = 24;

    Serial.begin(115200);
    //Serial << endl << endl << endl << F("*** Geek Gathering IoT Shield ***") << endl;
    Serial << F(" - Team ID                   = ") << TEAM_ID << endl;



    pinMode(PIN_BUZZER, OUTPUT);
    analogWrite(PIN_BUZZER, 0);

    pinMode(PIN_RGB_RED, OUTPUT);
    analogWrite(PIN_RGB_RED, 0xFF);
    pinMode(PIN_RGB_GREEN, OUTPUT);
    analogWrite(PIN_RGB_GREEN, 0xFF);
    pinMode(PIN_RGB_BLUE, OUTPUT);
    analogWrite(PIN_RGB_BLUE, 0xFF);


    for(int i = 0 ; i < sizeOfArray(sButtons) ; ++i) {
        sButtons[i].begin();
    }



    client.begin(CHIP_ENABLE, CHIP_SELECT, CLIENT_NODE_ID, GATEWAY_NODE_ID, CLIENT_ID, RF_CHANNEL, callback);
    analogWrite(PIN_BUZZER, 0);

    sServo.attach(SERVO_PIN_A);


    sMpu.initialize();
    //Serial << F("Initializing DMP...") << endl;
    devStatus = sMpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    sMpu.setXGyroOffset(220);
    sMpu.setYGyroOffset(76);
    sMpu.setZGyroOffset(-85);
    sMpu.setZAccelOffset(1688); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        sMpu.setDMPEnabled(true);

        mpuIntStatus = sMpu.getIntStatus();
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = sMpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial << F("DMP failed :")  << devStatus << endl;
    }

    //Serial << F("Try connect ...") << endl;
    connect();

    unsigned long now = millis();
    lastConnect = now;

}

void loop() {
    if(client.loop()) {
        publish();
    }
    else {
        Serial << F("Connection lost try reconnect ...") << endl;
        connect();
        lastConnect = millis();
    }
}


void connect() {
    while (!client.connect()) {
        Serial << F("... connect failed, reset client ... ") << endl;
        client.end();
        Serial << F("... and retry connect after delay @ ") <<  (millis() + RETRY_CONNECT_DELAY) << F(" ...") << endl;
        client.begin(CHIP_ENABLE, CHIP_SELECT, CLIENT_NODE_ID, GATEWAY_NODE_ID, CLIENT_ID, RF_CHANNEL, callback);
        analogWrite(PIN_BUZZER, 0);
        Serial << F("... retry connect ...") << endl;
    }
    Serial << F("... connected") << endl;
    //client.subscribe(TOPIC_MILLIS);
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

    readMpu();


    for(int i = 0 ; i < sizeOfArray(sButtons) ; ++i) {
        sButtons[i].publish(client);
    }

    if(mediumLoop || slowLoop) {
        sNextMediumLoop = now + 100;
        for(int i = 0 ; i < sizeOfArray(sAnalogSensors) ; ++i) {
            sAnalogSensors[i].publish(client,!slowLoop);
        }
    }

    if(slowLoop) {
        sNextSlowLoop = now + 3000;
        publishDth();
        publishMpu();
    }
}

void readMpu() {
    // reset interrupt flag and get INT_STATUS byte
    mpuIntStatus = sMpu.getIntStatus();

    // get current FIFO count
    fifoCount = sMpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        sMpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = sMpu.getFIFOCount();

        // read a packet from FIFO
        sMpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        sMpu.dmpGetQuaternion(&q, fifoBuffer);
        sMpu.dmpGetGravity(&gravity, &q);
        sMpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

void publishMpu() {
    client.publish(TOPIC_GYRO_YAW, dtostrf(ypr[0] * 180/M_PI,4,2,sConvertbuffer));
    client.publish(TOPIC_GYRO_PITCH, dtostrf(ypr[1] * 180/M_PI,4,2,sConvertbuffer));
    client.publish(TOPIC_GYRO_ROW, dtostrf(ypr[2] * 180/M_PI,4,2,sConvertbuffer));
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
    analogWrite(PIN_RGB_BLUE, 0xFF - (colors & 0xff));
    colors = colors >> 8;
    analogWrite(PIN_RGB_GREEN, 0xFF - (colors & 0xff));
    colors = colors >> 8;
    analogWrite(PIN_RGB_RED, 0xFF -  (colors & 0xff));
}
