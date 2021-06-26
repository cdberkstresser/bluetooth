#include <QtBluetooth/qlowenergyadvertisingdata.h>
#include <QtBluetooth/qlowenergyadvertisingparameters.h>
#include <QtBluetooth/qlowenergycharacteristic.h>
#include <QtBluetooth/qlowenergycharacteristicdata.h>
#include <QtBluetooth/qlowenergydescriptordata.h>
#include <QtBluetooth/qlowenergycontroller.h>
#include <QtBluetooth/qlowenergyservice.h>
#include <QtBluetooth/qlowenergyservicedata.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qcoreapplication.h>
#include <QtCore/qlist.h>
#include <QtCore/qloggingcategory.h>
#include <QtCore/qscopedpointer.h>
#include <QtCore/qtimer.h>
#include <chrono>
#include <wiringPi.h>

int main(int argc, char *argv[])
{
    // GPI BCM PIN 2
    const int WHEEL_SENSOR_PIN = 8;
    // CYCLE INTERVAL IN MILLISECONDS
    const int TIMER_REPORTING_INTERVAL = 2;
    // SET UP WIRING PI CONNECTION
    wiringPiSetup();

    QCoreApplication app(argc, argv);

    // SET UP ADVERTISING SERVICE FOR THE SPEED SENSOR
    QLowEnergyAdvertisingData advertisingData;
    advertisingData.setDiscoverability(QLowEnergyAdvertisingData::DiscoverabilityGeneral);
    advertisingData.setIncludePowerLevel(true);
    advertisingData.setLocalName("Berkstresser_Speed");
    advertisingData.setServices(QList<QBluetoothUuid>() << QBluetoothUuid::CyclingSpeedAndCadence);

    // SET UP CHARACTERISTIC DATA FOR SPEED MEASUREMENT
    QLowEnergyCharacteristicData cscMeasurementData;
    cscMeasurementData.setUuid(QBluetoothUuid::CSCMeasurement);
    cscMeasurementData.setValue(QByteArray(2, 0));
    cscMeasurementData.setProperties(QLowEnergyCharacteristic::Notify);
    const QLowEnergyDescriptorData clientConfig(QBluetoothUuid::ClientCharacteristicConfiguration,
                                                QByteArray(2, 0));
    cscMeasurementData.addDescriptor(clientConfig);

    // SET UP CHARACTERISTIC DATA FOR WEEL REVOLUTION DATA
    QLowEnergyCharacteristicData cscDescriptionData;
    cscDescriptionData.setUuid(QBluetoothUuid::CSCFeature);
    QByteArray cscDescriptionBytes;
    // first value is the "Wheel Revolution Data Present" flag
    cscDescriptionBytes.append(char(1));
    // second value is the "Crank Revolution Data Present" flag
    cscDescriptionBytes.append(char(0));
    cscDescriptionData.setValue(cscDescriptionBytes);
    cscDescriptionData.setProperties(QLowEnergyCharacteristic::Read);

    // SET UP SENSOR LOCATION.  USE RIGHT CRANK FOR IT
    QLowEnergyCharacteristicData cscLocationData;
    cscLocationData.setUuid(QBluetoothUuid::SensorLocation);
    QByteArray cscLocationBytes;
    cscLocationBytes.append(char(6));
    cscLocationData.setValue(cscLocationBytes);
    cscLocationData.setProperties(QLowEnergyCharacteristic::Read);

    // NOW COLLECT ALL CHARACTERISTICS AND ATTACH TO SERVICE
    QLowEnergyServiceData serviceData;
    serviceData.setType(QLowEnergyServiceData::ServiceTypePrimary);
    serviceData.setUuid(QBluetoothUuid::CyclingSpeedAndCadence);
    serviceData.addCharacteristic(cscMeasurementData);
    serviceData.addCharacteristic(cscDescriptionData);
    serviceData.addCharacteristic(cscLocationData);

    // START THE ADVERTISING SERVICE
    const QScopedPointer<QLowEnergyController> leController(QLowEnergyController::createPeripheral());
    QScopedPointer<QLowEnergyService> service(leController->addService(serviceData));
    leController->startAdvertising(QLowEnergyAdvertisingParameters(), advertisingData,
                                   advertisingData);

    // DEFINE THE TIMER THAT WILL QUERY THE HALL SENSOR FOR PIN CHANGES
    QTimer cyclingServiceLoop;
    auto startMillis = std::chrono::system_clock::now();
    pinMode(WHEEL_SENSOR_PIN, INPUT);

    int lastWheelValue = digitalRead(WHEEL_SENSOR_PIN);
    unsigned int numberOfRevolutions = 0;
    int wheelSensorValue = lastWheelValue;
    unsigned short lowWheelMilliBit = 1;
    unsigned short highWheelMilliBit = 0;
    auto lastReportingTimeInMillis = std::chrono::system_clock::now();
    const auto cyclingServiceProvider = [&service, &startMillis, &wheelSensorValue, &numberOfRevolutions, &lastWheelValue, &lastReportingTimeInMillis, &lowWheelMilliBit, &highWheelMilliBit]()
    {
        auto currentMillis = std::chrono::system_clock::now();
        unsigned long millisecondsElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - startMillis).count();
        unsigned short lowRevBit = numberOfRevolutions % 256;
        unsigned short midRevBit = numberOfRevolutions / 256 % 256;
        unsigned short highRevBit = numberOfRevolutions / (256 * 256);
        unsigned short sensorChanged = 0;
        QByteArray value;

        if (wheelSensorValue == HIGH && lastWheelValue != HIGH)
        {
            numberOfRevolutions += 1;
            lastWheelValue = HIGH;
            lowWheelMilliBit = millisecondsElapsed % 256;
            highWheelMilliBit = millisecondsElapsed / 256 % 256; // make sure it isn't over 255 and just let it overflow

            sensorChanged = 1;
        }
        else
        {
            lastWheelValue = wheelSensorValue;
            sensorChanged = 0;
        }
        wheelSensorValue = digitalRead(WHEEL_SENSOR_PIN);
        value.append(char(1));                 // required for csc data 1=wheel, 2=crank, 3=both
                                               // ************************************
                                               // WHEEL REVOLUTION DATA uint32
        value.append(char(lowRevBit));         // low bit of revolutions
        value.append(char(midRevBit));         // mid1 bit of revolutions
        value.append(char(highRevBit));        // mid2 bit of revolutions
        value.append(char(0));                 // high bit of revolutions.  Not bothering
                                               // ************************************
                                               // WHEEL TIME DATA uint16
        value.append(char(lowWheelMilliBit));  // low bit of milliseconds
        value.append(char(highWheelMilliBit)); // high bit of milliseconds
                                               // ************************************
        QLowEnergyCharacteristic characteristic = service->characteristic(QBluetoothUuid::CSCMeasurement);
        Q_ASSERT(characteristic.isValid());

        if (sensorChanged)
        {
            service->writeCharacteristic(characteristic, value); // Do the notify.
        }
    };
    QObject::connect(&cyclingServiceLoop, &QTimer::timeout, cyclingServiceProvider);
    cyclingServiceLoop.start(TIMER_REPORTING_INTERVAL);

    auto reconnect = [&leController, advertisingData, &service, serviceData]()
    {
        service.reset(leController->addService(serviceData));
        if (!service.isNull())
            leController->startAdvertising(QLowEnergyAdvertisingParameters(),
                                           advertisingData, advertisingData);
    };
    QObject::connect(leController.data(), &QLowEnergyController::disconnected, reconnect);

    return app.exec();
}
