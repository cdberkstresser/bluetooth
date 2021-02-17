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
    const int SENSOR_PIN = 8;
    // CYCLE INTERVAL IN MILLISECONDS
    const int TIMER_REPORTING_INTERVAL = 5;
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
    const char cscLocationBytes[] = {0x06};
    cscLocationData.setValue(QByteArray(cscLocationBytes, 1));
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
    pinMode(SENSOR_PIN, INPUT);

    int lastValue = digitalRead(SENSOR_PIN);
    unsigned long numberOfRevolutions = 0;
    int sensorValue;
    auto lastReportingTimeInMillis = std::chrono::system_clock::now();
    const auto cyclingServiceProvider = [&service, &startMillis, &sensorValue, &numberOfRevolutions, &lastValue, &lastReportingTimeInMillis]() {
        int revsJustChangedFlag = 0;
        auto currentMillis = std::chrono::system_clock::now();
        unsigned long millisecondsElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - startMillis).count();
        unsigned short lowMilliBit = millisecondsElapsed % 256;
        unsigned short highMilliBit = millisecondsElapsed / 256 % 256;   // make sure it isn't over 255 and just let it overflow
        unsigned short lowRevBit = numberOfRevolutions % 256;
        unsigned short midRevBit = numberOfRevolutions / 256 % 256;
        unsigned short highRevBit = numberOfRevolutions / (256 * 256);
        unsigned long millisecondsElapsedSinceLastReporting = std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - lastReportingTimeInMillis).count();
        QByteArray value;

        if (sensorValue == HIGH && lastValue != HIGH)
        {
            numberOfRevolutions += 1;
            revsJustChangedFlag = 1;
            lastValue = HIGH;
        }
        else
        {
            lastValue = sensorValue;
        }
        sensorValue = digitalRead(SENSOR_PIN);
        value.append(char(1));            // required for csc data
        value.append(char(lowRevBit));    // low bit of revolutions
        value.append(char(midRevBit));    // mid1 bit of revolutions
        value.append(char(highRevBit));   // mid2 bit of revolutions
        value.append(char(0));            // high bit of revolutions.  Not bothering
        value.append(char(lowMilliBit));  // low bit of milliseconds
        value.append(char(highMilliBit)); // high bit of milliseconds
        QLowEnergyCharacteristic characteristic = service->characteristic(QBluetoothUuid::CSCMeasurement);
        Q_ASSERT(characteristic.isValid());

        // REPORT IF IT HAS BEEN MORE THAN ONE SECOND OR IF THE REVOLUTIONS JUST INCREASED
        if (millisecondsElapsedSinceLastReporting > 1000 || revsJustChangedFlag == 1)
        {
            lastReportingTimeInMillis = currentMillis;
            service->writeCharacteristic(characteristic, value); // Potentially causes notification.
        }
    };
    QObject::connect(&cyclingServiceLoop, &QTimer::timeout, cyclingServiceProvider);
    cyclingServiceLoop.start(TIMER_REPORTING_INTERVAL);

    auto reconnect = [&leController, advertisingData, &service, serviceData]() {
        service.reset(leController->addService(serviceData));
        if (!service.isNull())
            leController->startAdvertising(QLowEnergyAdvertisingParameters(),
                                           advertisingData, advertisingData);
    };
    QObject::connect(leController.data(), &QLowEnergyController::disconnected, reconnect);

    return app.exec();
}