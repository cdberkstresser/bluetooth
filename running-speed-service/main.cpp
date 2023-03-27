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
#include <cmath>


// DISTANCE TRAVELLED IN ON RPM OF THE TREADMILL IN MILLIMETERS
#define DISTANCE_PER_REVOLUTION 133

int main(int argc, char *argv[])
{

    // GPI BCM PIN 2
    const int WHEEL_SENSOR_PIN = 8;
    // CYCLE INTERVAL IN MILLISECONDS
    const int TIMER_REPORTING_INTERVAL = 1;
    // MAXIMUM TIME ALLOWED TO MANUALLY TRIP A NOTIFICATION UPDATE IN MILLISECONDS
    const int MAX_REPORTING_INTERVAL = 1000;
    // SET UP WIRING PI CONNECTION
    wiringPiSetup();

    QCoreApplication app(argc, argv);

    // SET UP ADVERTISING SERVICE FOR THE SPEED SENSOR
    QLowEnergyAdvertisingData advertisingData;
    advertisingData.setDiscoverability(QLowEnergyAdvertisingData::DiscoverabilityGeneral);
    advertisingData.setIncludePowerLevel(true);
    advertisingData.setLocalName("Golds410");
    advertisingData.setServices(QList<QBluetoothUuid>() << QBluetoothUuid::RunningSpeedAndCadence);

    // SET UP CHARACTERISTIC DATA FOR SPEED MEASUREMENT
    QLowEnergyCharacteristicData rscMeasurementData;
    rscMeasurementData.setUuid(QBluetoothUuid::RSCMeasurement);
    rscMeasurementData.setValue(QByteArray(5, 0));
    rscMeasurementData.setProperties(QLowEnergyCharacteristic::Notify);
    const QLowEnergyDescriptorData clientConfig(QBluetoothUuid::ClientCharacteristicConfiguration,
                                                QByteArray(2, 0));
    rscMeasurementData.addDescriptor(clientConfig);

    // SET UP CHARACTERISTIC DATA FOR WEEL REVOLUTION DATA
    QLowEnergyCharacteristicData rscDescriptionData;
    rscDescriptionData.setUuid(QBluetoothUuid::RSCFeature);
    QByteArray rscDescriptionBytes;
    // first value is the "Instantaneous Stride Length Present" flag
    rscDescriptionBytes.append(char(0));
    // second value is the "Total Distance Present" flag
    rscDescriptionBytes.append(char(0));
    // third value is the "Walking or Running Status" bits" flag (0 = walking, 1 = running)
    // rscDescriptionBytes.append(char(0));
    rscDescriptionData.setValue(rscDescriptionBytes);
    rscDescriptionData.setProperties(QLowEnergyCharacteristic::Read);

    // NOW COLLECT ALL CHARACTERISTICS AND ATTACH TO SERVICE
    QLowEnergyServiceData serviceData;
    serviceData.setType(QLowEnergyServiceData::ServiceTypePrimary);
    serviceData.setUuid(QBluetoothUuid::RunningSpeedAndCadence);
    serviceData.addCharacteristic(rscMeasurementData);
    serviceData.addCharacteristic(rscDescriptionData);

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
    unsigned int numberOfRevolutionsSinceLastReporting = 0;
    int wheelSensorValue = lastWheelValue;
    unsigned short lowWheelMilliBit = 1;
    unsigned short highWheelMilliBit = 0;
    auto lastReportingTimeInMillis = std::chrono::system_clock::now();
    const auto cyclingServiceProvider = [&service, &startMillis, &wheelSensorValue, &numberOfRevolutionsSinceLastReporting, &lastWheelValue, &lastReportingTimeInMillis, &lowWheelMilliBit, &highWheelMilliBit]()
    {
        auto currentMillis = std::chrono::system_clock::now();
        unsigned long millisecondsElapsedSinceLastReporting = std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - lastReportingTimeInMillis).count();

        QByteArray value;

        if (wheelSensorValue == HIGH && lastWheelValue != HIGH)
        {
            numberOfRevolutionsSinceLastReporting += 1;
            lastWheelValue = HIGH;
        }
        else
        {
            lastWheelValue = wheelSensorValue;
        }
        wheelSensorValue = digitalRead(WHEEL_SENSOR_PIN);

        QLowEnergyCharacteristic characteristic = service->characteristic(QBluetoothUuid::RSCMeasurement);
        Q_ASSERT(characteristic.isValid());

        // REPORT IF IT HAS BEEN MORE THAN ONE SECOND OR IF THE REVOLUTIONS JUST INCREASED
        if (millisecondsElapsedSinceLastReporting > MAX_REPORTING_INTERVAL || numberOfRevolutionsSinceLastReporting == 10)
        {
            double wholeMetersPerSecond;
            double fractionalMetersPerSecond = modf(double(numberOfRevolutionsSinceLastReporting) * DISTANCE_PER_REVOLUTION / millisecondsElapsedSinceLastReporting, &wholeMetersPerSecond);

            // ************************************
            // WHEEL REVOLUTION DATA
            value.append(char(0));   // no idea what this represents - spacer
            value.append(char(short(fractionalMetersPerSecond * 256))); // speed - m/s - fractional portion * 256
            value.append(char(short(wholeMetersPerSecond)));    // speed m/s
            value.append(char(0));   // cadence
            lastReportingTimeInMillis = currentMillis;
            numberOfRevolutionsSinceLastReporting = 0;
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
