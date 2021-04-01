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
    // GPI BCM PIN 3 ACROSS FROM GROUND
    const int CRANK_SENSOR_PIN = 9;
    // CYCLE INTERVAL IN MILLISECONDS
    const int TIMER_REPORTING_INTERVAL = 2;
    // MAXIMUM TIME ALLOWED TO MANUALLY TRIP A NOTIFICATION UPDATE IN MILLISECONDS
    const int MAX_REPORTING_INTERVAL = 250;
    // SET UP WIRING PI CONNECTION
    wiringPiSetup();

    QCoreApplication app(argc, argv);

    // SET UP ADVERTISING SERVICE FOR THE SPEED SENSOR
    QLowEnergyAdvertisingData advertisingData;
    advertisingData.setDiscoverability(QLowEnergyAdvertisingData::DiscoverabilityGeneral);
    advertisingData.setIncludePowerLevel(true);
    advertisingData.setLocalName("Berkstresser_Running");
    advertisingData.setServices(QList<QBluetoothUuid>() << QBluetoothUuid::RunningSpeedAndCadence);

    // SET UP CHARACTERISTIC DATA FOR SPEED MEASUREMENT
    QLowEnergyCharacteristicData rscMeasurementData;
    rscMeasurementData.setUuid(QBluetoothUuid::RSCMeasurement);
    rscMeasurementData.setValue(QByteArray(2, 0));
    rscMeasurementData.setProperties(QLowEnergyCharacteristic::Notify);
    const QLowEnergyDescriptorData clientConfig(QBluetoothUuid::ClientCharacteristicConfiguration,
                                                QByteArray(2, 0));
    rscMeasurementData.addDescriptor(clientConfig);

    // SET UP CHARACTERISTIC DATA FOR WEEL REVOLUTION DATA
    QLowEnergyCharacteristicData rscDescriptionData;
    rscDescriptionData.setUuid(QBluetoothUuid::RSCFeature);
    QByteArray rscDescriptionBytes;
    // first value is the "Wheel Revolution Data Present" flag
    rscDescriptionBytes.append(char(0));
    // second value is the "Crank Revolution Data Present" flag
    rscDescriptionBytes.append(char(0));
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
    pinMode(CRANK_SENSOR_PIN, INPUT);

    int lastCrankValue = digitalRead(CRANK_SENSOR_PIN);
    unsigned int numberOfRevolutions = 0;
    unsigned int numberOfCranks = 0;
    int crankSensorValue = lastCrankValue;
    unsigned short lowCrankMilliBit = 1;
    unsigned short highCrankMilliBit = 0;
    auto lastReportingTimeInMillis = std::chrono::system_clock::now();
    const auto cyclingServiceProvider = [&service, &startMillis, &crankSensorValue, &numberOfRevolutions, &numberOfCranks, &lastCrankValue, &lastReportingTimeInMillis, &lowCrankMilliBit, &highCrankMilliBit]() {
        auto currentMillis = std::chrono::system_clock::now();
        unsigned long millisecondsElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - startMillis).count();
        unsigned short lowCrankBit = numberOfCranks % 256;
        unsigned short highCrankBit = numberOfCranks / 256 % 256; // make sure it isn't over 255 and just let it overflow
        unsigned long millisecondsElapsedSinceLastReporting = std::chrono::duration_cast<std::chrono::milliseconds>(currentMillis - lastReportingTimeInMillis).count();
        QByteArray value;

        if (crankSensorValue == HIGH && lastCrankValue != HIGH)
        {
            numberOfCranks += 1;
            lastCrankValue = HIGH;
            lowCrankMilliBit = millisecondsElapsed % 256;
            highCrankMilliBit = millisecondsElapsed / 256 % 256; // make sure it isn't over 255 and just let it overflow
        }
        else
        {
            lastCrankValue = crankSensorValue;
        }
        crankSensorValue = digitalRead(CRANK_SENSOR_PIN);
        value.append(char(0));                 // required for rsc data 1=wheel, 2=crank, 3=both
                                               // ************************************
//                                               // WHEEL REVOLUTION DATA uint32
//        value.append(char(lowRevBit));         // low bit of revolutions
//        value.append(char(midRevBit));         // mid1 bit of revolutions
//        value.append(char(highRevBit));        // mid2 bit of revolutions
//        value.append(char(0));                 // high bit of revolutions.  Not bothering
//                                               // ************************************
//                                               // WHEEL TIME DATA uint16
//        value.append(char(lowWheelMilliBit));  // low bit of milliseconds
//        value.append(char(highWheelMilliBit)); // high bit of milliseconds
                                               // ************************************
                                               // CRANK REVOLUTION DATA uint16
//        value.append(char(lowCrankBit));       // low bit of revolutions
//        value.append(char(highCrankBit));      // mid1 bit of revolutions
                                               // ************************************
                                               // CRANK TIME DATA uint16
//        value.append(char(lowCrankMilliBit));  // low bit of milliseconds
//        value.append(char(highCrankMilliBit)); // high bit of milliseconds
                                               // ************************************
				               // INSTANTANEOUS SPEED uint16 
	value.append(char(83));		       // low bit of m/s 
	value.append(char(1));		       // high bit of m/s 
        				       // INSTANTANEOUS CADENCE uint8
        value.append(char(0));    	       // value in steps per minute
		
	QLowEnergyCharacteristic characteristic = service->characteristic(QBluetoothUuid::RSCMeasurement);
        Q_ASSERT(characteristic.isValid());

        // REPORT IF IT HAS BEEN MORE THAN ONE SECOND OR IF THE REVOLUTIONS JUST INCREASED
        if (millisecondsElapsedSinceLastReporting > MAX_REPORTING_INTERVAL)
        {
            lastReportingTimeInMillis = currentMillis;
            service->writeCharacteristic(characteristic, value); // Do the notify.
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

