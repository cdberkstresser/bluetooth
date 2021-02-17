TEMPLATE = app
TARGET = csc-speed-service.bin

QT = core bluetooth
CONFIG += c++11

SOURCES += main.cpp

LIBS += -lwiringPi
target.path = .
INSTALLS += target
