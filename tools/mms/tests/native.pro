QT += core gui widgets opengl openglwidgets xml testlib
CONFIG += c++11 console testcase release
CONFIG -= debug debug_and_release
TARGET = native_tests
DESTDIR = release
TEMPLATE = app
MMS_ROOT = $$clean_path($$PWD/..)
INCLUDEPATH += $$MMS_ROOT/source/src
SOURCES += native_tests.cpp
APP_OBJECTS = $$files($$MMS_ROOT/build/release-obj/*.o, true)
for(object, APP_OBJECTS) {
    !contains(object, .*Main\.o$): LIBS += $$object
}
