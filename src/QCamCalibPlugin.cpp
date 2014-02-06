#include "QCamCalibPlugin.hpp"
#include "QCamCalib.hpp"

Q_EXPORT_PLUGIN2(QCamCalib, QCamCalibPlugin)

QCamCalibPlugin::QCamCalibPlugin(QObject *parent)
    : QObject(parent)
{
    initialized = false;
}

QCamCalibPlugin::~QCamCalibPlugin()
{
}

bool QCamCalibPlugin::isContainer() const
{
    return false;
}

bool QCamCalibPlugin::isInitialized() const
{
    return initialized;
}

QIcon QCamCalibPlugin::icon() const
{
    return QIcon("");
}

QString QCamCalibPlugin::domXml() const
{
        return "<ui language=\"c++\">\n"
            " <widget class=\"QCamCalib\" name=\"qcamcalib\">\n"
            "  <property name=\"geometry\">\n"
            "   <rect>\n"
            "    <x>0</x>\n"
            "    <y>0</y>\n"
            "     <width>300</width>\n"
            "     <height>120</height>\n"
            "   </rect>\n"
            "  </property>\n"
//            "  <property name=\"toolTip\" >\n"
//            "   <string>QCamCalib</string>\n"
//            "  </property>\n"
//            "  <property name=\"whatsThis\" >\n"
//            "   <string>QCamCalib</string>\n"
//            "  </property>\n"
            " </widget>\n"
            "</ui>\n";
}

QString QCamCalibPlugin::group() const {
    return "Rock-Robotics";
}

QString QCamCalibPlugin::includeFile() const {
    return "QCamCalib/QCamCalib.hpp";
}

QString QCamCalibPlugin::name() const {
    return "QCamCalib";
}

QString QCamCalibPlugin::toolTip() const {
    return whatsThis();
}

QString QCamCalibPlugin::whatsThis() const
{
    return "";
}

QWidget* QCamCalibPlugin::createWidget(QWidget *parent)
{
    return new QCamCalib(parent);
}

void QCamCalibPlugin::initialize(QDesignerFormEditorInterface *core)
{
     if (initialized)
         return;
     initialized = true;
}
