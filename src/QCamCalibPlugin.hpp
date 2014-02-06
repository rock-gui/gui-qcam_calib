#ifndef QCAMCALIBPLUGIN_HPP
#define QCAMCALIBPLUGIN_HPP

#include <QtGui>
#include <QtDesigner/QDesignerCustomWidgetInterface>

class QCamCalibPlugin : public QObject, public QDesignerCustomWidgetInterface
{
    Q_OBJECT
    Q_INTERFACES(QDesignerCustomWidgetInterface)

public:
    QCamCalibPlugin(QObject *parent = 0);
    virtual ~QCamCalibPlugin();

    bool isContainer() const;
    bool isInitialized() const;
    QIcon icon() const;
    QString domXml() const;
    QString group() const;
    QString includeFile() const;
    QString name() const;
    QString toolTip() const;
    QString whatsThis() const;
    QWidget* createWidget(QWidget *parent);
    void initialize(QDesignerFormEditorInterface *core);

private:
    bool initialized; 
};

#endif /* QCAMCALIBPLUGIN_HPP */  
