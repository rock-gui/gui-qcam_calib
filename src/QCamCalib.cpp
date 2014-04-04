#include "QCamCalib.hpp"
#include "Items.hpp"
#include "ImageView.hpp"

#include "ui_main_gui.h"
#include <iostream>
#include <stdexcept>
#include <sstream>

#include <QAction>
#include <QFileDialog>

#include <boost/bind.hpp>

using namespace qcam_calib;
const char* CAMERA_BASE_NAME = "camera_";
const char* STRUCTURED_LIGHT_BASE_NAME = "structure_light_";

QCamCalib::QCamCalib(QWidget *parent) :
    QWidget(parent),
    current_load_path("."),
    tree_model(NULL),
    camera_item_menu(NULL),
    structured_light_item_menu(NULL),
    structured_light_image_item_menu(NULL),
    tree_view_menu(NULL),
    image_item_menu(NULL),
    progress_dialog_images(NULL),
    progress_dialog_chessboard(NULL),
    progress_dialog_calibrate(NULL),
    progress_dialog_laser_line(NULL),
    future_watcher_images(NULL),
    future_watcher_chessboard(NULL),
    future_watcher_calibrate(NULL),
    future_watcher_laser_line(NULL)
{
    Ui::MainGui gui;
    gui.setupUi(this);

    // tree model
    tree_model = new QStandardItemModel(gui.treeView);
    tree_model->setHorizontalHeaderLabels((QStringList() << "Cameras" << "Value"));
    tree_model->setColumnCount(2);
    gui.treeView->setModel(tree_model);
    connect(gui.treeView,SIGNAL(customContextMenuRequested(const QPoint&)),SLOT(contextMenuTreeView(const QPoint&)));
    connect(gui.treeView,SIGNAL(clicked(const QModelIndex&)),SLOT(clickedTreeView(const QModelIndex&)));
    connect(gui.treeView->selectionModel(), SIGNAL(currentChanged(QModelIndex,QModelIndex)), this, SLOT(clickedTreeView(const QModelIndex&)));

    // camera item menu
    camera_item_menu = new QMenu(this);
    QAction* act = new QAction("load images",this);
    connect(act,SIGNAL(triggered()),this,SLOT(loadCameraImages()));
    camera_item_menu->addAction(act);

    QAction *act_find2 = new QAction("find chessboard fastX",this);
    connect(act_find2,SIGNAL(triggered()),this,SLOT(findChessBoardFastX()));
    camera_item_menu->addAction(act_find2);

    QAction *act_find1 = new QAction("find chessboard opencv",this);
    connect(act_find1,SIGNAL(triggered()),this,SLOT(findChessBoard()));
    camera_item_menu->addAction(act_find1);
    camera_item_menu->addSeparator();

    act = new QAction("calibrate",this);
    connect(act,SIGNAL(triggered()),this,SLOT(calibrateCamera()));
    camera_item_menu->addAction(act);
    camera_item_menu->addSeparator();

    act = new QAction("save parameter",this);
    connect(act,SIGNAL(triggered()),this,SLOT(saveCalibParameter()));
    camera_item_menu->addAction(act);
    camera_item_menu->addSeparator();

    QAction *act_remove = new QAction("remove",this);
    connect(act_remove,SIGNAL(triggered()),this,SLOT(removeCurrentItem()));
    camera_item_menu->addAction(act_remove);

    // structured_light_menu
    structured_light_item_menu = new QMenu(this);
    act = new QAction("load images",this);
    connect(act,SIGNAL(triggered()),this,SLOT(loadStructuredLightImages()));
    structured_light_item_menu->addAction(act);
    QAction *act_laser = new QAction("find laser line",this);
    connect(act_laser,SIGNAL(triggered()),this,SLOT(findLaserLine()));
    structured_light_item_menu->addAction(act_laser);
    act = new QAction("calibrate",this);
    connect(act,SIGNAL(triggered()),this,SLOT(calibrateStructuredLight()));
    structured_light_item_menu->addAction(act);
    structured_light_item_menu->addSeparator();
    act = new QAction("save parameter",this);
    connect(act,SIGNAL(triggered()),this,SLOT(saveCalibParameter()));
    structured_light_item_menu->addAction(act);

    // structured_light_image_menu
    structured_light_image_item_menu = new QMenu(this);
    structured_light_image_item_menu->addAction(act_find2);
    structured_light_image_item_menu->addAction(act_find1);
    structured_light_image_item_menu->addAction(act_laser);
    structured_light_image_item_menu->addAction(act_remove);

    // tree view menu
    tree_view_menu = new QMenu(this);
    act = new QAction("add Camera",this);
    connect(act,SIGNAL(triggered()),this,SLOT(addCamera()));
    tree_view_menu->addAction(act);

    act = new QAction("add Structured Light",this);
    connect(act,SIGNAL(triggered()),this,SLOT(addLineStructuredLight()));
    tree_view_menu->addAction(act);

    // image item menu
    image_item_menu = new QMenu(this);
    image_item_menu->addAction(act_remove);
    image_item_menu->addAction(act_find1);
    image_item_menu->addAction(act_find2);

    //graphics view
    image_view = gui.imageView;

    // add initial camera
    addCamera();

    // progress dialog
    progress_dialog_images = new QProgressDialog("loading images","cancel",0,0,this);
    future_watcher_images = new QFutureWatcher<QImage>(this);
    connect(future_watcher_images, SIGNAL(progressValueChanged(int)), progress_dialog_images, SLOT(setValue(int)));
    connect(future_watcher_images, SIGNAL(progressRangeChanged(int, int)), progress_dialog_images, SLOT(setRange(int, int)));
    connect(future_watcher_images, SIGNAL(finished()), progress_dialog_images, SLOT(accept()));

    progress_dialog_chessboard = new QProgressDialog("searching for chessboards","cancel",0,0,this);
    future_watcher_chessboard= new QFutureWatcher<QVector<QPointF> >(this);
    connect(future_watcher_chessboard, SIGNAL(progressValueChanged(int)), progress_dialog_chessboard, SLOT(setValue(int)));
    connect(future_watcher_chessboard, SIGNAL(finished()), progress_dialog_chessboard, SLOT(accept()));
    connect(future_watcher_chessboard, SIGNAL(progressRangeChanged(int, int)), progress_dialog_chessboard, SLOT(setRange(int, int)));

    progress_dialog_calibrate = new QProgressDialog("calibrating camera","cancel",0,0,this);
    progress_dialog_calibrate->setCancelButton(NULL);
    future_watcher_calibrate = new QFutureWatcher<void>(this);
    connect(future_watcher_calibrate, SIGNAL(progressValueChanged(int)), progress_dialog_calibrate, SLOT(setValue(int)));
    connect(future_watcher_calibrate, SIGNAL(finished()), progress_dialog_calibrate, SLOT(accept()));

    progress_dialog_laser_line= new QProgressDialog("searching for laser line","cancel",0,0,this);
    progress_dialog_laser_line->setCancelButton(NULL);
    future_watcher_laser_line= new QFutureWatcher<QVector<QPointF> >(this);
    connect(future_watcher_laser_line, SIGNAL(progressValueChanged(int)), progress_dialog_laser_line, SLOT(setValue(int)));
    connect(future_watcher_laser_line, SIGNAL(finished()), progress_dialog_laser_line, SLOT(accept()));
    connect(future_watcher_laser_line, SIGNAL(progressRangeChanged(int, int)), progress_dialog_laser_line, SLOT(setRange(int, int)));
}

QCamCalib::~QCamCalib()
{
}

void QCamCalib::contextMenuTreeView(const QPoint &point)
{
    QTreeView *tree_view = findChild<QTreeView*>("treeView");
    if(!tree_view)
        throw std::runtime_error("Cannot find treeView object");

    QModelIndex index = tree_view->indexAt(point);
    if(index.isValid())
    {
        QCamCalibItem *item = dynamic_cast<QCamCalibItem*>(tree_model->itemFromIndex(index));
        if(!item)
            return;
        else if(dynamic_cast<StructuredLightItem*>(item))
            structured_light_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if(dynamic_cast<StructuredLightImageItem*>(item))
            structured_light_image_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if(dynamic_cast<CameraItem*>(item))
            camera_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if(dynamic_cast<ImageItem*>(item))
            image_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
    }
    else {
        tree_view_menu->exec(tree_view->viewport()->mapToGlobal(point));
    }
}

void QCamCalib::addCamera(int camera_id)
{
    if(camera_id < 0)
        camera_id = 0;
    for(int i=0;i<=tree_model->rowCount();++i)
    {
        std::stringstream strstr;
        strstr << CAMERA_BASE_NAME << (camera_id+i);
        QModelIndexList list = tree_model->match(tree_model->index(0,0),Qt::DisplayRole,QVariant(strstr.str().c_str()),1,Qt::MatchExactly);
        if(list.empty())
        {
            tree_model->appendRow(new qcam_calib::CameraItem(camera_id+i,QString(strstr.str().c_str())));
            break;
        }
    }
}

void QCamCalib::addLineStructuredLight(int id)
{
    if(id < 0)
        id = 0;
    for(int i=0;i<=tree_model->rowCount();++i)
    {
        std::stringstream strstr;
        strstr << STRUCTURED_LIGHT_BASE_NAME << (id+i);
        QModelIndexList list = tree_model->match(tree_model->index(0,0),Qt::DisplayRole,QVariant(strstr.str().c_str()),1,Qt::MatchExactly);
        if(list.empty())
        {
            tree_model->appendRow(new qcam_calib::StructuredLightItem(id+i,QString(strstr.str().c_str())));
            break;
        }
    }
}

void QCamCalib::saveCalibParameter(int obj_id)
{
    CalibrationObj *obj = getItem<CalibrationObj>(obj_id,true);
    if(!obj->isCalibrated())
    {
        if(dynamic_cast<CameraItem*>(obj))
            calibrateCamera(obj_id);
        else if(dynamic_cast<StructuredLightItem*>(obj))
            calibrateStructuredLight(obj_id);
        if(!obj->isCalibrated())
            return;
    }
    QString path = QFileDialog::getSaveFileName(this, "Save Parameter",current_load_path, "config (*.yml *.xml)");
    if(path.size() != 0)
        obj->saveParameter(path);
}

void QCamCalib::removeCamera(int camera_id)
{
    CameraItem *item = getCameraItem(camera_id);
    tree_model->removeRow(item->row());
}

QImage loadImage(const QString &path)
{
    return QImage(path);
}

void QCamCalib::loadImages(QStringList &files, QList<QImage> &images)
{
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if(!cols || !rows)
        throw std::runtime_error("cannot find chessboard config");

    //select images
    QStringList paths = QFileDialog::getOpenFileNames(this, "Open images",current_load_path, "Images (*.png *.jpg)");

    //load images in parallel
    QFuture<QImage> imgs = QtConcurrent::mapped(paths,loadImage);
    future_watcher_images->setFuture(imgs);
    if(QDialog::Accepted != progress_dialog_images->exec() && future_watcher_images->isCanceled())
        return;
    future_watcher_images->waitForFinished();
    //progress_dialog_images->close();

    //add items
    QStringList::const_iterator iter_path = paths.begin();
    QFuture<QImage>::const_iterator iter_image = imgs.begin();
    for(;iter_path!=paths.end() && iter_image!=imgs.end();++iter_path,++iter_image)
    {
        QFileInfo info(*iter_path);
        current_load_path = info.absolutePath();
        files.push_back(info.fileName());
        images.push_back(*iter_image);
    }
}

void QCamCalib::loadCameraImages(int camera_id)
{
    QStringList files;
    QList<QImage> images;
    loadImages(files,images);

    QStringList::const_iterator iter_path = files.begin();
    QList<QImage>::const_iterator iter_image = images.begin();
    for(;iter_path!=files.end() && iter_image!=images.end();++iter_path,++iter_image)
    {
        CameraItem *item = getCameraItem(camera_id);
        item->addImage(*iter_path,*iter_image);
    }
}

void QCamCalib::loadStructuredLightImages(int id)
{
    QStringList files;
    QList<QImage> images;
    loadImages(files,images);

    QStringList::const_iterator iter_path = files.begin();
    QList<QImage>::const_iterator iter_image = images.begin();
    for(;iter_path!=files.end() && iter_image!=images.end();++iter_path,++iter_image)
    {
        StructuredLightItem *item = getStructuredLightItem(id);
        item->addImage(*iter_path,*iter_image);
    }
}

void QCamCalib::calibrateStructuredLight(int id)
{
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    QDoubleSpinBox *dx = findChild<QDoubleSpinBox*>("spinBoxDx");
    QDoubleSpinBox *dy = findChild<QDoubleSpinBox*>("spinBoxDy");
    if(!cols || !rows || !dx || !dy)
        throw std::runtime_error("cannot find chessboard config");

    //select images
    StructuredLightItem *item = getStructuredLightItem(id);
    if(item->countLaserLines() < 5)
    {
        QErrorMessage box;
        box.showMessage("Not enough images with laser lines. Find laser lines first." );
        box.exec();
        return;
    }

    future_watcher_calibrate->setFuture(QtConcurrent::run(item,&StructuredLightItem::calibrate,cols->value(),rows->value(),dx->value(),dy->value()));
    progress_dialog_calibrate->setRange(0,0);
    if(QDialog::Accepted != progress_dialog_calibrate->exec() && future_watcher_calibrate->isCanceled())
        return;
    future_watcher_calibrate->waitForFinished();
}


void QCamCalib::calibrateCamera(int camera_id)
{
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    QDoubleSpinBox *dx = findChild<QDoubleSpinBox*>("spinBoxDx");
    QDoubleSpinBox *dy = findChild<QDoubleSpinBox*>("spinBoxDy");
    if(!cols || !rows || !dx || !dy)
        throw std::runtime_error("cannot find chessboard config");

    //select images
    CameraItem *item = getCameraItem(camera_id);
    if(item->countChessboards() < 5)
    {
        QErrorMessage box;
        box.showMessage("Not enough chessboards for calibration. Find chessboard corners first." );
        box.exec();
        return;
    }

    future_watcher_calibrate->setFuture(QtConcurrent::run(item,&CameraItem::calibrate,cols->value(),rows->value(),dx->value(),dy->value()));
    progress_dialog_calibrate->setRange(0,0);
    if(QDialog::Accepted != progress_dialog_calibrate->exec() && future_watcher_calibrate->isCanceled())
        return;
    future_watcher_calibrate->waitForFinished();
}

CameraItem *QCamCalib::getCameraItem(int camera_id,bool raise)
{
    return getItem<CameraItem>(camera_id,raise);
}

StructuredLightItem *QCamCalib::getStructuredLightItem(int id,bool raise)
{
    return getItem<StructuredLightItem>(id,raise);
}

ImageItem *QCamCalib::getImageItem(int camera_id,const QString &name,bool raise)
{
    return getImageItem<ImageItem,CameraItem>(camera_id,name,raise);
}

StructuredLightImageItem *QCamCalib::getStructuredLightImageItem(int id,const QString &name,bool raise)
{
    return getImageItem<StructuredLightImageItem,StructuredLightItem>(id,name,raise);
}

void QCamCalib::addImage(const QString &name,const QImage &image,int camera_id)
{
    CameraItem *item = getCameraItem(camera_id);
    item->addImage(name,image);
}

void QCamCalib::findLaserLine(int id,const QString &name)
{
    QVector<StructuredLightImageItem*> items;
    StructuredLightItem *s_item = getStructuredLightItem(id,false);
    if(s_item)
    {
        int count = s_item->countImages()-1;
        for(;count >=0;--count)
            items.push_back(s_item->getImageItem(count));
        if(items.empty())
        {
            QErrorMessage box;
            box.showMessage("No images." );
            box.exec();
            return;
        }
    }
    else
        items.push_back(getStructuredLightImageItem(id,name));

    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if(!cols || !rows)
        throw std::runtime_error("cannot find chessboard config");

    // get all images
    QList<QImage> images;
    QVector<StructuredLightImageItem*>::iterator iter =  items.begin();
    for(;iter != items.end();++iter)
        images.push_back((*iter)->getRawImage());

    // find chessboards
    QFuture<QVector<QPointF> > chessboards;
    chessboards = QtConcurrent::mapped(images,
            boost::bind(static_cast<QVector<QPointF>(*)(const QImage&,int,int)>(ImageItem::findChessboardFastX),_1,cols->value(),rows->value()));
    future_watcher_chessboard->setFuture(chessboards);
    if(QDialog::Accepted != progress_dialog_chessboard->exec() && future_watcher_chessboard->isCanceled())
        return;
    future_watcher_chessboard->waitForFinished();

    // set chessboards and prepare laser line serach
    QVector<StructuredLightImageItem::Data> data;
    int count = images.size();
    for(int i =0;i < count; ++i)
    {
        StructuredLightImageItem *image_item = items[i];
        image_item->setChessboard(chessboards.results()[i],cols->value(),rows->value());
        if(image_item->hasChessboard())
        {
            StructuredLightImageItem::Data d;
            d.pointer = image_item;
            d.image = images[i];
            d.rect[0] = chessboards.results()[i].front();
            d.rect[1] = chessboards.results()[i][cols->value()-1];
            d.rect[2] = chessboards.results()[i].back();
            d.rect[3] = chessboards.results()[i][cols->value()*(rows->value()-1)];
            data.push_back(d);
        }
    }
    displayImage(items.front()->getImage());

    // search for laser line
    if(data.empty())
    {
        QErrorMessage box;
        box.showMessage("no images with valid chessboard");
        box.exec();
        return;
    }

    QFuture<QVector<QPointF> > laser_lines;
    laser_lines = QtConcurrent::mapped(data,
            boost::bind(static_cast<QVector<QPointF>(*)(const StructuredLightImageItem::Data&)>(StructuredLightImageItem::findLaserLine),_1));
    future_watcher_laser_line->setFuture(laser_lines);
    if(QDialog::Accepted != progress_dialog_laser_line->exec() && future_watcher_laser_line->isCanceled())
        return;
    future_watcher_laser_line->waitForFinished();

    // set laser line
    count = data.size();
    for(int i =0;i < count; ++i)
    {
        StructuredLightImageItem *image_item = data[i].pointer;
        image_item->setLaserLine(laser_lines.results()[i]);
    }
    displayImage(data.front().pointer->getImage());
}


void QCamCalib::findChessBoard(int camera_id,const QString &name,bool fastx)
{
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if(!cols || !rows)
        throw std::runtime_error("cannot find chessboard config");

    QList<QImage> images;
    CameraItem *camera_item = getCameraItem(camera_id,false);
    ImageItem *image_item = NULL;
    if(camera_item)
    {
        images = camera_item->getRawImages();
        if(camera_item->countImages() == 0)
        {
            QErrorMessage box;
            box.showMessage("No images." );
            box.exec();
            return;
        }
        image_item = camera_item->getImageItem(0);
    }
    else
    {
        image_item = getImageItem(camera_id,name);
        images.push_back(image_item->getRawImage());
    }

    QFuture<QVector<QPointF> > chessboards;
    if(fastx)
        chessboards = QtConcurrent::mapped(images,
                boost::bind(static_cast<QVector<QPointF>(*)(const QImage&,int,int)>(ImageItem::findChessboardFastX),_1,cols->value(),rows->value()));
    else
        chessboards = QtConcurrent::mapped(images,
                boost::bind(static_cast<QVector<QPointF>(*)(const QImage&,int,int)>(ImageItem::findChessboard),_1,cols->value(),rows->value()));
    future_watcher_chessboard->setFuture(chessboards);
    if(QDialog::Accepted != progress_dialog_chessboard->exec() && future_watcher_chessboard->isCanceled())
        return;

    // set all other items
    if(camera_item)
    {
        int count = camera_item->countImages();
        for(int i =0;i < count; ++i)
        {
            image_item = camera_item->getImageItem(i);
            image_item->setChessboard(chessboards.results()[i],cols->value(),rows->value());
        }
    }
    else
        image_item->setChessboard(chessboards.results().front(),cols->value(),rows->value());
    displayImage(image_item->getImage());
}

void QCamCalib::findChessBoardFastX(int camera_id,const QString &name)
{
    findChessBoard(camera_id,name,true);
}

void QCamCalib::removeCurrentItem()
{
    QTreeView *tree_view = findChild<QTreeView*>("treeView");
    if(!tree_view)
        throw std::runtime_error("Cannot find treeView object");
    QModelIndex index = tree_view->currentIndex();
    if(index.isValid())
    {
        if(index.parent().isValid())
            tree_model->itemFromIndex(index.parent())->removeRow(index.row());
        else
            tree_model->removeRow(index.row());
    }
}

void QCamCalib::displayImage(const QImage &image)
{
    image_view->displayImage(image);
}

void QCamCalib::clickedTreeView(const QModelIndex& index)
{
    if(!index.isValid())
        return;
    const QStandardItemModel *model = dynamic_cast<const QStandardItemModel*>(index.model());
    if(!model)
        return;
    QStandardItem *item = model->itemFromIndex(index);
    if(!item)
        return;
    ImageItem *image = dynamic_cast<ImageItem*>(item);
    if(image)
        image_view->displayImage(image->getImage());
}


