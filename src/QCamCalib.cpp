#include "QCamCalib.hpp"
#include "Items.hpp"
#include "ItemsStereoCamera.hpp"
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

QCamCalib::QCamCalib(QWidget *parent) :
        QWidget(parent), current_load_path("."), tree_model(NULL), camera_item_menu(NULL), tree_view_menu(NULL), image_item_menu(NULL), progress_dialog_images(NULL), progress_dialog_chessboard(NULL), progress_dialog_calibrate(
        NULL), future_watcher_images(NULL), future_watcher_chessboard(NULL), future_watcher_calibrate(NULL) {
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
    QAction* act = new QAction("load images", this);
    connect(act, SIGNAL(triggered()), this, SLOT(loadImages()));
    camera_item_menu->addAction(act);

    act = new QAction("calibrate", this);
    connect(act, SIGNAL(triggered()), this, SLOT(calibrateCamera()));
    camera_item_menu->addAction(act);

    act = new QAction("save parameter", this);
    connect(act, SIGNAL(triggered()), this, SLOT(saveCameraParameter()));
    camera_item_menu->addAction(act);

    QAction *act_remove = new QAction("Remove", this);
    connect(act_remove, SIGNAL(triggered()), this, SLOT(removeCurrentItem()));
    camera_item_menu->addAction(act_remove);

    // tree view menu
    // mono camera
    tree_view_menu = new QMenu(this);
    act = new QAction("add Camera", this);
    connect(act, SIGNAL(triggered()), this, SLOT(addCamera()));
    tree_view_menu->addAction(act);

    // image item menu
    image_item_menu = new QMenu(this);
    image_item_menu->addAction(act_remove);
    act = new QAction("find chessboard", this);
    connect(act, SIGNAL(triggered()), this, SLOT(findChessBoard()));
    image_item_menu->addAction(act);

    // tree view menu
    // stereo item
    act = new QAction("Add Stereo Camera", this);
    connect(act, SIGNAL(triggered()), this, SLOT(addStereoItem()));
    tree_view_menu->addAction(act);

    // stereo item view menu
    this->stereo_item_menu = new QMenu(this);

    act = new QAction("Calibre Stereo Camera", this);
    this->stereo_item_menu->addAction(act);
    connect(act, SIGNAL(triggered()), this, SLOT(calibreStereoItem()));
    this->stereo_item_menu->addAction(act);

    act = new QAction("Save Stereo Camera Parameters", this);
    this->stereo_item_menu->addAction(act);
    connect(act, SIGNAL(triggered()), this, SLOT(saveStereoItemParameters()));
    this->stereo_item_menu->addAction(act);

    this->stereo_item_menu->addAction(act_remove);

    // stereo camera menu
    this->stereo_camera_item_menu = new QMenu(this);
    act = new QAction("Load Images", this);
    this->stereo_camera_item_menu->addAction(act);
    connect(act, SIGNAL(triggered()), this, SLOT(loadStereoImages()));
    this->stereo_camera_item_menu->addAction(act);

    // stereo image menu
    this->stereo_image_item_menu = new QMenu(this);
    this->stereo_image_item_menu->addAction(act_remove);

    //graphics view
    image_view = gui.imageView;

    // add initial camera
    //addCamera();

    // progress dialog
    progress_dialog_images = new QProgressDialog("loading images", "cancel", 0, 0, this);
    future_watcher_images = new QFutureWatcher<QImage>(this);
    connect(future_watcher_images, SIGNAL(progressValueChanged(int)), progress_dialog_images, SLOT(setValue(int)));
    connect(future_watcher_images, SIGNAL(progressRangeChanged(int, int)), progress_dialog_images, SLOT(setRange(int, int)));
    connect(future_watcher_images, SIGNAL(finished()), progress_dialog_images, SLOT(accept()));

    progress_dialog_chessboard = new QProgressDialog("searching for chessboards", "cancel", 0, 0, this);
    future_watcher_chessboard = new QFutureWatcher<QVector<QPointF> >(this);
    connect(future_watcher_chessboard, SIGNAL(progressValueChanged(int)), progress_dialog_chessboard, SLOT(setValue(int)));
    connect(future_watcher_chessboard, SIGNAL(finished()), progress_dialog_chessboard, SLOT(accept()));
    //  connect(future_watcher_chessboard, SIGNAL(progressRangeChanged(int, int)), progress_dialog_chessboard, SLOT(setRange(int, int)));

    progress_dialog_calibrate = new QProgressDialog("calibrate camera", "cancel", 0, 0, this);
    progress_dialog_calibrate->setCancelButton(NULL);
    future_watcher_calibrate = new QFutureWatcher<void>(this);
    connect(future_watcher_calibrate, SIGNAL(progressValueChanged(int)), progress_dialog_calibrate, SLOT(setValue(int)));
    connect(future_watcher_calibrate, SIGNAL(finished()), progress_dialog_calibrate, SLOT(accept()));

    // process dialog for stereo procedures
    this->progress_dialog_stereo_images_items = new QProgressDialog("Loading images and searching chessboard", "Cancel", 0, 0, this);
    this->future_watcher_stereo_images_items = new QFutureWatcher<QList<QStandardItem*> >(this);
    connect(this->future_watcher_stereo_images_items, SIGNAL(progressValueChanged(int)), this->progress_dialog_stereo_images_items, SLOT(setValue(int)));
    connect(this->future_watcher_stereo_images_items, SIGNAL(progressRangeChanged(int, int)), this->progress_dialog_stereo_images_items, SLOT(setRange(int, int)));
    connect(this->future_watcher_stereo_images_items, SIGNAL(finished()), this->progress_dialog_stereo_images_items, SLOT(accept()));

}

QCamCalib::~QCamCalib() {
}

void QCamCalib::contextMenuTreeView(const QPoint &point) {
    QTreeView *tree_view = findChild<QTreeView*>("treeView");
    if (!tree_view)
        throw std::runtime_error("Cannot find treeView object");

    QModelIndex index = tree_view->indexAt(point);
    if (index.isValid()) {
        QCamCalibItem *item = dynamic_cast<QCamCalibItem*>(tree_model->itemFromIndex(index));
        if (!item)
            return;
        else if (dynamic_cast<CameraItem*>(item))
            camera_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if (dynamic_cast<ImageItem*>(item))
            image_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if (dynamic_cast<StereoItem*>(item))
            this->stereo_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if (dynamic_cast<StereoCameraItem*>(item))
            this->stereo_camera_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
        else if (dynamic_cast<StereoImageItem*>(item))
            this->stereo_image_item_menu->exec(tree_view->viewport()->mapToGlobal(point));
    } else {
        tree_view_menu->exec(tree_view->viewport()->mapToGlobal(point));
    }
}

void QCamCalib::addCamera(int camera_id) {
    if (camera_id < 0)
        camera_id = 0;
    for (int i = 0; i <= tree_model->rowCount(); ++i) {
        std::stringstream strstr;
        strstr << CAMERA_BASE_NAME << (camera_id + i);
        QModelIndexList list = tree_model->match(tree_model->index(0, 0), Qt::DisplayRole, QVariant(strstr.str().c_str()), 1, Qt::MatchExactly);
        if (list.empty()) {
            tree_model->appendRow(new qcam_calib::CameraItem(camera_id + i, QString(strstr.str().c_str())));
            break;
        }
    }
}

void QCamCalib::saveCameraParameter(int camera_id) {
    CameraItem *item = getCameraItem(camera_id);
    if (!item->isCalibrated()) {
        calibrateCamera(camera_id);
        if (!item->isCalibrated())
            return;
    }
    QString path = QFileDialog::getSaveFileName(this, "Save Parameter", current_load_path, "config (*.yml *.xml)");
    if (path.size() != 0)
        item->saveParameter(path);
}

void QCamCalib::removeCamera(int camera_id) {
    CameraItem *item = getCameraItem(camera_id);
    tree_model->removeRow(item->row());
}

QImage loadImage(const QString &path) {
    return QImage(path);
}

void QCamCalib::loadImages(int camera_id) {
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if (!cols || !rows)
        throw std::runtime_error("cannot find chessboard config");

    //select images
    QStringList paths = QFileDialog::getOpenFileNames(this, "Open images", current_load_path, "Images (*.png *.jpg)");

    //load images in parallel
    QFuture<QImage> images = QtConcurrent::mapped(paths, loadImage);
    future_watcher_images->setFuture(images);
    if (QDialog::Accepted != progress_dialog_images->exec() && future_watcher_images->isCanceled())
        return;
    future_watcher_images->waitForFinished();
    //progress_dialog_images->close();

    //find chess boards in parallel
    QFuture<QVector<QPointF> > chessboards;
    chessboards = QtConcurrent::mapped(images, boost::bind(static_cast<QVector<QPointF> (*)(const QImage&, int, int)>(ImageItem::findChessboard), _1,cols->value(),rows->value()));
    future_watcher_chessboard->setFuture(chessboards);
    progress_dialog_chessboard->setRange(progress_dialog_images->minimum(), progress_dialog_images->maximum());
    if (QDialog::Accepted != progress_dialog_chessboard->exec() && future_watcher_chessboard->isCanceled())
        return;
    future_watcher_chessboard->waitForFinished();
    progress_dialog_chessboard->close();

    //add items
    QStringList::const_iterator iter_path = paths.begin();
    QFuture<QImage>::const_iterator iter_image = images.begin();
    QFuture<QVector<QPointF> >::const_iterator iter_chessboard = chessboards.begin();
    ImageItem *last_image_item = NULL;
    for (; iter_path != paths.end() && iter_image != images.end() && iter_chessboard != chessboards.end(); ++iter_path, ++iter_image, ++iter_chessboard) {
        QFileInfo info(*iter_path);
        current_load_path = info.absolutePath();

        CameraItem *item = getCameraItem(camera_id);
        last_image_item = item->addImage(info.fileName(), *iter_image);
        last_image_item->setChessboard(*iter_chessboard, cols->value(), rows->value());
    }
    if (last_image_item)
        displayImage(last_image_item->getImage());
}

void QCamCalib::calibrateCamera(int camera_id) {
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    QDoubleSpinBox *dx = findChild<QDoubleSpinBox*>("spinBoxDx");
    QDoubleSpinBox *dy = findChild<QDoubleSpinBox*>("spinBoxDy");
    if (!cols || !rows || !dx || !dy)
        throw std::runtime_error("cannot find chessboard config");

    //select images
    CameraItem *item = getCameraItem(camera_id);
    if (item->countChessboards() < 5) {
        QErrorMessage box;
        box.showMessage("Not enough chessboards for calibration. Add more images.");
        box.exec();
        return;
    }

    future_watcher_calibrate->setFuture(QtConcurrent::run(item, &CameraItem::calibrate, cols->value(), rows->value(), dx->value(), dy->value()));
    progress_dialog_calibrate->setRange(0, 0);
    if (QDialog::Accepted != progress_dialog_calibrate->exec() && future_watcher_calibrate->isCanceled())
        return;
}

CameraItem *QCamCalib::getCameraItem(int camera_id) {
    CameraItem *item = NULL;
    if (camera_id < 0) {
        QTreeView *tree_view = findChild<QTreeView*>("treeView");
        if (!tree_view)
            throw std::runtime_error("Cannot find treeView object");
        QModelIndex index = tree_view->currentIndex();
        if (index.isValid())
            item = dynamic_cast<CameraItem*>(tree_model->itemFromIndex(index));
    } else {
        for (int i = 0; i < tree_model->rowCount(); ++i) {
            item = dynamic_cast<CameraItem*>(tree_model->item(i, 0));
            if (item && item->getId() == camera_id)
                break;
            else
                item = NULL;
        }
    }
    if (item == NULL)
        throw std::runtime_error("Internal error: cannot find camera");
    return item;
}

ImageItem *QCamCalib::getImageItem(int camera_id, const QString &name) {
    ImageItem *item = NULL;
    if (camera_id < 0) {
        QTreeView *tree_view = findChild<QTreeView*>("treeView");
        if (!tree_view)
            throw std::runtime_error("Cannot find treeView object");
        QModelIndex index = tree_view->currentIndex();
        if (index.isValid())
            item = dynamic_cast<ImageItem*>(tree_model->itemFromIndex(index));
    } else {
        for (int i = 0; i < tree_model->rowCount() && !item; ++i) {
            CameraItem *camera = dynamic_cast<CameraItem*>(tree_model->item(i, 0));
            if (camera)
                item = camera->getImageItem(name);
        }
    }
    if (!item)
        throw std::runtime_error("Internal error: cannot find image item");
    return item;
}

void QCamCalib::addImage(const QString &name, const QImage &image, int camera_id) {
    CameraItem *item = getCameraItem(camera_id);
    item->addImage(name, image);
}

void QCamCalib::findChessBoard(int camera_id, const QString &name) {
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if (!cols || !rows)
        throw std::runtime_error("cannot find chessboard config");

    ImageItem *item = getImageItem(camera_id, name);
    QList<QImage> images;
    images.push_back(item->getRawImage());
    QFuture<QVector<QPointF> > chessboards;
    chessboards = QtConcurrent::mapped(images, boost::bind(static_cast<QVector<QPointF> (*)(const QImage&, int, int)>(ImageItem::findChessboard), _1,cols->value(),rows->value()));
    future_watcher_chessboard->setFuture(chessboards);
    progress_dialog_chessboard->setRange(0, 1);
    if (QDialog::Accepted != progress_dialog_chessboard->exec() && future_watcher_chessboard->isCanceled())
        return;
    item->setChessboard(chessboards.results().front(), cols->value(), rows->value());
    displayImage(item->getImage());
}

void QCamCalib::removeCurrentItem() {
    QTreeView *tree_view = findChild<QTreeView*>("treeView");
    if (!tree_view)
        throw std::runtime_error("Cannot find treeView object");
    QModelIndex index = tree_view->currentIndex();
    if (index.isValid()) {
        if (index.parent().isValid())
            tree_model->itemFromIndex(index.parent())->removeRow(index.row());
        else
            tree_model->removeRow(index.row());
    }
}

void QCamCalib::displayImage(const QImage &image) {
    image_view->displayImage(image);
}

void QCamCalib::clickedTreeView(const QModelIndex& index) {
    if (!index.isValid())
        return;
    const QStandardItemModel *model = dynamic_cast<const QStandardItemModel*>(index.model());
    if (!model)
        return;
    QStandardItem *item = model->itemFromIndex(index);
    if (!item)
        return;
    ImageItem *image = dynamic_cast<ImageItem*>(item);
    if (image)
        image_view->displayImage(image->getImage());

    StereoImageItem *stereo_image = dynamic_cast<StereoImageItem*>(item);
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if (stereo_image)
        image_view->displayImage(stereo_image->getImageWithChessboard(cols->value(), rows->value()));
}

void QCamCalib::addStereoItem(int camera_id) {
    if (camera_id < 0)
        camera_id = 0;
    for (int i = 0; i <= tree_model->rowCount(); ++i) {
        std::stringstream strstr;
        strstr << qcam_calib::StereoItem::getBaseName() << (camera_id + i);
        QModelIndexList list = tree_model->match(tree_model->index(0, 0), Qt::DisplayRole, QVariant(strstr.str().c_str()), 1, Qt::MatchExactly);
        if (list.empty()) {
            tree_model->appendRow(new qcam_calib::StereoItem(camera_id + i, QString(strstr.str().c_str())));
            break;
        }
    }
}

template<typename T>
T QCamCalib::getItems(int item_id, QString erro_msg) {
    T item = NULL;
    if (item_id < 0) {
        QTreeView *tree_view = findChild<QTreeView*>("treeView");
        if (!tree_view)
            throw std::runtime_error("Cannot find treeView object");
        QModelIndex index = tree_view->currentIndex();
        if (index.isValid())
            item = dynamic_cast<T>(tree_model->itemFromIndex(index));
    } else {
        for (int i = 0; i < tree_model->rowCount(); ++i) {
            item = dynamic_cast<T>(tree_model->item(i, 0));
            if (item && item->getId() == item_id)
                break;
            else
                item = NULL;
        }
    }
    if (item == NULL)
        throw std::runtime_error(erro_msg.toStdString());
    return item;
}

void QCamCalib::loadStereoImages(int camera_id) {

    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    if (!cols || !rows)
        throw std::runtime_error("Cannot find chessboard config!!");

    //select images
    QStringList paths = QFileDialog::getOpenFileNames(this, "Open images", current_load_path, "Images (*.png *.jpg)");

    //load stereo image items in parallel
    QFuture<QList<QStandardItem*> > stereoImagesItems = QtConcurrent::mapped(paths,
            boost::bind(static_cast<QList<QStandardItem*> (*)(const QString&, int, int)>(StereoTools::loadStereoImageAndFindChessboardItem), _1,cols->value(),rows->value()));

    //progress bar
    this->future_watcher_stereo_images_items->setFuture(stereoImagesItems);
    if (QDialog::Accepted != this->progress_dialog_stereo_images_items->exec() && this->future_watcher_stereo_images_items->isCanceled())
        return;
    this->future_watcher_stereo_images_items->waitForFinished();

    //add images items to camera
    QString error_msg = "Internal error: Cannot find camera";
    StereoCameraItem *item = getItems<StereoCameraItem*>(camera_id, error_msg);

    QFuture<QList<QStandardItem*> >::const_iterator iterStereoImageitems;
    for (iterStereoImageitems = stereoImagesItems.begin(); iterStereoImageitems != stereoImagesItems.end(); ++iterStereoImageitems)
        item->addImages(*iterStereoImageitems);
}


void QCamCalib::saveStereoItemParameters(int stereo_id) {
    QString erro_msg = "Internal error: Cannot find stereo camera.";
    StereoItem *item = getItems<StereoItem*>(stereo_id, erro_msg);

    if (!item->isCalibrated()) {
        calibreStereoItem(stereo_id);
        if (!item->isCalibrated())
            return;
    }
    QString path = QFileDialog::getSaveFileName(this, "Save Parameter", current_load_path, "config (*.yml *.xml)");
    if (path.size() != 0)
        item->saveParameter(path);
}

void QCamCalib::calibreStereoItem(int stereo_id) {
    QSpinBox *cols = findChild<QSpinBox*>("spinBoxCols");
    QSpinBox *rows = findChild<QSpinBox*>("spinBoxRows");
    QDoubleSpinBox *dx = findChild<QDoubleSpinBox*>("spinBoxDx");
    QDoubleSpinBox *dy = findChild<QDoubleSpinBox*>("spinBoxDy");
    if (!cols || !rows || !dx || !dy)
        throw std::runtime_error("Cannot find chessboard config.");

    QString erro_msg = "Internal error: Cannot find stereo camera.";
    StereoItem* stereo_item = getItems<StereoItem*>(stereo_id, erro_msg);

    try {
        future_watcher_calibrate->setFuture(QtConcurrent::run(stereo_item, &StereoItem::calibrate, cols->value(), rows->value(), dx->value(), dy->value()));
        progress_dialog_calibrate->setRange(0, 0);
        if (QDialog::Accepted != progress_dialog_calibrate->exec() && future_watcher_calibrate->isCanceled())
            return;
    } catch (std::exception& e) {
        QErrorMessage box;
        box.showMessage(QString(e.what()));
        box.exec();
        return;
    }

}
