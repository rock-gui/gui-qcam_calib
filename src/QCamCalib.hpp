#ifndef QCAMCALIB_HPP
#define QCAMCALIB_HPP

#include <QtGui>
#include <QFuture>
#include <QFutureWatcher>

namespace qcam_calib {
class CameraItem;
class ImageItem;
class ImageView;
class StereoCameraItem;
}

/**
 * \brief Widget for calibrating pinhole cameras
 *
 * As underlying Back-End OpenCV is used:
 *  * cv::findChessboardCorners
 *  * cv::calibrateCamera
 *
 * \author Alexander.Duda@dfki.de
 */
class QCamCalib: public QWidget {
    Q_OBJECT
public:
    QCamCalib(QWidget *parent = 0);
    virtual ~QCamCalib();

public slots:
    /**
     * \brief Adds a new camera to the workspace
     *
     * \param[in] camera_id The id of the camera. If no id is given the next free id is used starting from 0
     * \author Alexander.Duda@dfki.de
     */
    void addCamera(int camera_id = -1);

    /**
     * \brief Removes a camera from the workspace
     *
     * \note If no camera id is given it is assumed that a camera item is selected in the TreeView.
     *
     * \param[in] camera_id The id of the camera.
     * \author Alexander.Duda@dfki.de
     */
    void removeCamera(int camera_id = -1);

    /**
     * \brief Opens a dialog to select images which are going to be loaded and added to the camera.
     *
     * \note If no camera id is given it is assumed that a camera item is selected in the TreeView.
     *
     * This call automatically performs chessboard detection
     *
     * \param[in] camera_id The id of the camera.
     * \author Alexander.Duda@dfki.de
     */
    void loadImages(int camera_id = -1);

    /**
     * \brief Adds an image to a camera
     *
     * \note If no camera id is given it is assumed that a camera item is selected in the TreeView.
     *
     * \param[in] name The name of the image
     * \param[in] image The image
     * \param[in] camera_id The id of the camera.
     * \author Alexander.Duda@dfki.de
     */
    void addImage(const QString &name, const QImage &image, int camera_id = -1);

    /**
     * \brief Opens a file dialog and saves the camera parameter as YAML or XML
     *
     * \note If no camera id is given it is assumed that a camera item is selected in the TreeView.
     *
     * \param[in] camera_id The id of the camera.
     * \author Alexander.Duda@dfki.de
     */
    void saveCameraParameter(int camera_id = -1);

    /**
     * \brief Calibrates a camera
     *
     * \note If no camera id is given it is assumed that a camera item is selected in the TreeView.
     *
     * \param[in] camera_id The id of the camera.
     * \author Alexander.Duda@dfki.de
     */
    void calibrateCamera(int camera_id = -1);

    /**
     * \brief Finds chessboard corners in an image
     *
     * \note if no camera id is given it is assumed that an image is selected in the TreeView
     *
     * \param[in] camera_id The id of the camera.
     * \param[in] name The name of the image
     * \author Alexander.Duda@dfki.de
     */
    void findChessBoard(int camera_id = -1, const QString &name = QString(""));


    // stereo camrea methods
    /**
     * \brief Adds a new stereo camera to the workspace
     *
     * \param[in] camera_id The id of the stereo camera. If no id is given the next free id is used starting from 0
     * \author trocolit2@gmail.com
     */
    void addStereoItem(int camera_id = -1);

    void saveStereoItemParameters(int camera_id = -1);

    void calibreStereoItem(int stereo_id = -1);

    void loadStereoImages(int camera_id = -1);

private slots:
    void contextMenuTreeView(const QPoint &point);
    void clickedTreeView(const QModelIndex& index);
    void displayImage(const QImage &image);
    void removeCurrentItem();

private:
    qcam_calib::CameraItem *getCameraItem(int camera_id);
    qcam_calib::ImageItem *getImageItem(int camera_id, const QString &name);

    //find camera from stereo item
    template<typename T>
    T getItems(int item_id, QString erro_msg);




private:
    // file paths
    QString current_load_path;

    // tree model
    QStandardItemModel *tree_model;

    // menues
    QMenu *camera_item_menu;
    QMenu *tree_view_menu;
    QMenu *image_item_menu;

    QMenu *stereo_item_menu;
    QMenu *stereo_camera_item_menu;
    QMenu *stereo_image_item_menu;

    // image dispay
    qcam_calib::ImageView *image_view;

    // progress stuff
    QProgressDialog *progress_dialog_images;
    QProgressDialog *progress_dialog_chessboard;
    QProgressDialog *progress_dialog_calibrate;
    QFutureWatcher<QImage> *future_watcher_images;
    QFutureWatcher<QVector<QPointF> > *future_watcher_chessboard;
    QFutureWatcher<void> *future_watcher_calibrate;

    QProgressDialog *progress_dialog_stereo_images_items;
    QFutureWatcher<QList<QStandardItem*> > *future_watcher_stereo_images_items;
};

#endif /* QCAMCALIB_HPP */
