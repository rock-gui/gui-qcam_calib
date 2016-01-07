/*
 * ItensStereoCamera.h
 *
 *  Created on: Jan 4, 2016
 *      Author: tiagotrocoli
 */

#ifndef GUI_QCAM_CALIB_SRC_ITEMSSTEREOCAMERA_HPP_
#define GUI_QCAM_CALIB_SRC_ITEMSSTEREOCAMERA_HPP_

#include <QStandardItem>
#include <QMenu>
#include <QVector>

#include <string>

#include "Items.hpp"

namespace qcam_calib {

static const QList<QString> INTRINSIC_PARAMETERS_LIST = QList<QString>() << "fx" << "fy" << "cx" << "cy" << "k1" << "k2" << "p1" << "p2" << "projection error" << "pixel error";
static const QList<QString> FUNDAMENTAL_MATRIX_PARAMETERS_LIST = QList<QString>() << "e1" << "e2" << "e3" << "e4" << "e5" << "e6" << "e7" << "e8" << "e9";

class StereoCameraParameterItem: public QCamCalibItem {
public:
    StereoCameraParameterItem(const QString &string, QList<QString> parameters);
    void setParameter(const QString &name, double val = 0);
    double getParameter(const QString &name) const;

    // vou modificar
    void save(const QString &path) const;

};

class StereoImageItem: public QCamCalibItem {
public:
    QVector<QPointF> findChessboard(const QImage &image, int cols, int rows);
    StereoImageItem(const QString &name, const QString &path);
    virtual ~StereoImageItem();

    const QString &getImagePath();
    const QVector<QPointF> &getChessboardCorners() const;
    QImage loadRawImage();
    QImage drawImageWithChessboard();
    bool findChessboard();

private:
    QString image_path; // path to image
    QVector<QPointF> chessboard;
};

class StereoCameraItem: public QCamCalibItem {
public:
    StereoCameraItem(int id, const QString &string);
    int getId();
    StereoImageItem* addImages(const QList<QStandardItem*> &stereoImageitems);
    StereoImageItem* getImageItem(const QString &name);

private:
    int camera_id;
    StereoCameraParameterItem* parameter;
    QStandardItem *images;
};

class StereoItem: public QCamCalibItem {
public:
    StereoItem(int id, const QString &string);
    int getId();
    static char* getBaseName();

    void calibrate(int cols, int rows, float dx, float dy);
    void saveParameter(const QString &path) const;
    bool isCalibrated();

private:
    int stereo_id;
    StereoCameraParameterItem* fundamental_matrix;
    StereoCameraItem *left_camera;
    StereoCameraItem *right_camera;
};

class StereoTools {

public:

    static QList<QStandardItem*> loadStereoImageItem(const QString &path);
    static QList<QStandardItem*> findChessboard(const QList<QStandardItem*> &items);

};

}

#endif /* GUI_QCAM_CALIB_SRC_ITEMSSTEREOCAMERA_HPP_ */
