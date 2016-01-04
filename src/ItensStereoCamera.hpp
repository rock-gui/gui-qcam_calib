/*
 * ItensStereoCamera.h
 *
 *  Created on: Jan 4, 2016
 *      Author: tiagotrocoli
 */

#ifndef GUI_QCAM_CALIB_SRC_ITENSSTEREOCAMERA_HPP_
#define GUI_QCAM_CALIB_SRC_ITENSSTEREOCAMERA_HPP_

#include <QStandardItem>
#include <QMenu>
#include <QVector>

#include "Items.hpp"

namespace qcam_calib {

class StereoCameraParameterItem: public QCamCalibItem {
public:
    StereoCameraParameterItem(const QString &string);
    void setParameter(const QString &name, double val = 0);
    void save(const QString &path) const;
    double getParameter(const QString &name) const;
};

class StereoImageItem: public QCamCalibItem {
public:
    static QVector<QPointF> findChessboard(const QImage &image, int cols, int rows);
    StereoImageItem(const QString &name, const QImage &image);
    virtual ~StereoImageItem();

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
    StereoImageItem* addImage(const QString &name, const QImage &image);
    StereoImageItem* getImageItem(const QString &name);

    void calibrate(int cols, int rows, float dx, float dy);
    void saveParameter(const QString &path) const;
    bool isCalibrated();
    int countChessboards();

private:
    int stereo_camera_id;
    StereoCameraParameterItem* stereo_camera_parameter;
    QStandardItem *left_images;
    QStandardItem *right_images;
};

}

#endif /* GUI_QCAM_CALIB_SRC_ITENSSTEREOCAMERA_HPP_ */
