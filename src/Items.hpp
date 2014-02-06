#ifndef QCAMCALIB_ITEM_MODELS_HPP
#define QCAMCALIB_ITEM_MODELS_HPP

#include <QStandardItem>
#include <QMenu>
#include <QVector>

namespace qcam_calib
{
    class QCamCalibItem: public QStandardItem
    {
        public:
            QCamCalibItem():QStandardItem(){};
            QCamCalibItem(const QString &string):QStandardItem(string){};
            QCamCalibItem(float val):QStandardItem(val){};
    };

    class CameraParameterItem: public QCamCalibItem
    {
        public:
            CameraParameterItem(const QString &string);
            void setParameter(const QString &name,double val=0);
            void save(const QString &path)const;
            double getParameter(const QString &name)const;
    };

    class ImageItem : public QCamCalibItem
    {
        public:
            static QVector<QPointF> findChessboard(const QImage &image,int cols ,int rows);

            ImageItem(const QString &name, const QImage &image);
            virtual ~ImageItem();
            QImage &getImage();
            QImage &getRawImage();
            const QVector<QPointF> &getChessboardCorners()const;

            bool findChessboard(int cols ,int rows);
            void setChessboard(const QVector<QPointF> &chessboard,int cols,int rows);

        private:
            QImage raw_image;
            QImage image;     // image with chessboard overlay
            QVector<QPointF> chessboard;
    };

    class CameraItem: public QCamCalibItem
    {
        public:
            CameraItem(int id, const QString &string);
            int getId();
            ImageItem* addImage(const QString &name,const QImage &image);
            ImageItem* getImageItem(const QString &name);
            void calibrate(int cols,int rows,float dx,float dy);
            void saveParameter(const QString &path)const;
            bool isCalibrated();
            int countChessboards();

        private:
            int camera_id;
            CameraParameterItem* camera_parameter;
            QStandardItem *images;
    };

}
#endif
