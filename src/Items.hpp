#ifndef QCAMCALIB_ITEM_MODELS_HPP
#define QCAMCALIB_ITEM_MODELS_HPP

#include <QStandardItem>
#include <QMenu>
#include <QVector>
#include <QVector3D>

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
            virtual ~CameraParameterItem();
            virtual void setParameter(const QString &name,double val=0);
            virtual void save(const QString &path)const;
            virtual double getParameter(const QString &name)const;
    };

    class StructuredLightParameterItem: public CameraParameterItem
    {
        public:
            StructuredLightParameterItem(const QString &string);
            virtual ~StructuredLightParameterItem();
            virtual void save(const QString &path)const;
    };

    class ImageItem : public QCamCalibItem
    {
        public:
            static QVector<QPointF> findChessboard(const QImage &image,int cols ,int rows);
            static QVector<QPointF> findChessboardFastX(const QImage &image,int cols ,int rows);

            ImageItem(const QString &name, const QImage &image);
            virtual ~ImageItem();
            QImage &getImage();
            QImage &getRawImage();
            const QVector<QPointF> &getChessboardCorners()const;

            bool findChessboard(int cols ,int rows);
            bool findChessboardFastX(int cols ,int rows);
            void setChessboard(const QVector<QPointF> &chessboard,int cols,int rows);
            void setChessboardPlane(const QVector3D &plane, const QVector3D &normal);

            bool hasChessboard()const;

        protected:
            QImage raw_image;
            QImage image;     // image with chessboard overlay
            QVector<QPointF> chessboard;
            QVector3D chessboard_plane;
            QVector3D chessboard_normal;
            int chessboard_rows;
            int chessboard_cols;
    };

    class StructuredLightImageItem :public ImageItem
    {
        public:
            struct Data
            {
                QImage image;
                QPointF rect[4];
                StructuredLightImageItem *pointer;
                Data():pointer(NULL){};
            };
        public:
            static QVector<QPointF> findLaserLine(const Data &data);
            static QVector<QVector3D> get3DPoints(const QVector<QPointF> &laser_line,const QVector3D &chessboard_plane,const QVector3D &chessboard_normal);

            StructuredLightImageItem(const QString &name, const QImage &image);
            virtual ~StructuredLightImageItem(){};

            const QVector<QPointF> &getLaserLine()const;
            void setLaserLine(const QVector<QPointF> &laser_line);
        private:
            QVector<QPointF> laser_line;
    };

    class CalibrationObj : public QCamCalibItem
    {
        public:
            CalibrationObj(int id, const QString &name);
            virtual ~CalibrationObj(){};

            int getId();
            ImageItem* addImage(const QString &name,const QImage &image);
            ImageItem* getImageItem(const QString &name);

            int countImages()const;
            ImageItem* getImageItem(int id);
            QList<QImage> getRawImages();

            virtual bool isCalibrated()const = 0;
            virtual void saveParameter(const QString &path)const = 0;
        protected:
            int id;
            QStandardItem *images;
    };

    class CameraItem: public CalibrationObj
    {
        public:
            CameraItem(int id, const QString &string);
            virtual ~CameraItem(){};

            virtual bool isCalibrated()const;
            virtual void saveParameter(const QString &path)const;
            virtual void calibrate(int cols,int rows,float dx,float dy);

            int countChessboards()const;
        protected:
            CameraParameterItem* parameter;
    };

    class StructuredLightItem: public CameraItem
    {
        public:
            StructuredLightItem(int id, const QString &string);
            virtual ~StructuredLightItem(){};

            StructuredLightImageItem* addImage(const QString &name,const QImage &image);
            StructuredLightImageItem* getImageItem(int id);
            StructuredLightImageItem* getImageItem(const QString &name);
            int countLaserLines()const;

            virtual void calibrate(int cols,int rows,float dx,float dy);
            virtual bool isCalibrated()const;
    };
}
#endif
