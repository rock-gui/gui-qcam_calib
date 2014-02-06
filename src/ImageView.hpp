#ifndef IMAGEVIEW_HPP
#define IMAGEVIEW_HPP

#include <QGraphicsView>

namespace qcam_calib
{
    class ImageView : public QGraphicsView
    {
        public:
            ImageView(QWidget *parent = 0);
            virtual ~ImageView();

        public slots:
            void displayImage(const QImage &image);
            virtual void resizeEvent(QResizeEvent * event);
            void fitImage();

        private:
            QGraphicsPixmapItem *pixmap_item;
            QGraphicsTextItem *welcome;
    };
}

#endif
