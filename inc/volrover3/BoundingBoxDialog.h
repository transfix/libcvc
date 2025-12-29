#ifndef BOUNDINGBOXDIALOG_H
#define BOUNDINGBOXDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <cvc/bounding_box.h>

class BoundingBoxDialog : public QDialog
{
    Q_OBJECT

public:
    explicit BoundingBoxDialog(const cvc::bounding_box& bounds, QWidget *parent = nullptr);
    
    cvc::bounding_box getBoundingBox() const;
    
private slots:
    void onResetToGeometry();
    void onResetToVolume();
    
private:
    void setupUI();
    
    QLineEdit *m_minXEdit;
    QLineEdit *m_minYEdit;
    QLineEdit *m_minZEdit;
    QLineEdit *m_maxXEdit;
    QLineEdit *m_maxYEdit;
    QLineEdit *m_maxZEdit;
};

#endif // BOUNDINGBOXDIALOG_H
