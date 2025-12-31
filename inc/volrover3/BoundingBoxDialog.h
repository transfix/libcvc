#ifndef BOUNDINGBOXDIALOG_H
#define BOUNDINGBOXDIALOG_H

#include <QDialog>
#include <QLineEdit>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QPushButton>
#include <cvc/bounding_box.h>

class BoundingBoxDialog : public QDialog
{
    Q_OBJECT

public:
    explicit BoundingBoxDialog(const cvc::bounding_box& bounds, QWidget *parent = nullptr);
    
    cvc::bounding_box getBoundingBox() const;
    
private slots:
    void onResetToGraphics();
    void chooseTickLabelColor();
    void updateColorButton();
    
private:
    void setupUI();
    void loadTickSettings();
    void saveTickSettings();
    
    QLineEdit *m_minXEdit;
    QLineEdit *m_minYEdit;
    QLineEdit *m_minZEdit;
    QLineEdit *m_maxXEdit;
    QLineEdit *m_maxYEdit;
    QLineEdit *m_maxZEdit;
    
    // Coordinate controls (vertex-only, no interval)
    QCheckBox *m_showTicksCheckbox;
    QCheckBox *m_worldBBoxVisibleCheckbox;
    QPushButton *m_tickLabelColorButton;
    QSpinBox *m_tickLabelFontSizeSpinBox;
    double m_tickLabelColor[3];
};

#endif // BOUNDINGBOXDIALOG_H
