#ifndef GRIDOPTIONSDIALOG_H
#define GRIDOPTIONSDIALOG_H

#include <QDialog>

class QCheckBox;
class QSpinBox;
class QPushButton;

class GridOptionsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit GridOptionsDialog(QWidget *parent = nullptr);
    ~GridOptionsDialog() override = default;

protected:
    void showEvent(QShowEvent *event) override;

private slots:
    void applyChanges();
    void loadFromState();
    void chooseYZPlaneColor();
    void chooseXZPlaneColor();
    void chooseXYPlaneColor();
    void chooseTickLabelColor();
    
private:
    void setupUI();
    void connectSignals();
    void updateColorButton(QPushButton* button, double r, double g, double b);
    
    // Plane visibility checkboxes
    QCheckBox *m_yzPlaneCheckBox;  // YZ plane at X=0
    QCheckBox *m_xzPlaneCheckBox;  // XZ plane at Y=0
    QCheckBox *m_xyPlaneCheckBox;  // XY plane at Z=0
    
    // Tick visibility
    QCheckBox *m_showTicksCheckBox;
    
    // Grid divisions spin boxes
    QSpinBox *m_xDivisionsSpinBox;
    QSpinBox *m_yDivisionsSpinBox;
    QSpinBox *m_zDivisionsSpinBox;
    
    // Tick interval spin boxes
    QSpinBox *m_xTickIntervalSpinBox;
    QSpinBox *m_yTickIntervalSpinBox;
    QSpinBox *m_zTickIntervalSpinBox;
    
    // Per-plane color buttons
    QPushButton *m_yzPlaneColorButton;
    QPushButton *m_xzPlaneColorButton;
    QPushButton *m_xyPlaneColorButton;
    
    // Tick label properties
    QPushButton *m_tickLabelColorButton;
    QSpinBox *m_tickLabelFontSizeSpinBox;
    
    // Color storage
    double m_yzPlaneColor[3];
    double m_xzPlaneColor[3];
    double m_xyPlaneColor[3];
    double m_tickLabelColor[3];
};

#endif // GRIDOPTIONSDIALOG_H
