#ifndef GRIDOPTIONSDIALOG_H
#define GRIDOPTIONSDIALOG_H

#include <QWidget>
#include <memory>
#include <boost/signals2.hpp>

class QCheckBox;
class QSpinBox;
class QDoubleSpinBox;
class QSlider;
class QPushButton;
class GridNode;

class GridOptionsDialog : public QWidget
{
    Q_OBJECT

public:
    explicit GridOptionsDialog(std::shared_ptr<GridNode> gridNode, QWidget *parent = nullptr);
    ~GridOptionsDialog() override;

protected:
    void showEvent(QShowEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

private slots:
    void applyChanges();
    void chooseYZPlaneColor();
    void chooseXZPlaneColor();
    void chooseXYPlaneColor();
    void chooseTickLabelColor();
    
private:
    void setupUI();
    void connectSignals();
    void connectStateMonitoring();
    void disconnectStateMonitoring();
    void loadFromState();
    void onStateChanged();
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
    
    // Plane line width and opacity spin boxes
    QDoubleSpinBox *m_yzLineWidthSpinBox;
    QDoubleSpinBox *m_xzLineWidthSpinBox;
    QDoubleSpinBox *m_xyLineWidthSpinBox;
    QSlider *m_yzOpacitySlider;
    QSlider *m_xzOpacitySlider;
    QSlider *m_xyOpacitySlider;
    
    // Color storage
    double m_yzPlaneColor[3];
    double m_xzPlaneColor[3];
    double m_xyPlaneColor[3];
    double m_tickLabelColor[3];
    
    // Grid node reference
    std::shared_ptr<GridNode> m_gridNode;
    
    // State change monitoring
    std::vector<boost::signals2::scoped_connection> m_stateConnections;
    bool m_updatingFromState;
};

#endif // GRIDOPTIONSDIALOG_H
