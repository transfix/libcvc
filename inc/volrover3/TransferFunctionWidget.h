#ifndef TRANSFERFUNCTIONWIDGET_H
#define TRANSFERFUNCTIONWIDGET_H

#include <QWidget>
#include <vector>
#include <QColor>

class QCustomPlot;
class QCPGraph;
class QCPColorMap;
class QComboBox;

class TransferFunctionWidget : public QWidget
{
    Q_OBJECT

public:
    // Color control points (value, r, g, b)
    struct ColorPoint {
        double value;
        QColor color;
    };

    // Opacity control points (value, opacity)
    struct OpacityPoint {
        double value;
        double opacity;
    };

    explicit TransferFunctionWidget(QWidget *parent = nullptr);
    ~TransferFunctionWidget();

    void setDataRange(double min, double max);
    
    std::vector<double> getColorTable() const;
    std::vector<double> getOpacityTable() const;
    
    void applyPreset(const QString &presetName);

signals:
    void transferFunctionChanged();

private slots:
    void onPresetChanged(int index);
    void onColorMapClicked(double x, double y);
    void onOpacityGraphChanged();

private:
    void setupUI();
    void createDefaultTransferFunction();
    void updateColorBar();

    QComboBox *m_presetCombo;
    QWidget *m_colorBarWidget;
    QWidget *m_opacityWidget;

    double m_dataMin;
    double m_dataMax;

    std::vector<ColorPoint> m_colorPoints;
    std::vector<OpacityPoint> m_opacityPoints;
};

#endif // TRANSFERFUNCTIONWIDGET_H
