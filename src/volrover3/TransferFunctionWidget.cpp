#include <volrover3/TransferFunctionWidget.h>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QPainter>
#include <QMouseEvent>
#include <QPushButton>
#include <QSlider>
#include <cmath>
#include <algorithm>

// Simple color bar widget
class ColorBarWidget : public QWidget
{
public:
    ColorBarWidget(QWidget *parent = nullptr) 
        : QWidget(parent) 
    {
        setMinimumHeight(40);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    }

    void setColorPoints(const std::vector<TransferFunctionWidget::ColorPoint> &points) {
        m_colorPoints = points;
        update();
    }

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        if (m_colorPoints.empty()) {
            painter.fillRect(rect(), Qt::gray);
            return;
        }

        // Draw gradient
        int w = width();
        for (int x = 0; x < w; ++x) {
            double t = static_cast<double>(x) / (w - 1);
            QColor color = interpolateColor(t);
            painter.setPen(color);
            painter.drawLine(x, 0, x, height());
        }
    }

private:
    QColor interpolateColor(double t) const {
        if (m_colorPoints.empty()) return Qt::white;
        if (m_colorPoints.size() == 1) return m_colorPoints[0].color;

        // Find surrounding color points
        for (size_t i = 0; i < m_colorPoints.size() - 1; ++i) {
            if (t >= m_colorPoints[i].value && t <= m_colorPoints[i + 1].value) {
                double localT = (t - m_colorPoints[i].value) / 
                               (m_colorPoints[i + 1].value - m_colorPoints[i].value);
                
                QColor c1 = m_colorPoints[i].color;
                QColor c2 = m_colorPoints[i + 1].color;
                
                int r = c1.red() + localT * (c2.red() - c1.red());
                int g = c1.green() + localT * (c2.green() - c1.green());
                int b = c1.blue() + localT * (c2.blue() - c1.blue());
                
                return QColor(r, g, b);
            }
        }

        return m_colorPoints.back().color;
    }

    std::vector<TransferFunctionWidget::ColorPoint> m_colorPoints;
};

// Simple opacity graph widget
class OpacityGraphWidget : public QWidget
{
    Q_OBJECT
public:
    OpacityGraphWidget(QWidget *parent = nullptr)
        : QWidget(parent)
        , m_selectedPoint(-1)
        , m_dragging(false)
    {
        setMinimumHeight(100);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setMouseTracking(true);
    }

    void setOpacityPoints(const std::vector<TransferFunctionWidget::OpacityPoint> &points) {
        m_opacityPoints = points;
        update();
    }
    
    const std::vector<TransferFunctionWidget::OpacityPoint>& getOpacityPoints() const {
        return m_opacityPoints;
    }

signals:
    void opacityChanged();

protected:
    void paintEvent(QPaintEvent *) override {
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing);

        // Draw background
        painter.fillRect(rect(), QColor(240, 240, 240));

        // Draw grid
        painter.setPen(QColor(200, 200, 200));
        for (int i = 0; i <= 4; ++i) {
            int y = i * height() / 4;
            painter.drawLine(0, y, width(), y);
        }

        if (m_opacityPoints.empty()) return;

        // Draw opacity curve
        painter.setPen(QPen(Qt::blue, 2));
        for (size_t i = 0; i < m_opacityPoints.size() - 1; ++i) {
            int x1 = m_opacityPoints[i].value * width();
            int y1 = (1.0 - m_opacityPoints[i].opacity) * height();
            int x2 = m_opacityPoints[i + 1].value * width();
            int y2 = (1.0 - m_opacityPoints[i + 1].opacity) * height();
            painter.drawLine(x1, y1, x2, y2);
        }

        // Draw control points
        painter.setBrush(Qt::red);
        for (const auto &pt : m_opacityPoints) {
            int x = pt.value * width();
            int y = (1.0 - pt.opacity) * height();
            painter.drawEllipse(QPoint(x, y), 5, 5);
        }
    }

    void mousePressEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            // Find nearest control point
            int nearestIdx = findNearestPoint(event->pos());
            if (nearestIdx >= 0 && nearestIdx < static_cast<int>(m_opacityPoints.size())) {
                double dist = pointDistance(event->pos(), nearestIdx);
                if (dist < 10.0) {
                    m_selectedPoint = nearestIdx;
                    m_dragging = true;
                    return;
                }
            }
            
            // Add new point if clicked away from existing points
            double x = static_cast<double>(event->pos().x()) / width();
            double y = 1.0 - static_cast<double>(event->pos().y()) / height();
            x = std::max(0.0, std::min(1.0, x));
            y = std::max(0.0, std::min(1.0, y));
            
            // Insert point in sorted order
            TransferFunctionWidget::OpacityPoint newPt{x, y};
            auto it = std::lower_bound(m_opacityPoints.begin(), m_opacityPoints.end(), newPt,
                [](const TransferFunctionWidget::OpacityPoint& a, const TransferFunctionWidget::OpacityPoint& b) { 
                    return a.value < b.value; 
                });
            m_selectedPoint = std::distance(m_opacityPoints.begin(), it);
            m_opacityPoints.insert(it, newPt);
            m_dragging = true;
            update();
            emit opacityChanged();
        } else if (event->button() == Qt::RightButton) {
            // Remove point on right-click (but keep at least 2 points)
            if (m_opacityPoints.size() > 2) {
                int nearestIdx = findNearestPoint(event->pos());
                if (nearestIdx >= 0 && nearestIdx < static_cast<int>(m_opacityPoints.size())) {
                    double dist = pointDistance(event->pos(), nearestIdx);
                    if (dist < 10.0) {
                        m_opacityPoints.erase(m_opacityPoints.begin() + nearestIdx);
                        update();
                        emit opacityChanged();
                    }
                }
            }
        }
    }
    
    void mouseMoveEvent(QMouseEvent *event) override {
        if (m_dragging && m_selectedPoint >= 0 && m_selectedPoint < static_cast<int>(m_opacityPoints.size())) {
            double x = static_cast<double>(event->pos().x()) / width();
            double y = 1.0 - static_cast<double>(event->pos().y()) / height();
            
            // Clamp to valid range
            y = std::max(0.0, std::min(1.0, y));
            
            // Don't allow moving endpoints horizontally, only vertically
            if (m_selectedPoint == 0) {
                x = 0.0;
            } else if (m_selectedPoint == static_cast<int>(m_opacityPoints.size()) - 1) {
                x = 1.0;
            } else {
                // Constrain x between neighbors
                double minX = m_opacityPoints[m_selectedPoint - 1].value + 0.01;
                double maxX = m_opacityPoints[m_selectedPoint + 1].value - 0.01;
                x = std::max(minX, std::min(maxX, x));
            }
            
            m_opacityPoints[m_selectedPoint].value = x;
            m_opacityPoints[m_selectedPoint].opacity = y;
            update();
            emit opacityChanged();
        }
    }
    
    void mouseReleaseEvent(QMouseEvent *event) override {
        if (event->button() == Qt::LeftButton) {
            m_dragging = false;
            m_selectedPoint = -1;
        }
    }

private:
    int findNearestPoint(const QPoint& pos) const {
        if (m_opacityPoints.empty()) return -1;
        
        int nearest = 0;
        double minDist = pointDistance(pos, 0);
        
        for (size_t i = 1; i < m_opacityPoints.size(); ++i) {
            double dist = pointDistance(pos, i);
            if (dist < minDist) {
                minDist = dist;
                nearest = i;
            }
        }
        
        return nearest;
    }
    
    double pointDistance(const QPoint& pos, int idx) const {
        if (idx < 0 || idx >= static_cast<int>(m_opacityPoints.size())) return 1e9;
        
        int x = m_opacityPoints[idx].value * width();
        int y = (1.0 - m_opacityPoints[idx].opacity) * height();
        
        int dx = pos.x() - x;
        int dy = pos.y() - y;
        
        return std::sqrt(dx * dx + dy * dy);
    }

    std::vector<TransferFunctionWidget::OpacityPoint> m_opacityPoints;
    int m_selectedPoint;
    bool m_dragging;
};

TransferFunctionWidget::TransferFunctionWidget(QWidget *parent)
    : QWidget(parent)
    , m_presetCombo(nullptr)
    , m_colorBarWidget(nullptr)
    , m_opacityWidget(nullptr)
    , m_dataMin(0.0)
    , m_dataMax(1.0)
{
    setupUI();
    createDefaultTransferFunction();
}

TransferFunctionWidget::~TransferFunctionWidget()
{
}

void TransferFunctionWidget::setupUI()
{
    QVBoxLayout *layout = new QVBoxLayout(this);

    // Preset selector
    QHBoxLayout *presetLayout = new QHBoxLayout();
    presetLayout->addWidget(new QLabel("Preset:"));
    m_presetCombo = new QComboBox();
    m_presetCombo->addItem("Grayscale");
    m_presetCombo->addItem("Rainbow");
    m_presetCombo->addItem("Hot");
    m_presetCombo->addItem("Cool");
    m_presetCombo->addItem("X-Ray");
    connect(m_presetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TransferFunctionWidget::onPresetChanged);
    presetLayout->addWidget(m_presetCombo);
    layout->addLayout(presetLayout);

    // Color bar
    layout->addWidget(new QLabel("Color Map:"));
    m_colorBarWidget = new ColorBarWidget(this);
    layout->addWidget(m_colorBarWidget);

    // Opacity graph
    layout->addWidget(new QLabel("Opacity:"));
    m_opacityWidget = new OpacityGraphWidget(this);
    connect(static_cast<OpacityGraphWidget*>(m_opacityWidget), &OpacityGraphWidget::opacityChanged,
            this, &TransferFunctionWidget::onOpacityGraphChanged);
    layout->addWidget(m_opacityWidget);
    
    // Add instructions
    QLabel *instructions = new QLabel("Left-click to add/drag points, Right-click to remove");
    instructions->setStyleSheet("QLabel { color: gray; font-size: 9pt; }");
    layout->addWidget(instructions);

    layout->addStretch();
}

void TransferFunctionWidget::createDefaultTransferFunction()
{
    applyPreset("Grayscale");
    
    // Initialize default opacity points if empty
    if (m_opacityPoints.empty() && m_opacityWidget != nullptr) {
        m_opacityPoints.push_back({0.0, 0.0});
        m_opacityPoints.push_back({1.0, 1.0});
        
        // Update the opacity widget
        auto opacityWidget = static_cast<OpacityGraphWidget*>(m_opacityWidget);
        opacityWidget->setOpacityPoints(m_opacityPoints);
    }
}

void TransferFunctionWidget::applyPreset(const QString &presetName)
{
    m_colorPoints.clear();
    // Don't clear opacity points - keep them independent!

    if (presetName == "Grayscale") {
        m_colorPoints.push_back({0.0, QColor(0, 0, 0)});
        m_colorPoints.push_back({1.0, QColor(255, 255, 255)});
    } else if (presetName == "Rainbow") {
        m_colorPoints.push_back({0.0, QColor(0, 0, 255)});      // Blue
        m_colorPoints.push_back({0.25, QColor(0, 255, 255)});   // Cyan
        m_colorPoints.push_back({0.5, QColor(0, 255, 0)});      // Green
        m_colorPoints.push_back({0.75, QColor(255, 255, 0)});   // Yellow
        m_colorPoints.push_back({1.0, QColor(255, 0, 0)});      // Red
    } else if (presetName == "Hot") {
        m_colorPoints.push_back({0.0, QColor(0, 0, 0)});
        m_colorPoints.push_back({0.33, QColor(255, 0, 0)});
        m_colorPoints.push_back({0.66, QColor(255, 255, 0)});
        m_colorPoints.push_back({1.0, QColor(255, 255, 255)});
    } else if (presetName == "Cool") {
        m_colorPoints.push_back({0.0, QColor(0, 255, 255)});
        m_colorPoints.push_back({1.0, QColor(255, 0, 255)});
    } else if (presetName == "X-Ray") {
        m_colorPoints.push_back({0.0, QColor(0, 0, 0)});
        m_colorPoints.push_back({1.0, QColor(255, 255, 255)});
    }

    // Only initialize opacity points if they're empty
    if (m_opacityPoints.empty()) {
        m_opacityPoints.push_back({0.0, 0.0});
        m_opacityPoints.push_back({0.5, 0.5});
        m_opacityPoints.push_back({1.0, 1.0});
    }

    updateColorBar();
    emit transferFunctionChanged();
}

void TransferFunctionWidget::setDataRange(double min, double max)
{
    m_dataMin = min;
    m_dataMax = max;
}

void TransferFunctionWidget::onPresetChanged(int index)
{
    applyPreset(m_presetCombo->currentText());
}

void TransferFunctionWidget::onColorMapClicked(double x, double y)
{
    // Placeholder for adding color control points
}

void TransferFunctionWidget::onOpacityGraphChanged()
{
    emit transferFunctionChanged();
}

void TransferFunctionWidget::updateColorBar()
{
    static_cast<ColorBarWidget*>(m_colorBarWidget)->setColorPoints(m_colorPoints);
    // Don't reset opacity points - they're managed independently by the opacity widget
}

std::vector<double> TransferFunctionWidget::getColorTable() const
{
    std::vector<double> table;
    
    for (const auto &pt : m_colorPoints) {
        double scalar = m_dataMin + pt.value * (m_dataMax - m_dataMin);
        table.push_back(scalar);
        table.push_back(pt.color.redF());
        table.push_back(pt.color.greenF());
        table.push_back(pt.color.blueF());
    }
    
    return table;
}

std::vector<double> TransferFunctionWidget::getOpacityTable() const
{
    std::vector<double> table;
    
    // Get the current opacity points from the opacity widget
    const auto &opacityPoints = static_cast<OpacityGraphWidget*>(m_opacityWidget)->getOpacityPoints();
    
    for (const auto &pt : opacityPoints) {
        double scalar = m_dataMin + pt.value * (m_dataMax - m_dataMin);
        table.push_back(scalar);
        table.push_back(pt.opacity);
    }
    
    return table;
}

#include "TransferFunctionWidget.moc"
