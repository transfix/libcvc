#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QMouseEvent>
#include <QPainter>
#include <QPushButton>
#include <QSlider>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <volrover3/SceneGraph.h>
#include <volrover3/TransferFunctionWidget.h>
#include <volrover3/VolumeNode.h>
#include <volrover3/volrover3_app.h>

// Simple color bar widget
class ColorBarWidget : public QWidget {
public:
  ColorBarWidget(QWidget *parent = nullptr) : QWidget(parent) {
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
    if (m_colorPoints.empty())
      return Qt::white;
    if (m_colorPoints.size() == 1)
      return m_colorPoints[0].color;

    // Find surrounding color points
    for (size_t i = 0; i < m_colorPoints.size() - 1; ++i) {
      if (t >= m_colorPoints[i].value && t <= m_colorPoints[i + 1].value) {
        double localT =
            (t - m_colorPoints[i].value) / (m_colorPoints[i + 1].value - m_colorPoints[i].value);

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
class OpacityGraphWidget : public QWidget {
  Q_OBJECT
public:
  OpacityGraphWidget(QWidget *parent = nullptr)
      : QWidget(parent), m_selectedPoint(-1), m_dragging(false) {
    setMinimumHeight(100);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    setMouseTracking(true);
  }

  void setOpacityPoints(const std::vector<TransferFunctionWidget::OpacityPoint> &points) {
    m_opacityPoints = points;
    update();
  }

  const std::vector<TransferFunctionWidget::OpacityPoint> &getOpacityPoints() const {
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

    if (m_opacityPoints.empty())
      return;

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
      auto it = std::lower_bound(
          m_opacityPoints.begin(), m_opacityPoints.end(), newPt,
          [](const TransferFunctionWidget::OpacityPoint &a,
             const TransferFunctionWidget::OpacityPoint &b) { return a.value < b.value; });
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
    if (m_dragging && m_selectedPoint >= 0 &&
        m_selectedPoint < static_cast<int>(m_opacityPoints.size())) {
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
  int findNearestPoint(const QPoint &pos) const {
    if (m_opacityPoints.empty())
      return -1;

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

  double pointDistance(const QPoint &pos, int idx) const {
    if (idx < 0 || idx >= static_cast<int>(m_opacityPoints.size()))
      return 1e9;

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
    : QWidget(parent), m_presetCombo(nullptr), m_colorBarWidget(nullptr), m_opacityWidget(nullptr),
      m_volumeCombo(nullptr), m_sceneGraph(nullptr), m_dataMin(0.0), m_dataMax(1.0),
      m_updatingFromState(false) {
  setupUI();
  createDefaultTransferFunction();
}

TransferFunctionWidget::~TransferFunctionWidget() {}

void TransferFunctionWidget::setupUI() {
  QVBoxLayout *layout = new QVBoxLayout(this);

  // Volume selector
  QHBoxLayout *volumeLayout = new QHBoxLayout();
  volumeLayout->addWidget(new QLabel("Volume:"));
  m_volumeCombo = new QComboBox();
  connect(m_volumeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &TransferFunctionWidget::onVolumeSelected);
  volumeLayout->addWidget(m_volumeCombo);
  layout->addLayout(volumeLayout);

  // Preset selector
  QHBoxLayout *presetLayout = new QHBoxLayout();
  presetLayout->addWidget(new QLabel("Preset:"));
  m_presetCombo = new QComboBox();
  m_presetCombo->addItem("Grayscale");
  m_presetCombo->addItem("Rainbow");
  m_presetCombo->addItem("Hot");
  m_presetCombo->addItem("Cool");
  m_presetCombo->addItem("X-Ray");
  connect(m_presetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &TransferFunctionWidget::onPresetChanged);
  presetLayout->addWidget(m_presetCombo);
  layout->addLayout(presetLayout);

  // Color bar
  layout->addWidget(new QLabel("Color Map:"));
  m_colorBarWidget = new ColorBarWidget(this);
  layout->addWidget(m_colorBarWidget);

  // Opacity graph
  layout->addWidget(new QLabel("Opacity:"));
  m_opacityWidget = new OpacityGraphWidget(this);
  connect(static_cast<OpacityGraphWidget *>(m_opacityWidget), &OpacityGraphWidget::opacityChanged,
          this, &TransferFunctionWidget::onOpacityGraphChanged);
  layout->addWidget(m_opacityWidget);

  // Add instructions
  QLabel *instructions = new QLabel("Left-click to add/drag points, Right-click to remove");
  instructions->setStyleSheet("QLabel { color: gray; font-size: 9pt; }");
  layout->addWidget(instructions);

  layout->addStretch();
}

void TransferFunctionWidget::createDefaultTransferFunction() {
  applyPreset("Grayscale");

  // Initialize default opacity points if empty
  if (m_opacityPoints.empty() && m_opacityWidget != nullptr) {
    m_opacityPoints.push_back({0.0, 0.0});
    m_opacityPoints.push_back({1.0, 1.0});

    // Update the opacity widget
    auto opacityWidget = static_cast<OpacityGraphWidget *>(m_opacityWidget);
    opacityWidget->setOpacityPoints(m_opacityPoints);
  }
}

void TransferFunctionWidget::setSceneGraph(SceneGraph *sceneGraph) {
  m_sceneGraph = sceneGraph;
  refreshVolumeList();

  // Connect to state tree to monitor for new volumes
  // Listen to graphics root's children changes
  if (m_sceneGraph) {
    std::string statePrefix = m_sceneGraph->getStatePrefix();
    std::string graphicsRootPath = statePrefix + ".graphics.root.children";

    m_graphicsChildrenConnection =
        cvc::state::instance(volrover3::app())(graphicsRootPath)
            .childChanged.connect([this](const std::string &) {
              // Post to Qt event loop to ensure thread safety
              QMetaObject::invokeMethod(this, "onGraphicsChildrenChanged", Qt::QueuedConnection);
            });
  }
}

void TransferFunctionWidget::onGraphicsChildrenChanged() {
  if (!m_sceneGraph || !m_volumeCombo) {
    return;
  }

  // Get current volume list from scene graph
  auto volumeGraphics = m_sceneGraph->getAllVolumeGraphics();

  // Check if count changed
  if (volumeGraphics.size() != m_volumes.size()) {
    refreshVolumeList();
    return;
  }

  // Check if the set of volume names changed (handles reordering and replacement)
  std::set<std::string> newNames;
  for (const auto &vol : volumeGraphics) {
    if (vol) {
      newNames.insert(vol->getName());
    }
  }

  std::set<std::string> currentNames;
  for (const auto &vol : m_volumes) {
    if (vol) {
      currentNames.insert(vol->getName());
    }
  }

  if (newNames != currentNames) {
    refreshVolumeList();
    return;
  }
}

void TransferFunctionWidget::refreshVolumeList() {
  if (!m_sceneGraph || !m_volumeCombo) {
    return;
  }

  // Store current selection
  QString currentText = m_volumeCombo->currentText();

  // Clear and repopulate
  m_volumeCombo->clear();
  m_volumes.clear();

  auto volumeGraphics = m_sceneGraph->getAllVolumeGraphics();
  for (const auto &volNode : volumeGraphics) {
    if (volNode) {
      QString name = QString::fromStdString(volNode->getName());
      m_volumeCombo->addItem(name);
      m_volumes.push_back(volNode);
    }
  }

  // Restore selection if possible
  int index = m_volumeCombo->findText(currentText);
  if (index >= 0) {
    m_volumeCombo->setCurrentIndex(index);
  } else if (m_volumeCombo->count() > 0) {
    m_volumeCombo->setCurrentIndex(0);
    if (!m_volumes.empty()) {
      loadTransferFunctionFromVolume(m_volumes[0]);
    }
  }
}

std::shared_ptr<VolumeNode> TransferFunctionWidget::getSelectedVolume() const {
  int index = m_volumeCombo ? m_volumeCombo->currentIndex() : -1;
  if (index >= 0 && index < static_cast<int>(m_volumes.size())) {
    return m_volumes[index];
  }
  return nullptr;
}

void TransferFunctionWidget::onVolumeSelected(int index) {
  if (index >= 0 && index < static_cast<int>(m_volumes.size())) {
    auto volume = m_volumes[index];
    emit selectedVolumeChanged(volume);
    connectToVolumeState(volume);
    loadTransferFunctionFromVolume(volume);
  } else {
    disconnectFromVolumeState();
  }
}

void TransferFunctionWidget::loadTransferFunctionFromVolume(std::shared_ptr<VolumeNode> volume) {
  if (!volume || !volume->hasVolume()) {
    volrover3::app().log(
        0,
        "TransferFunctionWidget::loadTransferFunctionFromVolume: No volume or volume not loaded");
    return;
  }

  // Update data range from volume's metadata
  auto minVal = volume->getMetadata("data_min");
  auto maxVal = volume->getMetadata("data_max");

  volrover3::app().log(0, "\nTransferFunctionWidget::loadTransferFunctionFromVolume[" +
                              volume->getName() + "]: Getting metadata");

  if (minVal.has_value() && maxVal.has_value()) {
    try {
      // Try double first, then string
      if (minVal.type() == typeid(double)) {
        m_dataMin = std::any_cast<double>(minVal);
        m_dataMax = std::any_cast<double>(maxVal);
      } else {
        std::string minStr = std::any_cast<std::string>(minVal);
        std::string maxStr = std::any_cast<std::string>(maxVal);
        m_dataMin = std::stod(minStr);
        m_dataMax = std::stod(maxStr);
      }
      volrover3::app().log(0, "  Set data range to: [" + std::to_string(m_dataMin) + ", " +
                                  std::to_string(m_dataMax) + "]\n");
    } catch (const std::exception &e) {
      volrover3::app().log(0, "TransferFunctionWidget::loadTransferFunctionFromVolume[" +
                                  volume->getName() + "]: Failed to convert metadata (" +
                                  std::string(e.what()) + "), using defaults [0.0, 1.0]");
      m_dataMin = 0.0;
      m_dataMax = 1.0;
    }
  } else {
    volrover3::app().log(0, "TransferFunctionWidget::loadTransferFunctionFromVolume[" +
                                volume->getName() +
                                "]: No metadata found, using defaults [0.0, 1.0]");
    m_dataMin = 0.0;
    m_dataMax = 1.0;
  }

  // Load transfer function from volume's state tree
  std::vector<double> colorTable = volume->getTransferFunctionColorTable();
  std::vector<double> opacityTable = volume->getTransferFunctionOpacityTable();

  volrover3::app().log(0, "  Raw from state: " + std::to_string(colorTable.size()) +
                              " color values, " + std::to_string(opacityTable.size()) +
                              " opacity values");

  if (!colorTable.empty() && !opacityTable.empty()) {
    // Parse color table into color points (format: scalar, r, g, b, ...)
    m_colorPoints.clear();

    volrover3::app().log(0, "  Parsing color table: " + std::to_string(colorTable.size()) +
                                " values = " + std::to_string(colorTable.size() / 4) + " points");

    for (size_t i = 0; i + 3 < colorTable.size(); i += 4) {
      double scalar = colorTable[i];
      double r = colorTable[i + 1];
      double g = colorTable[i + 2];
      double b = colorTable[i + 3];

      // Convert to normalized value [0, 1]
      double normalizedValue =
          (m_dataMax > m_dataMin) ? (scalar - m_dataMin) / (m_dataMax - m_dataMin) : 0.0;

      m_colorPoints.push_back({normalizedValue, QColor::fromRgbF(r, g, b)});
    }

    // Parse opacity table into opacity points (format: scalar, opacity, ...)
    m_opacityPoints.clear();
    for (size_t i = 0; i + 1 < opacityTable.size(); i += 2) {
      double scalar = opacityTable[i];
      double opacity = opacityTable[i + 1];

      // Convert to normalized value [0, 1]
      double normalizedValue =
          (m_dataMax > m_dataMin) ? (scalar - m_dataMin) / (m_dataMax - m_dataMin) : 0.0;

      m_opacityPoints.push_back({normalizedValue, opacity});
    }

    // Update UI
    updateColorBar();
    if (m_opacityWidget) {
      auto opacityWidget = static_cast<OpacityGraphWidget *>(m_opacityWidget);
      opacityWidget->setOpacityPoints(m_opacityPoints);
    }

    volrover3::app().log(0, "  Loaded TF: " + std::to_string(m_colorPoints.size()) +
                                " color pts, " + std::to_string(m_opacityPoints.size()) +
                                " opacity pts");
  } else {
    volrover3::app().log(0, "  No transfer function in state, keeping current widget TF");
  }
}

void TransferFunctionWidget::applyPreset(const QString &presetName) {
  cvc::thread_info ti(volrover3::app(), "Applying transfer function preset");

  m_colorPoints.clear();
  // Don't clear opacity points - keep them independent!

  if (presetName == "Grayscale") {
    m_colorPoints.push_back({0.0, QColor(0, 0, 0)});
    m_colorPoints.push_back({1.0, QColor(255, 255, 255)});
  } else if (presetName == "Rainbow") {
    m_colorPoints.push_back({0.0, QColor(0, 0, 255)});    // Blue
    m_colorPoints.push_back({0.25, QColor(0, 255, 255)}); // Cyan
    m_colorPoints.push_back({0.5, QColor(0, 255, 0)});    // Green
    m_colorPoints.push_back({0.75, QColor(255, 255, 0)}); // Yellow
    m_colorPoints.push_back({1.0, QColor(255, 0, 0)});    // Red
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

  // Apply to selected volume
  if (m_updatingFromState == 0) {
    auto selectedVolume = getSelectedVolume();
    if (selectedVolume) {
      m_updatingFromState++; // Increment before calling to prevent feedback
      selectedVolume->setTransferFunction(getColorTable(), getOpacityTable());
      m_updatingFromState--; // Decrement after
    }
  }

  emit transferFunctionChanged();
}

void TransferFunctionWidget::setDataRange(double min, double max) {
  m_dataMin = min;
  m_dataMax = max;
}

void TransferFunctionWidget::onPresetChanged(int index) {
  applyPreset(m_presetCombo->currentText());
}

void TransferFunctionWidget::onColorMapClicked(double x, double y) {
  // Placeholder for adding color control points
}

void TransferFunctionWidget::onOpacityGraphChanged() {
  if (m_updatingFromState == 0) {
    // Apply to selected volume
    auto selectedVolume = getSelectedVolume();
    if (selectedVolume) {
      m_updatingFromState++; // Increment before calling to prevent feedback
      selectedVolume->setTransferFunction(getColorTable(), getOpacityTable());
      m_updatingFromState--; // Decrement after
    }
    emit transferFunctionChanged();
  }
}

void TransferFunctionWidget::updateColorBar() {
  static_cast<ColorBarWidget *>(m_colorBarWidget)->setColorPoints(m_colorPoints);
  // Don't reset opacity points - they're managed independently by the opacity widget
}

std::vector<double> TransferFunctionWidget::getColorTable() const {
  std::vector<double> table;

  volrover3::app().log(0, "TransferFunctionWidget::getColorTable() - m_colorPoints.size() = " +
                              std::to_string(m_colorPoints.size()));

  for (const auto &pt : m_colorPoints) {
    double scalar = m_dataMin + pt.value * (m_dataMax - m_dataMin);
    table.push_back(scalar);
    table.push_back(pt.color.redF());
    table.push_back(pt.color.greenF());
    table.push_back(pt.color.blueF());
  }

  volrover3::app().log(0, "  Returning color table with " + std::to_string(table.size()) +
                              " values (" + std::to_string(table.size() / 4) + " points)");

  return table;
}

std::vector<double> TransferFunctionWidget::getOpacityTable() const {
  std::vector<double> table;

  // Get the current opacity points from the opacity widget
  const auto &opacityPoints =
      static_cast<OpacityGraphWidget *>(m_opacityWidget)->getOpacityPoints();

  for (const auto &pt : opacityPoints) {
    double scalar = m_dataMin + pt.value * (m_dataMax - m_dataMin);
    table.push_back(scalar);
    table.push_back(pt.opacity);
  }

  return table;
}

void TransferFunctionWidget::connectToVolumeState(std::shared_ptr<VolumeNode> volume) {
  disconnectFromVolumeState();

  if (!volume) {
    return;
  }

  // Connect to transfer function state changes
  std::string statePath = volume->getState().fullName();

  auto colorTFPath = statePath + ".transfer_function.color";
  auto opacityTFPath = statePath + ".transfer_function.opacity";

  // Use DirectConnection instead of QueuedConnection to ensure m_updatingFromState flag works
  // correctly
  m_colorTFConnection =
      cvc::state::instance(volrover3::app())(colorTFPath).valueChanged.connect([this]() {
        // Only reload if we're not currently updating state from widget
        if (m_updatingFromState == 0) {
          onVolumeTransferFunctionChanged();
        }
      });

  m_opacityTFConnection =
      cvc::state::instance(volrover3::app())(opacityTFPath).valueChanged.connect([this]() {
        // Only reload if we're not currently updating state from widget
        if (m_updatingFromState == 0) {
          onVolumeTransferFunctionChanged();
        }
      });
}

void TransferFunctionWidget::disconnectFromVolumeState() {
  m_colorTFConnection.disconnect();
  m_opacityTFConnection.disconnect();
}

void TransferFunctionWidget::onVolumeTransferFunctionChanged() {
  // Reload transfer function from selected volume's state
  auto volume = getSelectedVolume();
  if (volume && m_updatingFromState == 0) {
    m_updatingFromState++;
    loadTransferFunctionFromVolume(volume);
    m_updatingFromState--;
  }
}

#include "TransferFunctionWidget.moc"
