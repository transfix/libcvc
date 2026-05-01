#ifndef SDFDIALOG_H
#define SDFDIALOG_H

#include <QDialog>
#include <boost/signals2.hpp>
#include <memory>
#include <string>

class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;
class QProgressBar;
class QPushButton;
class QLabel;
class SceneGraph;

class SDFDialog : public QDialog {
  Q_OBJECT

public:
  explicit SDFDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);
  ~SDFDialog() override = default;

private slots:
  void onComputeClicked();
  void onGeometrySelected(int index);
  void updateProgress(int value);
  void onComputeFinished(bool success, const std::string &message);
  void onGraphicsChildrenChanged();

private:
  void setupUI();
  void connectSignals();
  void populateGeometryList();
  void setControlsEnabled(bool enabled);

  std::shared_ptr<SceneGraph> m_sceneGraph;

  // UI controls
  QComboBox *m_geometryComboBox;
  QSpinBox *m_dimXSpinBox;
  QSpinBox *m_dimYSpinBox;
  QSpinBox *m_dimZSpinBox;
  QComboBox *m_algorithmComboBox;
  QCheckBox *m_flipNormalsCheckBox;
  QCheckBox *m_useBoundsCheckBox;
  QDoubleSpinBox *m_minXSpinBox;
  QDoubleSpinBox *m_minYSpinBox;
  QDoubleSpinBox *m_minZSpinBox;
  QDoubleSpinBox *m_maxXSpinBox;
  QDoubleSpinBox *m_maxYSpinBox;
  QDoubleSpinBox *m_maxZSpinBox;
  QPushButton *m_computeButton;
  QPushButton *m_cancelButton;
  QProgressBar *m_progressBar;
  QLabel *m_statusLabel;

  // Geometry tracking
  std::vector<std::string> m_geometryNames;

  // State tree connection for monitoring geometry changes
  boost::signals2::scoped_connection m_graphicsChildrenConnection;

  // Thread tracking
  bool m_computing;
  std::string m_activeThreadKey;
};

#endif // SDFDIALOG_H
