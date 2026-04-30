#ifndef VIEWEROPTIONSDIALOG_H
#define VIEWEROPTIONSDIALOG_H

#include <QWidget>
#include <boost/signals2.hpp>
#include <memory>

class QCheckBox;
class QComboBox;
class QPushButton;
class VTKRenderWidget;
class SceneGraph;

class ViewerOptionsDialog : public QWidget {
  Q_OBJECT

public:
  explicit ViewerOptionsDialog(VTKRenderWidget *renderWidget,
                               std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);
  ~ViewerOptionsDialog() override;

protected:
  void showEvent(QShowEvent *event) override;
  void closeEvent(QCloseEvent *event) override;

private slots:
  void onShowFPSChanged(bool checked);
  void onGraphicsRootChanged(int index);
  void onCameraChanged(int index);
  void refreshGraphicsRoots();
  void refreshCameras();

private:
  void setupUI();
  void connectSignals();
  void loadFromState();

  VTKRenderWidget *m_renderWidget;
  std::shared_ptr<SceneGraph> m_sceneGraph;

  // Display options
  QCheckBox *m_showFPSCheckBox;

  // Scene selection
  QComboBox *m_graphicsRootComboBox;
  QPushButton *m_refreshRootsButton;

  // Camera selection
  QComboBox *m_cameraComboBox;
  QPushButton *m_refreshCamerasButton;

  std::vector<boost::signals2::connection> m_connections;
};

#endif // VIEWEROPTIONSDIALOG_H
