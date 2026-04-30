#ifndef BOUNDINGBOXDIALOG_H
#define BOUNDINGBOXDIALOG_H

#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <cvc/bounding_box.h>
#include <memory>
#include <vector>

class GraphicsNode;
class SceneGraph;

class BoundingBoxDialog : public QDialog {
  Q_OBJECT

public:
  explicit BoundingBoxDialog(std::shared_ptr<SceneGraph> sceneGraph, QWidget *parent = nullptr);

private slots:
  void onGraphicsSelectionChanged(int index);
  void onResetToGraphics();
  void onBBoxVisibilityChanged(bool visible);
  void onBBoxColorChanged();
  void onApplyChanges();

private:
  void setupUI();
  void populateGraphicsComboBox();
  void loadGraphicsSettings();
  void updateColorButton();

  std::shared_ptr<SceneGraph> m_sceneGraph;
  std::vector<std::shared_ptr<GraphicsNode>> m_graphicsList;
  std::shared_ptr<GraphicsNode> m_currentGraphics;

  QComboBox *m_graphicsComboBox;
  QLineEdit *m_minXEdit;
  QLineEdit *m_minYEdit;
  QLineEdit *m_minZEdit;
  QLineEdit *m_maxXEdit;
  QLineEdit *m_maxYEdit;
  QLineEdit *m_maxZEdit;

  QCheckBox *m_bboxVisibleCheckbox;
  QPushButton *m_bboxColorButton;
  double m_bboxColor[3];
};

#endif // BOUNDINGBOXDIALOG_H
