#ifndef STATETREEWIDGET_H
#define STATETREEWIDGET_H

#include <QPushButton>
#include <QSplitter>
#include <QTableWidget>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <boost/signals2.hpp>
#include <cvc/core/state.h>
#include <string>
#include <vector>

class StateTreeWidget : public QWidget {
  Q_OBJECT

public:
  explicit StateTreeWidget(QWidget *parent = nullptr);
  ~StateTreeWidget() override;

  void setRootState(cvc::state *root);
  void refresh();

signals:
  void stateChanged(); // Emitted when state data is modified

private slots:
  void onTreeItemSelected();
  void onTableValueChanged(int row, int column);
  void onAddStateClicked();
  void onDeleteStateClicked();
  void onCurrentStateChanged();
  void onTreeStructureChanged();
  void onCurrentStateDestroyed();

private:
  void populateTree(QTreeWidgetItem *parentItem, cvc::state *state, const std::string &path);
  void populateTable(cvc::state *state);
  std::string getStateValue(cvc::state *state);
  std::string getStateDataType(cvc::state *state);
  void setStateValue(cvc::state *state, const QString &valueStr);
  QTreeWidgetItem *findTreeItem(QTreeWidgetItem *parent, cvc::state *state);

  QTreeWidget *m_treeWidget;
  QTableWidget *m_tableWidget;
  QPushButton *m_addButton;
  QPushButton *m_deleteButton;

  // Non-owning pointers to states (owned by the cvc::state singleton tree)
  // We use the destroyed signal to track when states are deleted
  cvc::state *m_rootState;
  cvc::state *m_currentState;

  boost::signals2::connection m_stateChangeConnection;
  boost::signals2::connection m_treeChangeConnection;
  boost::signals2::connection m_currentStateDestroyedConnection;
};

#endif // STATETREEWIDGET_H
