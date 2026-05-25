#ifndef STATEDASHBOARDWIDGET_H
#define STATEDASHBOARDWIDGET_H

#include <QComboBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QSplitter>
#include <QTabWidget>
#include <QTableWidget>
#include <QTimer>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QWidget>
#include <boost/signals2.hpp>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_membership.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_exec/exec_coordinator.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_telemetry_aggregator.h>
#include <string>
#include <vector>

/// Comprehensive state management dashboard combining tree navigation,
/// DSL execution console, and cluster networking in a tabbed interface.
class StateDashboardWidget : public QWidget {
  Q_OBJECT

public:
  explicit StateDashboardWidget(QWidget *parent = nullptr);
  ~StateDashboardWidget() override;

  /// Set the root state node for the tree view.
  void setRootState(cvc::state *root);

  /// Attach a scheduler for process management in the exec tab.
  void setScheduler(cvc::state_exec::scheduler *sched);

  /// Attach a cluster shard for distributed networking in the cluster tab.
  void setShard(cvc::state_cluster_shard *shard);

  /// Attach a membership manager for peer management.
  void setMembership(cvc::state_cluster_membership *membership);

  /// Attach a coordinator for distributed exec management.
  void setCoordinator(cvc::state_exec::exec_coordinator *coord);

  /// Attach a telemetry aggregator for cluster stats.
  void setTelemetryAggregator(cvc::state_telemetry_aggregator *agg);

  /// Refresh all tabs.
  void refresh();

signals:
  void stateChanged();

  // ─── State Tree tab ────────────────────────────────────────────────
private slots:
  void onTreeItemSelected();
  void onTreeSearchTextChanged(const QString &text);
  void onPropertyValueChanged(int row, int column);
  void onAddStateClicked();
  void onDeleteStateClicked();
  void onTreeStructureChanged();
  void onCurrentStateChanged();
  void onCurrentStateDestroyed();

private:
  void buildStateTreeTab(QTabWidget *tabs);
  void refreshStateTree();
  void populateTree(QTreeWidgetItem *parentItem, cvc::state *state);
  void populateProperties(cvc::state *state);
  QTreeWidgetItem *findTreeItem(QTreeWidgetItem *parent, cvc::state *target);
  std::string getStateValue(cvc::state *state);
  void setStateValue(cvc::state *state, const QString &valueStr);

  QLineEdit *m_treeSearch;
  QTreeWidget *m_treeWidget;
  QTableWidget *m_propertyTable;
  QPushButton *m_addStateBtn;
  QPushButton *m_deleteStateBtn;

  cvc::state *m_rootState = nullptr;
  cvc::state *m_currentState = nullptr;

  boost::signals2::connection m_valueConn;
  boost::signals2::connection m_treeConn;
  boost::signals2::connection m_destroyConn;

  // ─── State Exec Console tab ────────────────────────────────────────
private slots:
  void onRunScriptClicked();
  void onClearOutputClicked();
  void onProcessTableSelectionChanged();
  void onPauseProcessClicked();
  void onResumeProcessClicked();
  void onKillProcessClicked();
  void refreshProcessList();

private:
  void buildExecConsoleTab(QTabWidget *tabs);

  QPlainTextEdit *m_scriptEditor;
  QPushButton *m_runBtn;
  QPushButton *m_clearOutputBtn;
  QPlainTextEdit *m_outputPanel;
  QTableWidget *m_processTable;
  QPushButton *m_pauseBtn;
  QPushButton *m_resumeBtn;
  QPushButton *m_killBtn;
  QTimer *m_processRefreshTimer;

  cvc::state_exec::scheduler *m_scheduler = nullptr;

  // ─── Cluster & Networking tab ──────────────────────────────────────
private slots:
  void onConnectPeerClicked();
  void refreshClusterInfo();

private:
  void buildClusterTab(QTabWidget *tabs);

  QLabel *m_nodeIdLabel;
  QLabel *m_clusterIdLabel;
  QLabel *m_leaderLabel;
  QTableWidget *m_peerTable;
  QLineEdit *m_peerEndpointInput;
  QPushButton *m_connectPeerBtn;
  QLabel *m_busStatsLabel;
  QLabel *m_shardStatsLabel;
  QLabel *m_telemetryLabel;
  QTimer *m_clusterRefreshTimer;

  cvc::state_cluster_shard *m_shard = nullptr;
  cvc::state_cluster_membership *m_membership = nullptr;
  cvc::state_exec::exec_coordinator *m_coordinator = nullptr;
  cvc::state_telemetry_aggregator *m_telemetryAgg = nullptr;
};

#endif // STATEDASHBOARDWIDGET_H
