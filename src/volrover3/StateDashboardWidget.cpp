#include <QApplication>
#include <QFont>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QInputDialog>
#include <QMessageBox>
#include <QMetaObject>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/evaluator.h>
#include <cvc/core/state_exec/process.h>
#include <sstream>
#include <volrover3/StateDashboardWidget.h>
#include <volrover3/volrover3_app.h>

using namespace cvc;
using namespace cvc::state_exec;

// ═══════════════════════════════════════════════════════════════════════════
// Construction
// ═══════════════════════════════════════════════════════════════════════════

StateDashboardWidget::StateDashboardWidget(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(4, 4, 4, 4);

  auto *tabs = new QTabWidget(this);
  buildStateTreeTab(tabs);
  buildExecConsoleTab(tabs);
  buildClusterTab(tabs);
  layout->addWidget(tabs);
}

StateDashboardWidget::~StateDashboardWidget() {
  m_valueConn.disconnect();
  m_treeConn.disconnect();
  m_destroyConn.disconnect();

  if (m_processRefreshTimer)
    m_processRefreshTimer->stop();
  if (m_clusterRefreshTimer)
    m_clusterRefreshTimer->stop();
}

// ═══════════════════════════════════════════════════════════════════════════
// Public setters
// ═══════════════════════════════════════════════════════════════════════════

void StateDashboardWidget::setRootState(state *root) {
  m_valueConn.disconnect();
  m_treeConn.disconnect();
  m_destroyConn.disconnect();
  m_currentState = nullptr;

  m_rootState = root;
  if (m_rootState) {
    m_treeConn = m_rootState->childChanged.connect([this](const std::string &) {
      QMetaObject::invokeMethod(this, "onTreeStructureChanged", Qt::QueuedConnection);
    });
  }
  refreshStateTree();
}

void StateDashboardWidget::setScheduler(scheduler *sched) {
  m_scheduler = sched;
  refreshProcessList();
}

void StateDashboardWidget::setShard(state_cluster_shard *shard) { m_shard = shard; }

void StateDashboardWidget::setMembership(state_cluster_membership *membership) {
  m_membership = membership;
  refreshClusterInfo();
}

void StateDashboardWidget::setCoordinator(exec_coordinator *coord) { m_coordinator = coord; }

void StateDashboardWidget::setTelemetryAggregator(state_telemetry_aggregator *agg) {
  m_telemetryAgg = agg;
}

void StateDashboardWidget::refresh() {
  refreshStateTree();
  refreshProcessList();
  refreshClusterInfo();
}

// ═══════════════════════════════════════════════════════════════════════════
// Tab 1 — State Tree
// ═══════════════════════════════════════════════════════════════════════════

void StateDashboardWidget::buildStateTreeTab(QTabWidget *tabs) {
  auto *page = new QWidget;
  auto *layout = new QVBoxLayout(page);
  layout->setContentsMargins(2, 2, 2, 2);

  // Search bar
  m_treeSearch = new QLineEdit;
  m_treeSearch->setPlaceholderText(tr("Filter state tree..."));
  connect(m_treeSearch, &QLineEdit::textChanged, this,
          &StateDashboardWidget::onTreeSearchTextChanged);
  layout->addWidget(m_treeSearch);

  // Splitter: tree left, properties right
  auto *splitter = new QSplitter(Qt::Horizontal);

  // Tree widget
  m_treeWidget = new QTreeWidget;
  m_treeWidget->setHeaderLabel(tr("State Tree"));
  m_treeWidget->setSelectionMode(QAbstractItemView::SingleSelection);
  connect(m_treeWidget, &QTreeWidget::itemSelectionChanged, this,
          &StateDashboardWidget::onTreeItemSelected);
  splitter->addWidget(m_treeWidget);

  // Right side: property table + buttons
  auto *rightPanel = new QWidget;
  auto *rightLayout = new QVBoxLayout(rightPanel);
  rightLayout->setContentsMargins(0, 0, 0, 0);

  m_propertyTable = new QTableWidget;
  m_propertyTable->setColumnCount(2);
  m_propertyTable->setHorizontalHeaderLabels({tr("Property"), tr("Value")});
  m_propertyTable->horizontalHeader()->setStretchLastSection(true);
  m_propertyTable->verticalHeader()->hide();
  m_propertyTable->setEditTriggers(QAbstractItemView::DoubleClicked |
                                   QAbstractItemView::SelectedClicked);
  connect(m_propertyTable, &QTableWidget::cellChanged, this,
          &StateDashboardWidget::onPropertyValueChanged);
  rightLayout->addWidget(m_propertyTable);

  auto *btnBar = new QHBoxLayout;
  m_addStateBtn = new QPushButton(tr("Add State"));
  m_deleteStateBtn = new QPushButton(tr("Delete State"));
  m_deleteStateBtn->setEnabled(false);
  connect(m_addStateBtn, &QPushButton::clicked, this, &StateDashboardWidget::onAddStateClicked);
  connect(m_deleteStateBtn, &QPushButton::clicked, this,
          &StateDashboardWidget::onDeleteStateClicked);
  btnBar->addWidget(m_addStateBtn);
  btnBar->addWidget(m_deleteStateBtn);
  btnBar->addStretch();
  rightLayout->addLayout(btnBar);

  splitter->addWidget(rightPanel);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  layout->addWidget(splitter);

  tabs->addTab(page, tr("State Tree"));
}

void StateDashboardWidget::refreshStateTree() {
  m_treeWidget->clear();
  if (!m_rootState)
    return;

  auto *root = new QTreeWidgetItem(m_treeWidget);
  root->setText(0, QString::fromStdString(m_rootState->name()));
  root->setData(0, Qt::UserRole, QVariant::fromValue(static_cast<void *>(m_rootState)));
  populateTree(root, m_rootState);
  root->setExpanded(true);
}

void StateDashboardWidget::populateTree(QTreeWidgetItem *parentItem, state *s) {
  auto childPaths = s->children();
  std::string parentFull = s->fullName();

  for (auto &childFullName : childPaths) {
    // Only immediate children: relative path has no dots
    std::string rel = childFullName.substr(parentFull.size());
    if (!rel.empty() && rel[0] == '.')
      rel = rel.substr(1);
    if (rel.empty() || rel.find('.') != std::string::npos)
      continue;

    auto &child = (*s)(rel);
    if (!child.initialized())
      continue;

    auto *item = new QTreeWidgetItem(parentItem);
    item->setText(0, QString::fromStdString(rel));
    item->setData(0, Qt::UserRole, QVariant::fromValue(static_cast<void *>(&child)));
    populateTree(item, &child);
  }
}

void StateDashboardWidget::onTreeItemSelected() {
  m_valueConn.disconnect();
  m_destroyConn.disconnect();
  m_currentState = nullptr;

  auto items = m_treeWidget->selectedItems();
  if (items.isEmpty()) {
    m_propertyTable->setRowCount(0);
    m_deleteStateBtn->setEnabled(false);
    return;
  }

  auto *ptr = static_cast<state *>(items[0]->data(0, Qt::UserRole).value<void *>());
  if (!ptr)
    return;

  m_currentState = ptr;
  m_deleteStateBtn->setEnabled(true);

  // Listen for value changes on the selected node
  m_valueConn = m_currentState->valueChanged.connect(
      [this]() { QMetaObject::invokeMethod(this, "onCurrentStateChanged", Qt::QueuedConnection); });
  m_destroyConn = m_currentState->destroyed.connect([this]() {
    QMetaObject::invokeMethod(this, "onCurrentStateDestroyed", Qt::QueuedConnection);
  });

  populateProperties(m_currentState);
}

void StateDashboardWidget::populateProperties(state *s) {
  m_propertyTable->blockSignals(true);
  m_propertyTable->setRowCount(0);

  auto addRow = [this](const QString &key, const QString &val, bool editable = false) {
    int row = m_propertyTable->rowCount();
    m_propertyTable->insertRow(row);
    auto *keyItem = new QTableWidgetItem(key);
    keyItem->setFlags(keyItem->flags() & ~Qt::ItemIsEditable);
    m_propertyTable->setItem(row, 0, keyItem);
    auto *valItem = new QTableWidgetItem(val);
    if (!editable)
      valItem->setFlags(valItem->flags() & ~Qt::ItemIsEditable);
    m_propertyTable->setItem(row, 1, valItem);
  };

  addRow(tr("Name"), QString::fromStdString(s->name()));
  addRow(tr("Full Path"), QString::fromStdString(s->fullName()));
  addRow(tr("Value"), QString::fromStdString(s->value()), !s->readOnly());
  addRow(tr("Value Type"), QString::fromStdString(s->valueTypeName()));
  addRow(tr("Data Type"), QString::fromStdString(s->dataTypeName()));
  addRow(tr("Read Only"), s->readOnly() ? tr("true") : tr("false"));
  addRow(tr("Hidden"), s->hidden() ? tr("true") : tr("false"));
  addRow(tr("Comment"), QString::fromStdString(s->comment()), true);

  // Last modified
  auto lastMod = s->lastMod();
  if (!lastMod.is_not_a_date_time())
    addRow(tr("Last Modified"),
           QString::fromStdString(boost::posix_time::to_simple_string(lastMod)));
  else
    addRow(tr("Last Modified"), tr("(unknown)"));

  addRow(tr("Children"), QString::number(s->numChildren()));
  addRow(tr("Initialized"), s->initialized() ? tr("true") : tr("false"));

  // Link info
  addRow(tr("Is Link"), s->isLink() ? tr("true") : tr("false"));
  if (s->isLink()) {
    addRow(tr("Link Target"), QString::fromStdString(s->linkTarget()));
    addRow(tr("Link Mode"),
           s->linkMode() == state::link_mode::transparent ? tr("transparent") : tr("opaque"));
    addRow(tr("Link Writable"), s->linkWritable() ? tr("true") : tr("false"));

    auto res = s->resolveLink();
    QString kindStr;
    switch (res.kind) {
    case state::link_resolution_kind::resolved:
      kindStr = tr("resolved");
      break;
    case state::link_resolution_kind::cycle_detected:
      kindStr = tr("cycle");
      break;
    case state::link_resolution_kind::budget_exhausted:
      kindStr = tr("budget exhausted");
      break;
    case state::link_resolution_kind::broken:
      kindStr = tr("broken");
      break;
    case state::link_resolution_kind::none:
      kindStr = tr("none");
      break;
    }
    addRow(tr("Link Resolution"), kindStr);
    addRow(tr("Resolved Value"), QString::fromStdString(s->resolvedValue()));
  }

  // Expiry info
  addRow(tr("Has Expiry"), s->hasExpiry() ? tr("true") : tr("false"));
  if (s->hasExpiry()) {
    auto exp = s->expiryTime();
    addRow(tr("Expiry Time"), QString::fromStdString(boost::posix_time::to_simple_string(exp)));
    addRow(tr("Expired"), s->isExpired() ? tr("true") : tr("false"));
  }

  m_propertyTable->blockSignals(false);
}

void StateDashboardWidget::onPropertyValueChanged(int row, int column) {
  if (column != 1 || !m_currentState)
    return;

  auto *keyItem = m_propertyTable->item(row, 0);
  if (!keyItem)
    return;

  QString key = keyItem->text();
  QString val = m_propertyTable->item(row, 1)->text();

  if (key == tr("Value") && !m_currentState->readOnly()) {
    setStateValue(m_currentState, val);
    emit stateChanged();
  } else if (key == tr("Comment")) {
    m_currentState->comment(val.toStdString());
  }
}

void StateDashboardWidget::onTreeSearchTextChanged(const QString &text) {
  // Show/hide tree items based on filter text
  std::function<bool(QTreeWidgetItem *)> filterItem = [&](QTreeWidgetItem *item) -> bool {
    bool childVisible = false;
    for (int i = 0; i < item->childCount(); ++i) {
      if (filterItem(item->child(i)))
        childVisible = true;
    }
    bool selfMatch = text.isEmpty() || item->text(0).contains(text, Qt::CaseInsensitive);
    bool visible = selfMatch || childVisible;
    item->setHidden(!visible);
    if (visible && !text.isEmpty())
      item->setExpanded(true);
    return visible;
  };

  for (int i = 0; i < m_treeWidget->topLevelItemCount(); ++i)
    filterItem(m_treeWidget->topLevelItem(i));
}

void StateDashboardWidget::onAddStateClicked() {
  bool ok;
  QString path = QInputDialog::getText(this, tr("Add State"), tr("State path (dot-separated):"),
                                       QLineEdit::Normal, QString(), &ok);
  if (!ok || path.isEmpty())
    return;

  std::string pathStr = path.toStdString();
  if (!state::isValidStateName(pathStr)) {
    std::string sanitized = state::sanitizeStateName(pathStr);
    if (sanitized.empty()) {
      QMessageBox::warning(this, tr("Invalid Path"), tr("The path is not valid."));
      return;
    }
    pathStr = sanitized;
  }

  state *parent = m_currentState ? m_currentState : m_rootState;
  if (!parent)
    return;

  (*parent)(pathStr).value(std::string(""));
  emit stateChanged();
}

void StateDashboardWidget::onDeleteStateClicked() {
  if (!m_currentState || m_currentState == m_rootState)
    return;

  auto reply = QMessageBox::question(this, tr("Delete State"),
                                     tr("Reset state '%1' and all children?")
                                         .arg(QString::fromStdString(m_currentState->fullName())),
                                     QMessageBox::Yes | QMessageBox::No);

  if (reply == QMessageBox::Yes) {
    m_currentState->reset();
    emit stateChanged();
  }
}

void StateDashboardWidget::onTreeStructureChanged() { refreshStateTree(); }

void StateDashboardWidget::onCurrentStateChanged() {
  if (m_currentState)
    populateProperties(m_currentState);
}

void StateDashboardWidget::onCurrentStateDestroyed() {
  m_valueConn.disconnect();
  m_destroyConn.disconnect();
  m_currentState = nullptr;
  m_propertyTable->setRowCount(0);
  m_deleteStateBtn->setEnabled(false);
  refreshStateTree();
}

std::string StateDashboardWidget::getStateValue(state *s) { return s->value(); }

void StateDashboardWidget::setStateValue(state *s, const QString &valueStr) {
  std::string typeName = s->valueTypeName();
  std::string v = valueStr.toStdString();

  if (typeName == "double")
    s->value(valueStr.toDouble());
  else if (typeName == "float")
    s->value(valueStr.toFloat());
  else if (typeName == "int")
    s->value(valueStr.toInt());
  else if (typeName == "unsigned int")
    s->value(static_cast<unsigned int>(valueStr.toUInt()));
  else if (typeName == "bool")
    s->value(v == "true" || v == "1");
  else if (typeName == "long")
    s->value(valueStr.toLong());
  else if (typeName == "unsigned long" || typeName == "size_t")
    s->value(static_cast<std::size_t>(valueStr.toULong()));
  else
    s->value(v);
}

QTreeWidgetItem *StateDashboardWidget::findTreeItem(QTreeWidgetItem *parent, state *target) {
  for (int i = 0; i < parent->childCount(); ++i) {
    auto *child = parent->child(i);
    auto *ptr = static_cast<state *>(child->data(0, Qt::UserRole).value<void *>());
    if (ptr == target)
      return child;
    auto *found = findTreeItem(child, target);
    if (found)
      return found;
  }
  return nullptr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Tab 2 — State Exec Console
// ═══════════════════════════════════════════════════════════════════════════

void StateDashboardWidget::buildExecConsoleTab(QTabWidget *tabs) {
  auto *page = new QWidget;
  auto *layout = new QVBoxLayout(page);
  layout->setContentsMargins(2, 2, 2, 2);

  auto *splitter = new QSplitter(Qt::Vertical);

  // Top: script editor + run button
  auto *editorGroup = new QGroupBox(tr("S-Expression Script"));
  auto *editorLayout = new QVBoxLayout(editorGroup);

  m_scriptEditor = new QPlainTextEdit;
  m_scriptEditor->setPlaceholderText(tr("Enter state_exec program...\ne.g. (+ 1 2 3)"));
  QFont mono("monospace");
  mono.setStyleHint(QFont::Monospace);
  m_scriptEditor->setFont(mono);
  m_scriptEditor->setTabStopDistance(QFontMetrics(mono).horizontalAdvance(' ') * 2);
  editorLayout->addWidget(m_scriptEditor);

  auto *editorBtnBar = new QHBoxLayout;
  m_runBtn = new QPushButton(tr("Run"));
  m_clearOutputBtn = new QPushButton(tr("Clear Output"));
  connect(m_runBtn, &QPushButton::clicked, this, &StateDashboardWidget::onRunScriptClicked);
  connect(m_clearOutputBtn, &QPushButton::clicked, this,
          &StateDashboardWidget::onClearOutputClicked);
  editorBtnBar->addWidget(m_runBtn);
  editorBtnBar->addWidget(m_clearOutputBtn);
  editorBtnBar->addStretch();
  editorLayout->addLayout(editorBtnBar);
  splitter->addWidget(editorGroup);

  // Middle: output panel
  auto *outputGroup = new QGroupBox(tr("Output"));
  auto *outputLayout = new QVBoxLayout(outputGroup);
  m_outputPanel = new QPlainTextEdit;
  m_outputPanel->setReadOnly(true);
  m_outputPanel->setFont(mono);
  outputLayout->addWidget(m_outputPanel);
  splitter->addWidget(outputGroup);

  // Bottom: process list
  auto *processGroup = new QGroupBox(tr("Processes"));
  auto *processLayout = new QVBoxLayout(processGroup);

  m_processTable = new QTableWidget;
  m_processTable->setColumnCount(8);
  m_processTable->setHorizontalHeaderLabels({tr("PID"), tr("Name"), tr("Status"), tr("Priority"),
                                             tr("UID"), tr("Steps"), tr("Elapsed"), tr("Memory")});
  m_processTable->horizontalHeader()->setStretchLastSection(true);
  m_processTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_processTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  connect(m_processTable, &QTableWidget::itemSelectionChanged, this,
          &StateDashboardWidget::onProcessTableSelectionChanged);
  processLayout->addWidget(m_processTable);

  auto *procBtnBar = new QHBoxLayout;
  m_pauseBtn = new QPushButton(tr("Pause"));
  m_resumeBtn = new QPushButton(tr("Resume"));
  m_killBtn = new QPushButton(tr("Kill"));
  m_pauseBtn->setEnabled(false);
  m_resumeBtn->setEnabled(false);
  m_killBtn->setEnabled(false);
  connect(m_pauseBtn, &QPushButton::clicked, this, &StateDashboardWidget::onPauseProcessClicked);
  connect(m_resumeBtn, &QPushButton::clicked, this, &StateDashboardWidget::onResumeProcessClicked);
  connect(m_killBtn, &QPushButton::clicked, this, &StateDashboardWidget::onKillProcessClicked);
  procBtnBar->addWidget(m_pauseBtn);
  procBtnBar->addWidget(m_resumeBtn);
  procBtnBar->addWidget(m_killBtn);
  procBtnBar->addStretch();
  processLayout->addLayout(procBtnBar);

  splitter->addWidget(processGroup);
  splitter->setStretchFactor(0, 2);
  splitter->setStretchFactor(1, 1);
  splitter->setStretchFactor(2, 2);
  layout->addWidget(splitter);

  // Timer for auto-refreshing process list
  m_processRefreshTimer = new QTimer(this);
  m_processRefreshTimer->setInterval(1000);
  connect(m_processRefreshTimer, &QTimer::timeout, this, &StateDashboardWidget::refreshProcessList);
  m_processRefreshTimer->start();

  tabs->addTab(page, tr("Exec Console"));
}

void StateDashboardWidget::onRunScriptClicked() {
  QString script = m_scriptEditor->toPlainText().trimmed();
  if (script.isEmpty())
    return;

  m_outputPanel->appendPlainText(QStringLiteral("> ") + script);

  try {
    auto env = builtins::make_default_environment();
    evaluator ev(env);
    auto result = ev.evaluate_script(script.toStdString());
    m_outputPanel->appendPlainText(QString::fromStdString(to_string(result)));
  } catch (const std::exception &e) {
    m_outputPanel->appendPlainText(QStringLiteral("ERROR: ") + QString::fromUtf8(e.what()));
  }

  // Also submit to scheduler if available
  if (m_scheduler) {
    refreshProcessList();
  }
}

void StateDashboardWidget::onClearOutputClicked() { m_outputPanel->clear(); }

void StateDashboardWidget::refreshProcessList() {
  if (!m_scheduler) {
    m_processTable->setRowCount(0);
    return;
  }

  auto procs = m_scheduler->list_processes();
  m_processTable->setRowCount(static_cast<int>(procs.size()));

  for (int i = 0; i < static_cast<int>(procs.size()); ++i) {
    auto &p = procs[static_cast<size_t>(i)];
    auto setCell = [this, i](int col, const QString &text) {
      auto *item = new QTableWidgetItem(text);
      item->setFlags(item->flags() & ~Qt::ItemIsEditable);
      m_processTable->setItem(i, col, item);
    };

    setCell(0, QString::number(p.pid));
    setCell(1, QString::fromStdString(p.name));

    QString statusStr;
    switch (p.status) {
    case process_status::ready:
      statusStr = tr("ready");
      break;
    case process_status::running:
      statusStr = tr("running");
      break;
    case process_status::paused:
      statusStr = tr("paused");
      break;
    case process_status::waiting:
      statusStr = tr("waiting");
      break;
    case process_status::terminated:
      statusStr = tr("terminated");
      break;
    case process_status::killed:
      statusStr = tr("killed");
      break;
    }
    setCell(2, statusStr);
    setCell(3, QString::number(p.priority));
    setCell(4, QString::fromStdString(p.uid));
    setCell(5, QString::number(p.step_count));
    setCell(6, QString::number(p.elapsed_time, 'f', 3) + tr("s"));

    // Memory in human-readable format
    QString memStr;
    if (p.current_memory >= 1024 * 1024)
      memStr = QString::number(p.current_memory / (1024.0 * 1024.0), 'f', 1) + tr(" MB");
    else if (p.current_memory >= 1024)
      memStr = QString::number(p.current_memory / 1024.0, 'f', 1) + tr(" KB");
    else
      memStr = QString::number(p.current_memory) + tr(" B");
    setCell(7, memStr);
  }
}

void StateDashboardWidget::onProcessTableSelectionChanged() {
  bool hasSelection = !m_processTable->selectedItems().isEmpty();
  m_pauseBtn->setEnabled(hasSelection);
  m_resumeBtn->setEnabled(hasSelection);
  m_killBtn->setEnabled(hasSelection);
}

int getSelectedPid(QTableWidget *table) {
  auto items = table->selectedItems();
  if (items.isEmpty())
    return -1;
  return table->item(items[0]->row(), 0)->text().toInt();
}

void StateDashboardWidget::onPauseProcessClicked() {
  if (!m_scheduler)
    return;
  int pid = getSelectedPid(m_processTable);
  if (pid >= 0) {
    m_scheduler->pause(pid);
    refreshProcessList();
  }
}

void StateDashboardWidget::onResumeProcessClicked() {
  if (!m_scheduler)
    return;
  int pid = getSelectedPid(m_processTable);
  if (pid >= 0) {
    m_scheduler->resume(pid);
    refreshProcessList();
  }
}

void StateDashboardWidget::onKillProcessClicked() {
  if (!m_scheduler)
    return;
  int pid = getSelectedPid(m_processTable);
  if (pid >= 0) {
    m_scheduler->kill(pid);
    refreshProcessList();
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tab 3 — Cluster & Networking
// ═══════════════════════════════════════════════════════════════════════════

void StateDashboardWidget::buildClusterTab(QTabWidget *tabs) {
  auto *page = new QWidget;
  auto *layout = new QVBoxLayout(page);
  layout->setContentsMargins(2, 2, 2, 2);

  // Node identity
  auto *idGroup = new QGroupBox(tr("Local Node"));
  auto *idLayout = new QVBoxLayout(idGroup);
  m_nodeIdLabel = new QLabel(tr("Node ID: (not connected)"));
  m_clusterIdLabel = new QLabel(tr("Cluster ID: (none)"));
  m_leaderLabel = new QLabel(tr("Leader: (unknown)"));
  idLayout->addWidget(m_nodeIdLabel);
  idLayout->addWidget(m_clusterIdLabel);
  idLayout->addWidget(m_leaderLabel);
  layout->addWidget(idGroup);

  // Peer list
  auto *peerGroup = new QGroupBox(tr("Peers"));
  auto *peerLayout = new QVBoxLayout(peerGroup);

  m_peerTable = new QTableWidget;
  m_peerTable->setColumnCount(4);
  m_peerTable->setHorizontalHeaderLabels(
      {tr("Node ID"), tr("Endpoint"), tr("State"), tr("Last Heartbeat")});
  m_peerTable->horizontalHeader()->setStretchLastSection(true);
  m_peerTable->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_peerTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  peerLayout->addWidget(m_peerTable);

  // Connect to peer
  auto *connectBar = new QHBoxLayout;
  m_peerEndpointInput = new QLineEdit;
  m_peerEndpointInput->setPlaceholderText(tr("host:port"));
  m_connectPeerBtn = new QPushButton(tr("Connect"));
  connect(m_connectPeerBtn, &QPushButton::clicked, this,
          &StateDashboardWidget::onConnectPeerClicked);
  connectBar->addWidget(m_peerEndpointInput);
  connectBar->addWidget(m_connectPeerBtn);
  peerLayout->addLayout(connectBar);
  layout->addWidget(peerGroup);

  // Stats
  auto *statsGroup = new QGroupBox(tr("Cluster Statistics"));
  auto *statsLayout = new QVBoxLayout(statsGroup);
  m_busStatsLabel = new QLabel(tr("Message Bus: (no shard)"));
  m_shardStatsLabel = new QLabel(tr("Shard: (no shard)"));
  m_telemetryLabel = new QLabel(tr("Telemetry: (none)"));
  m_busStatsLabel->setWordWrap(true);
  m_shardStatsLabel->setWordWrap(true);
  m_telemetryLabel->setWordWrap(true);
  statsLayout->addWidget(m_busStatsLabel);
  statsLayout->addWidget(m_shardStatsLabel);
  statsLayout->addWidget(m_telemetryLabel);
  layout->addWidget(statsGroup);

  layout->addStretch();

  // Timer for auto-refreshing cluster info
  m_clusterRefreshTimer = new QTimer(this);
  m_clusterRefreshTimer->setInterval(2000);
  connect(m_clusterRefreshTimer, &QTimer::timeout, this, &StateDashboardWidget::refreshClusterInfo);
  m_clusterRefreshTimer->start();

  tabs->addTab(page, tr("Cluster"));
}

void StateDashboardWidget::onConnectPeerClicked() {
  QString endpoint = m_peerEndpointInput->text().trimmed();
  if (endpoint.isEmpty())
    return;

  if (!m_membership) {
    QMessageBox::information(this, tr("Not Connected"),
                             tr("No cluster membership manager configured."));
    return;
  }

  // Register the peer with the endpoint; the membership manager will
  // initiate heartbeats and the peer will be integrated into the cluster.
  std::string nodeId = "peer-" + endpoint.toStdString();
  m_membership->register_peer(nodeId, m_membership->cluster_id(), endpoint.toStdString());
  m_peerEndpointInput->clear();
  refreshClusterInfo();
}

void StateDashboardWidget::refreshClusterInfo() {
  // Identity
  if (m_membership) {
    m_nodeIdLabel->setText(
        tr("Node ID: %1").arg(QString::fromStdString(m_membership->local_node_id())));
    m_clusterIdLabel->setText(
        tr("Cluster ID: %1").arg(QString::fromStdString(m_membership->cluster_id())));
  } else if (m_shard) {
    m_nodeIdLabel->setText(tr("Node ID: %1").arg(QString::fromStdString(m_shard->local_node_id())));
    m_clusterIdLabel->setText(
        tr("Cluster ID: %1").arg(QString::fromStdString(m_shard->cluster_id())));
  }

  // Peer table
  if (m_membership) {
    auto peers = m_membership->peer_snapshot();
    m_peerTable->setRowCount(static_cast<int>(peers.size()));
    for (int i = 0; i < static_cast<int>(peers.size()); ++i) {
      auto &p = peers[static_cast<size_t>(i)];
      auto setCell = [this, i](int col, const QString &text) {
        auto *item = new QTableWidgetItem(text);
        item->setFlags(item->flags() & ~Qt::ItemIsEditable);
        m_peerTable->setItem(i, col, item);
      };

      setCell(0, QString::fromStdString(p.node_id));
      setCell(1, QString::fromStdString(p.endpoint));

      QString stateStr;
      switch (p.state) {
      case state_cluster_membership::peer_state::alive:
        stateStr = tr("alive");
        break;
      case state_cluster_membership::peer_state::suspect:
        stateStr = tr("suspect");
        break;
      case state_cluster_membership::peer_state::dead:
        stateStr = tr("dead");
        break;
      }
      setCell(2, stateStr);
      setCell(3, QString::number(p.last_heartbeat_ns));
    }
  }

  // Message bus stats
  if (m_shard) {
    auto &bus = m_shard->message_bus();
    m_busStatsLabel->setText(tr("Message Bus — admitted: %1, dispatched: %2, duplicates: %3, "
                                "dropped: %4, subscribers: %5, dedup size: %6")
                                 .arg(bus.total_admitted())
                                 .arg(bus.total_dispatched())
                                 .arg(bus.total_duplicates())
                                 .arg(bus.total_dropped())
                                 .arg(bus.subscriber_count())
                                 .arg(bus.dedup_size()));

    m_shardStatsLabel->setText(tr("Shard — attached: %1, remote applied: %2, rejected: %3, "
                                  "conflicts: %4, duplicates: %5")
                                   .arg(m_shard->is_attached() ? tr("yes") : tr("no"))
                                   .arg(m_shard->total_remote_applied())
                                   .arg(m_shard->total_remote_rejected())
                                   .arg(m_shard->total_conflicts_detected())
                                   .arg(m_shard->total_remote_duplicates()));
  }

  // Telemetry
  if (m_telemetryAgg) {
    auto summary = m_telemetryAgg->summarize();
    m_telemetryLabel->setText(tr("Telemetry — nodes: %1, stale: %2, mutations published: %3, "
                                 "applied: %4, messages admitted: %5")
                                  .arg(summary.node_count)
                                  .arg(summary.stale_count)
                                  .arg(summary.total_mutations_published)
                                  .arg(summary.total_mutations_applied)
                                  .arg(summary.total_messages_admitted));
  }
}
