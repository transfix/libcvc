#include <QApplication>
#include <QSet>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <cvc/core/app.h>
#include <cvc/core/state.h>
#include <cvc/core/state_cluster_membership.h>
#include <cvc/core/state_cluster_shard.h>
#include <cvc/core/state_exec/builtins.h>
#include <cvc/core/state_exec/scheduler.h>
#include <cvc/core/state_message_bus.h>
#include <cvc/core/state_telemetry_aggregator.h>
#include <gtest/gtest.h>
#include <volrover3/AppState.h>
#include <volrover3/StateDashboardWidget.h>
#include <volrover3/volrover3_app.h>

using namespace cvc;
using namespace cvc::state_exec;

// ═══════════════════════════════════════════════════════════════════════════
// State Tree Tab Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardTreeTest : public ::testing::Test {
protected:
  void SetUp() override {
    root = &state::instance(volrover3::app())("dashboard_test");
    root->operator()("alpha").value("aaa");
    root->operator()("beta").value("bbb");
    root->operator()("gamma").value("parent");
    root->operator()("gamma")("child1").value("c1");
    root->operator()("gamma")("child2").value("c2");

    widget = new StateDashboardWidget();
    widget->setRootState(root);
  }

  void TearDown() override {
    delete widget;
    root->reset();
  }

  state *root;
  StateDashboardWidget *widget;
};

TEST_F(StateDashboardTreeTest, WidgetCreation) {
  ASSERT_NE(widget, nullptr);
  EXPECT_FALSE(widget->isVisible());
}

TEST_F(StateDashboardTreeTest, RefreshDoesNotCrash) { EXPECT_NO_THROW(widget->refresh()); }

TEST_F(StateDashboardTreeTest, SetRootStatePopulatesTree) {
  // Re-set to force population
  widget->setRootState(root);
  QCoreApplication::processEvents();

  // Verify the state hierarchy is there
  auto children = root->children();
  bool hasAlpha = false, hasBeta = false, hasGamma = false;
  for (auto &c : children) {
    if (c.find("alpha") != std::string::npos)
      hasAlpha = true;
    if (c.find("beta") != std::string::npos)
      hasBeta = true;
    if (c.find("gamma") != std::string::npos)
      hasGamma = true;
  }
  EXPECT_TRUE(hasAlpha);
  EXPECT_TRUE(hasBeta);
  EXPECT_TRUE(hasGamma);
}

TEST_F(StateDashboardTreeTest, StateValueReadBack) {
  EXPECT_EQ(root->operator()("alpha").value(), "aaa");
  EXPECT_EQ(root->operator()("beta").value(), "bbb");
  EXPECT_EQ(root->operator()("gamma")("child1").value(), "c1");
}

TEST_F(StateDashboardTreeTest, SetRootNullSafe) {
  EXPECT_NO_THROW(widget->setRootState(nullptr));
  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardTreeTest, DynamicStateAdd) {
  widget->show();
  QCoreApplication::processEvents();

  root->operator()("dynamic_node").value("dyn");
  QCoreApplication::processEvents();

  EXPECT_TRUE(root->operator()("dynamic_node").initialized());
  EXPECT_EQ(root->operator()("dynamic_node").value(), "dyn");
}

TEST_F(StateDashboardTreeTest, StateReset) {
  auto &child = root->operator()("alpha");
  EXPECT_TRUE(child.initialized());
  child.reset();
  EXPECT_FALSE(child.initialized());
}

TEST_F(StateDashboardTreeTest, NestedStateNavigation) {
  auto &gamma = root->operator()("gamma");
  EXPECT_TRUE(gamma.initialized());
  EXPECT_EQ(gamma("child1").value(), "c1");
  EXPECT_EQ(gamma("child2").value(), "c2");
}

TEST_F(StateDashboardTreeTest, StateMetadata) {
  auto &s = root->operator()("alpha");

  s.comment("test comment");
  EXPECT_EQ(s.comment(), "test comment");

  EXPECT_FALSE(s.readOnly());
  EXPECT_FALSE(s.hidden());

  s.readOnly(true);
  EXPECT_TRUE(s.readOnly());

  s.hidden(true);
  EXPECT_TRUE(s.hidden());

  s.readOnly(false);
  s.hidden(false);
}

TEST_F(StateDashboardTreeTest, StateLastModified) {
  auto &s = root->operator()("alpha");
  auto mod = s.lastMod();
  EXPECT_FALSE(mod.is_not_a_date_time());
}

TEST_F(StateDashboardTreeTest, StateLinkProperties) {
  auto &link = root->operator()("link_node");
  EXPECT_FALSE(link.isLink());

  link.linkTo("dashboard_test.alpha");
  EXPECT_TRUE(link.isLink());
  EXPECT_EQ(link.linkTarget(), "dashboard_test.alpha");

  link.clearLink();
  EXPECT_FALSE(link.isLink());
}

TEST_F(StateDashboardTreeTest, StateExpiryProperties) {
  auto &s = root->operator()("expire_test");
  s.value("will expire");

  EXPECT_FALSE(s.hasExpiry());

  auto future = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::hours(1);
  s.expireAt(future);

  EXPECT_TRUE(s.hasExpiry());
  EXPECT_FALSE(s.isExpired());

  s.clearExpiry();
  EXPECT_FALSE(s.hasExpiry());
}

TEST_F(StateDashboardTreeTest, TypedValueRoundTrips) {
  auto &s = root->operator()("typed");

  s.value(42);
  EXPECT_EQ(s.value(), "42");

  s.value(3.14);
  EXPECT_NE(s.value(), "");
  EXPECT_TRUE(s.value().find("3.14") == 0);

  s.value(true);
  EXPECT_EQ(s.value(), "1");

  s.value(std::string("hello"));
  EXPECT_EQ(s.value(), "hello");
}

TEST_F(StateDashboardTreeTest, RefreshAfterMutation) {
  root->operator()("new_child").value("nc");
  EXPECT_NO_THROW(widget->refresh());

  root->operator()("new_child").reset();
  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardTreeTest, MultipleSetRootCalls) {
  EXPECT_NO_THROW(widget->setRootState(root));
  EXPECT_NO_THROW(widget->setRootState(root));
  EXPECT_NO_THROW(widget->setRootState(nullptr));
  EXPECT_NO_THROW(widget->setRootState(root));
}

// ═══════════════════════════════════════════════════════════════════════════
// Exec Console Tab Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardExecTest : public ::testing::Test {
protected:
  void SetUp() override {
    env = builtins::make_default_environment();
    sched = std::make_unique<scheduler>();
    widget = new StateDashboardWidget();
    widget->setScheduler(sched.get());
  }

  void TearDown() override { delete widget; }

  environment_ptr env;
  std::unique_ptr<scheduler> sched;
  StateDashboardWidget *widget;
};

TEST_F(StateDashboardExecTest, SetSchedulerNullSafe) {
  EXPECT_NO_THROW(widget->setScheduler(nullptr));
  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardExecTest, ProcessListEmpty) {
  auto procs = sched->list_processes();
  EXPECT_TRUE(procs.empty());
  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardExecTest, ProcessSubmitAndList) {
  execute_options opts;
  opts.name = "test_proc";
  int pid = sched->execute(std::string("(+ 1 2)"), opts);
  EXPECT_GT(pid, 0);

  auto procs = sched->list_processes();
  EXPECT_EQ(procs.size(), 1u);
  EXPECT_EQ(procs[0].pid, pid);
  EXPECT_EQ(procs[0].name, "test_proc");
}

TEST_F(StateDashboardExecTest, ProcessPauseResumeKill) {
  execute_options opts;
  opts.name = "infinite";
  int pid = sched->execute(std::string("(begin (while t nil))"), opts);

  // Run one step to make it running
  sched->step();

  auto info = sched->get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::ready);

  // Pause
  EXPECT_TRUE(sched->pause(pid));
  info = sched->get_process_info(pid);
  EXPECT_EQ(info->status, process_status::paused);

  // Resume
  EXPECT_TRUE(sched->resume(pid));
  info = sched->get_process_info(pid);
  EXPECT_EQ(info->status, process_status::ready);

  // Kill
  EXPECT_TRUE(sched->kill(pid));
  info = sched->get_process_info(pid);
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(StateDashboardExecTest, ProcessRunToCompletion) {
  execute_options opts;
  opts.name = "simple_add";
  int pid = sched->execute(std::string("(+ 10 20)"), opts);
  auto results = sched->run();

  EXPECT_TRUE(results.count(pid));
  auto &result = results[pid];
  EXPECT_EQ(std::get<int64_t>(result.v), 30);
}

TEST_F(StateDashboardExecTest, MultipleProcesses) {
  execute_options opts_a;
  opts_a.name = "proc_a";
  execute_options opts_b;
  opts_b.name = "proc_b";
  execute_options opts_c;
  opts_c.name = "proc_c";
  sched->execute(std::string("(+ 1 1)"), opts_a);
  sched->execute(std::string("(+ 2 2)"), opts_b);
  sched->execute(std::string("(+ 3 3)"), opts_c);

  auto procs = sched->list_processes();
  EXPECT_EQ(procs.size(), 3u);

  auto results = sched->run();
  EXPECT_EQ(results.size(), 3u);
}

TEST_F(StateDashboardExecTest, SchedulerStats) {
  sched->execute(std::string("(+ 1 1)"));
  sched->execute(std::string("(+ 2 2)"));

  auto stats = sched->get_stats();
  EXPECT_EQ(stats.total_processes, 2u);

  sched->run();
  stats = sched->get_stats();
  EXPECT_EQ(stats.terminated, 2u);
}

TEST_F(StateDashboardExecTest, RefreshWithProcesses) {
  execute_options opts;
  opts.name = "refresh_test";
  sched->execute(std::string("(+ 1 2)"), opts);
  EXPECT_NO_THROW(widget->refresh());
  sched->run();
  EXPECT_NO_THROW(widget->refresh());
}

// ═══════════════════════════════════════════════════════════════════════════
// Cluster Tab Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardClusterTest : public ::testing::Test {
protected:
  void SetUp() override {
    root = &state::instance(volrover3::app())("cluster_dashboard_test");
    root->value("cluster_root");

    shard = std::make_unique<state_cluster_shard>(volrover3::app(), "test_cluster", "node_local");
    membership = std::make_unique<state_cluster_membership>("test_cluster", "node_local");
    telemetry = std::make_unique<state_telemetry_aggregator>("test_cluster");

    widget = new StateDashboardWidget();
    widget->setShard(shard.get());
    widget->setMembership(membership.get());
    widget->setTelemetryAggregator(telemetry.get());
  }

  void TearDown() override {
    delete widget;
    root->reset();
  }

  state *root;
  std::unique_ptr<state_cluster_shard> shard;
  std::unique_ptr<state_cluster_membership> membership;
  std::unique_ptr<state_telemetry_aggregator> telemetry;
  StateDashboardWidget *widget;
};

TEST_F(StateDashboardClusterTest, WidgetCreation) { ASSERT_NE(widget, nullptr); }

TEST_F(StateDashboardClusterTest, SetComponentsNullSafe) {
  auto *w = new StateDashboardWidget();
  EXPECT_NO_THROW(w->setShard(nullptr));
  EXPECT_NO_THROW(w->setMembership(nullptr));
  EXPECT_NO_THROW(w->setTelemetryAggregator(nullptr));
  EXPECT_NO_THROW(w->setCoordinator(nullptr));
  EXPECT_NO_THROW(w->refresh());
  delete w;
}

TEST_F(StateDashboardClusterTest, ClusterIdentity) {
  EXPECT_EQ(shard->cluster_id(), "test_cluster");
  EXPECT_EQ(shard->local_node_id(), "node_local");
  EXPECT_EQ(membership->cluster_id(), "test_cluster");
  EXPECT_EQ(membership->local_node_id(), "node_local");
}

TEST_F(StateDashboardClusterTest, PeerRegistration) {
  membership->register_peer("peer_1", "test_cluster", "host1:9001");
  membership->register_peer("peer_2", "test_cluster", "host2:9002");

  auto peers = membership->peer_snapshot();
  EXPECT_EQ(peers.size(), 2u);
}

TEST_F(StateDashboardClusterTest, PeerSnapshot) {
  membership->register_peer("peer_a", "test_cluster", "10.0.0.1:5000");

  auto peers = membership->peer_snapshot();
  ASSERT_EQ(peers.size(), 1u);
  EXPECT_EQ(peers[0].node_id, "peer_a");
  EXPECT_EQ(peers[0].endpoint, "10.0.0.1:5000");
}

TEST_F(StateDashboardClusterTest, MessageBusStats) {
  auto &bus = shard->message_bus();
  EXPECT_EQ(bus.total_admitted(), 0u);
  EXPECT_EQ(bus.total_dispatched(), 0u);
  EXPECT_EQ(bus.total_dropped(), 0u);
}

TEST_F(StateDashboardClusterTest, ShardCounters) {
  EXPECT_EQ(shard->total_remote_applied(), 0u);
  EXPECT_EQ(shard->total_remote_rejected(), 0u);
  EXPECT_EQ(shard->total_conflicts_detected(), 0u);
}

TEST_F(StateDashboardClusterTest, ShardAttachDetach) {
  EXPECT_FALSE(shard->is_attached());
  shard->attach();
  EXPECT_TRUE(shard->is_attached());
  shard->detach();
  EXPECT_FALSE(shard->is_attached());
}

TEST_F(StateDashboardClusterTest, TelemetrySummary) {
  auto summary = telemetry->summarize();
  EXPECT_EQ(summary.node_count, 0u);
}

TEST_F(StateDashboardClusterTest, RefreshWithAllComponents) {
  membership->register_peer("refresh_peer", "test_cluster", "host:1234");
  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardClusterTest, MembershipCounters) {
  EXPECT_EQ(membership->total_heartbeats_sent(), 0u);
  EXPECT_EQ(membership->total_heartbeats_received(), 0u);
  EXPECT_EQ(membership->total_peers_joined(), 0u);
}

// ═══════════════════════════════════════════════════════════════════════════
// Cross-tab Integration Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardIntegrationTest : public ::testing::Test {
protected:
  void SetUp() override {
    root = &state::instance(volrover3::app())("integration_dashboard_test");
    root->operator()("config")("greeting").value("hello");

    env = builtins::make_default_environment();
    sched = std::make_unique<scheduler>();
    shard = std::make_unique<state_cluster_shard>(volrover3::app(), "integ_cluster", "integ_node");
    membership = std::make_unique<state_cluster_membership>("integ_cluster", "integ_node");

    widget = new StateDashboardWidget();
    widget->setRootState(root);
    widget->setScheduler(sched.get());
    widget->setShard(shard.get());
    widget->setMembership(membership.get());
  }

  void TearDown() override {
    delete widget;
    root->reset();
  }

  state *root;
  environment_ptr env;
  std::unique_ptr<scheduler> sched;
  std::unique_ptr<state_cluster_shard> shard;
  std::unique_ptr<state_cluster_membership> membership;
  StateDashboardWidget *widget;
};

TEST_F(StateDashboardIntegrationTest, FullRefreshAllTabs) {
  execute_options opts;
  opts.name = "integ_proc";
  sched->execute(std::string("(+ 1 1)"), opts);
  membership->register_peer("integ_peer", "integ_cluster", "localhost:9999");

  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardIntegrationTest, ShowAndRefresh) {
  widget->show();
  QCoreApplication::processEvents();

  EXPECT_NO_THROW(widget->refresh());
  QCoreApplication::processEvents();

  widget->hide();
}

TEST_F(StateDashboardIntegrationTest, WidgetDeleteOnClose) {
  auto *w = new StateDashboardWidget();
  w->setAttribute(Qt::WA_DeleteOnClose);
  w->setRootState(root);
  w->show();
  QCoreApplication::processEvents();
  w->close();
  QCoreApplication::processEvents();
  // Widget is now deleted due to WA_DeleteOnClose
}

TEST_F(StateDashboardIntegrationTest, StateModWhileProcessRunning) {
  execute_options opts;
  opts.name = "bg_proc";
  sched->execute(std::string("(begin (while t nil))"), opts);
  sched->step();

  root->operator()("config")("greeting").value("modified");
  EXPECT_EQ(root->operator()("config")("greeting").value(), "modified");

  EXPECT_NO_THROW(widget->refresh());
}

TEST_F(StateDashboardIntegrationTest, MultipleWidgets) {
  auto *w2 = new StateDashboardWidget();
  w2->setRootState(root);
  w2->setScheduler(sched.get());

  EXPECT_NO_THROW(widget->refresh());
  EXPECT_NO_THROW(w2->refresh());

  delete w2;
}

// ═══════════════════════════════════════════════════════════════════════════
// State Tree UI Interaction Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardTreeUITest : public ::testing::Test {
protected:
  void SetUp() override {
    root = &state::instance(volrover3::app())("tree_ui_test");
    root->operator()("apple").value("red");
    root->operator()("banana").value("yellow");
    root->operator()("cherry").value("dark");
    root->operator()("apple")("seed").value("small");
    root->operator()("typed_int").value(42);
    root->operator()("typed_dbl").value(3.14);
    root->operator()("typed_bool").value(true);

    widget = new StateDashboardWidget();
    widget->setRootState(root);
    widget->show();
    QCoreApplication::processEvents();

    // Locate internal widgets by type
    tree = widget->findChild<QTreeWidget *>();
    ASSERT_NE(tree, nullptr);

    // Identify tables by column count: property(2), process(8), peer(4)
    for (auto *table : widget->findChildren<QTableWidget *>()) {
      if (table->columnCount() == 2)
        propTable = table;
      else if (table->columnCount() == 8)
        procTable = table;
      else if (table->columnCount() == 4)
        peerTable = table;
    }
    ASSERT_NE(propTable, nullptr);
  }

  void TearDown() override {
    delete widget;
    root->reset();
  }

  // Select a tree item matching the given text (depth-first)
  QTreeWidgetItem *selectItemByText(QTreeWidgetItem *parent, const QString &text) {
    for (int i = 0; i < parent->childCount(); ++i) {
      auto *child = parent->child(i);
      if (child->text(0) == text) {
        auto idx = tree->indexFromItem(child);
        tree->selectionModel()->select(idx, QItemSelectionModel::ClearAndSelect);
        tree->setCurrentItem(child);
        QCoreApplication::processEvents();
        return child;
      }
      auto *found = selectItemByText(child, text);
      if (found)
        return found;
    }
    return nullptr;
  }

  state *root;
  StateDashboardWidget *widget;
  QTreeWidget *tree;
  QTableWidget *propTable;
  QTableWidget *procTable;
  QTableWidget *peerTable;
};

// Helper to find a property row by key name
static int findPropRow(QTableWidget *table, const QString &key) {
  for (int r = 0; r < table->rowCount(); ++r) {
    if (table->item(r, 0) && table->item(r, 0)->text() == key)
      return r;
  }
  return -1;
}

TEST_F(StateDashboardTreeUITest, SelectionPopulatesPropertyTable) {
  ASSERT_GT(tree->topLevelItemCount(), 0);
  auto *item = selectItemByText(tree->topLevelItem(0), "apple");
  ASSERT_NE(item, nullptr);

  EXPECT_GT(propTable->rowCount(), 0);
  int nameRow = findPropRow(propTable, "Name");
  ASSERT_GE(nameRow, 0);
  EXPECT_EQ(propTable->item(nameRow, 1)->text(), "apple");

  int valRow = findPropRow(propTable, "Value");
  ASSERT_GE(valRow, 0);
  EXPECT_EQ(propTable->item(valRow, 1)->text(), "red");
}

TEST_F(StateDashboardTreeUITest, PropertyTableShowsChildren) {
  selectItemByText(tree->topLevelItem(0), "apple");
  int row = findPropRow(propTable, "Children");
  ASSERT_GE(row, 0);
  EXPECT_EQ(propTable->item(row, 1)->text(), "1"); // apple has child "seed"
}

TEST_F(StateDashboardTreeUITest, PropertyTableShowsReadOnly) {
  root->operator()("apple").readOnly(true);
  widget->setRootState(root); // force clean tree rebuild
  QCoreApplication::processEvents();
  selectItemByText(tree->topLevelItem(0), "apple");

  int row = findPropRow(propTable, "Read Only");
  ASSERT_GE(row, 0);
  EXPECT_EQ(propTable->item(row, 1)->text(), "true");

  // Value cell should NOT be editable when readOnly
  int valRow = findPropRow(propTable, "Value");
  ASSERT_GE(valRow, 0);
  EXPECT_FALSE(propTable->item(valRow, 1)->flags().testFlag(Qt::ItemIsEditable));

  root->operator()("apple").readOnly(false);
}

TEST_F(StateDashboardTreeUITest, PropertyTableShowsHidden) {
  root->operator()("banana").hidden(true);
  widget->setRootState(root);
  QCoreApplication::processEvents();
  selectItemByText(tree->topLevelItem(0), "banana");

  int row = findPropRow(propTable, "Hidden");
  ASSERT_GE(row, 0);
  EXPECT_EQ(propTable->item(row, 1)->text(), "true");

  root->operator()("banana").hidden(false);
}

TEST_F(StateDashboardTreeUITest, PropertyTableShowsComment) {
  root->operator()("cherry").comment("a tart fruit");
  widget->setRootState(root);
  QCoreApplication::processEvents();
  selectItemByText(tree->topLevelItem(0), "cherry");

  int row = findPropRow(propTable, "Comment");
  ASSERT_GE(row, 0);
  EXPECT_EQ(propTable->item(row, 1)->text(), "a tart fruit");
}

TEST_F(StateDashboardTreeUITest, PropertyEditValueUpdatesState) {
  selectItemByText(tree->topLevelItem(0), "banana");

  int valRow = findPropRow(propTable, "Value");
  ASSERT_GE(valRow, 0);

  // Simulate editing the value cell
  propTable->item(valRow, 1)->setText("green");
  // cellChanged signal fires automatically on setText
  QCoreApplication::processEvents();

  EXPECT_EQ(root->operator()("banana").value(), "green");
}

TEST_F(StateDashboardTreeUITest, PropertyEditCommentUpdatesState) {
  selectItemByText(tree->topLevelItem(0), "cherry");

  int commentRow = findPropRow(propTable, "Comment");
  ASSERT_GE(commentRow, 0);

  propTable->item(commentRow, 1)->setText("updated comment");
  QCoreApplication::processEvents();

  EXPECT_EQ(root->operator()("cherry").comment(), "updated comment");
}

TEST_F(StateDashboardTreeUITest, ValueEditEmitsStateChanged) {
  selectItemByText(tree->topLevelItem(0), "apple");

  int signalCount = 0;
  QObject::connect(widget, &StateDashboardWidget::stateChanged,
                   [&signalCount]() { ++signalCount; });

  int valRow = findPropRow(propTable, "Value");
  ASSERT_GE(valRow, 0);

  propTable->item(valRow, 1)->setText("green");
  QCoreApplication::processEvents();

  EXPECT_GE(signalCount, 1);
}

TEST_F(StateDashboardTreeUITest, SearchFilterHidesNonMatching) {
  // Find the tree search box by placeholder text
  QLineEdit *searchBox = nullptr;
  for (auto *edit : widget->findChildren<QLineEdit *>()) {
    if (edit->placeholderText().contains("Filter")) {
      searchBox = edit;
      break;
    }
  }
  ASSERT_NE(searchBox, nullptr);

  searchBox->setText("apple");
  QCoreApplication::processEvents();

  auto *rootItem = tree->topLevelItem(0);
  for (int i = 0; i < rootItem->childCount(); ++i) {
    auto *child = rootItem->child(i);
    if (child->text(0) == "apple") {
      EXPECT_FALSE(child->isHidden()) << "apple should be visible";
    } else if (child->text(0) == "banana" || child->text(0) == "cherry") {
      EXPECT_TRUE(child->isHidden()) << child->text(0).toStdString() << " should be hidden";
    }
  }
}

TEST_F(StateDashboardTreeUITest, SearchFilterClearRestoresAll) {
  QLineEdit *searchBox = nullptr;
  for (auto *edit : widget->findChildren<QLineEdit *>()) {
    if (edit->placeholderText().contains("Filter")) {
      searchBox = edit;
      break;
    }
  }
  ASSERT_NE(searchBox, nullptr);

  searchBox->setText("apple");
  QCoreApplication::processEvents();

  searchBox->setText("");
  QCoreApplication::processEvents();

  // All items should be visible
  auto *rootItem = tree->topLevelItem(0);
  for (int i = 0; i < rootItem->childCount(); ++i) {
    EXPECT_FALSE(rootItem->child(i)->isHidden()) << rootItem->child(i)->text(0).toStdString();
  }
}

TEST_F(StateDashboardTreeUITest, LinkPropertiesShowInTable) {
  auto &link = root->operator()("linked");
  link.linkTo("tree_ui_test.apple");
  widget->setRootState(root);
  QCoreApplication::processEvents();

  selectItemByText(tree->topLevelItem(0), "linked");

  int isLinkRow = findPropRow(propTable, "Is Link");
  ASSERT_GE(isLinkRow, 0);
  EXPECT_EQ(propTable->item(isLinkRow, 1)->text(), "true");

  int targetRow = findPropRow(propTable, "Link Target");
  ASSERT_GE(targetRow, 0);
  EXPECT_EQ(propTable->item(targetRow, 1)->text(), "tree_ui_test.apple");

  int resRow = findPropRow(propTable, "Link Resolution");
  ASSERT_GE(resRow, 0);
  EXPECT_EQ(propTable->item(resRow, 1)->text(), "resolved");
}

TEST_F(StateDashboardTreeUITest, ExpiryPropertiesShowInTable) {
  auto &s = root->operator()("will_expire");
  s.value("temp");
  auto future = boost::posix_time::microsec_clock::universal_time() + boost::posix_time::hours(1);
  s.expireAt(future);
  widget->setRootState(root);
  QCoreApplication::processEvents();

  selectItemByText(tree->topLevelItem(0), "will_expire");

  int hasExpRow = findPropRow(propTable, "Has Expiry");
  ASSERT_GE(hasExpRow, 0);
  EXPECT_EQ(propTable->item(hasExpRow, 1)->text(), "true");

  int expiredRow = findPropRow(propTable, "Expired");
  ASSERT_GE(expiredRow, 0);
  EXPECT_EQ(propTable->item(expiredRow, 1)->text(), "false");
}

TEST_F(StateDashboardTreeUITest, TypedIntValueInTable) {
  selectItemByText(tree->topLevelItem(0), "typed_int");

  int valRow = findPropRow(propTable, "Value");
  ASSERT_GE(valRow, 0);
  EXPECT_EQ(propTable->item(valRow, 1)->text(), "42");

  int typeRow = findPropRow(propTable, "Value Type");
  ASSERT_GE(typeRow, 0);
  EXPECT_FALSE(propTable->item(typeRow, 1)->text().isEmpty());
}

TEST_F(StateDashboardTreeUITest, SelectionClearEmptiesProperties) {
  selectItemByText(tree->topLevelItem(0), "apple");
  EXPECT_GT(propTable->rowCount(), 0);

  tree->clearSelection();
  QCoreApplication::processEvents();

  EXPECT_EQ(propTable->rowCount(), 0);
}

TEST_F(StateDashboardTreeUITest, LastModifiedShownInProperties) {
  root->operator()("apple").value("updated");
  widget->setRootState(root);
  QCoreApplication::processEvents();
  selectItemByText(tree->topLevelItem(0), "apple");

  int row = findPropRow(propTable, "Last Modified");
  ASSERT_GE(row, 0);
  // Should not be "(unknown)" since we just wrote a value
  EXPECT_NE(propTable->item(row, 1)->text(), "(unknown)");
}

TEST_F(StateDashboardTreeUITest, DeleteButtonDisabledWithNoSelection) {
  auto buttons = widget->findChildren<QPushButton *>();
  QPushButton *deleteBtn = nullptr;
  for (auto *btn : buttons) {
    if (btn->text() == "Delete State") {
      deleteBtn = btn;
      break;
    }
  }
  ASSERT_NE(deleteBtn, nullptr);

  tree->clearSelection();
  QCoreApplication::processEvents();
  EXPECT_FALSE(deleteBtn->isEnabled());
}

TEST_F(StateDashboardTreeUITest, DeleteButtonEnabledWithSelection) {
  auto buttons = widget->findChildren<QPushButton *>();
  QPushButton *deleteBtn = nullptr;
  for (auto *btn : buttons) {
    if (btn->text() == "Delete State") {
      deleteBtn = btn;
      break;
    }
  }
  ASSERT_NE(deleteBtn, nullptr);

  selectItemByText(tree->topLevelItem(0), "banana");
  EXPECT_TRUE(deleteBtn->isEnabled());
}

TEST_F(StateDashboardTreeUITest, NestedChildSelectionShowsProperties) {
  selectItemByText(tree->topLevelItem(0), "seed");

  int nameRow = findPropRow(propTable, "Name");
  ASSERT_GE(nameRow, 0);
  EXPECT_EQ(propTable->item(nameRow, 1)->text(), "seed");

  int valRow = findPropRow(propTable, "Value");
  ASSERT_GE(valRow, 0);
  EXPECT_EQ(propTable->item(valRow, 1)->text(), "small");
}

// ═══════════════════════════════════════════════════════════════════════════
// Exec Console UI Interaction Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardExecUITest : public ::testing::Test {
protected:
  void SetUp() override {
    env = builtins::make_default_environment();
    sched = std::make_unique<scheduler>();

    widget = new StateDashboardWidget();
    widget->setScheduler(sched.get());
    widget->show();
    QCoreApplication::processEvents();

    // Locate internal widgets — identify by readOnly flag
    for (auto *editor : widget->findChildren<QPlainTextEdit *>()) {
      if (editor->isReadOnly())
        outputPanel = editor;
      else
        scriptEditor = editor;
    }
    ASSERT_NE(scriptEditor, nullptr);
    ASSERT_NE(outputPanel, nullptr);

    // processTable has 8 columns
    for (auto *table : widget->findChildren<QTableWidget *>()) {
      if (table->columnCount() == 8) {
        processTable = table;
        break;
      }
    }
    ASSERT_NE(processTable, nullptr);

    // Find buttons by text
    for (auto *btn : widget->findChildren<QPushButton *>()) {
      if (btn->text() == "Run")
        runBtn = btn;
      else if (btn->text() == "Clear Output")
        clearBtn = btn;
      else if (btn->text() == "Pause")
        pauseBtn = btn;
      else if (btn->text() == "Resume")
        resumeBtn = btn;
      else if (btn->text() == "Kill")
        killBtn = btn;
    }
  }

  void TearDown() override { delete widget; }

  environment_ptr env;
  std::unique_ptr<scheduler> sched;
  StateDashboardWidget *widget;
  QPlainTextEdit *scriptEditor = nullptr;
  QPlainTextEdit *outputPanel = nullptr;
  QTableWidget *processTable = nullptr;
  QPushButton *runBtn = nullptr;
  QPushButton *clearBtn = nullptr;
  QPushButton *pauseBtn = nullptr;
  QPushButton *resumeBtn = nullptr;
  QPushButton *killBtn = nullptr;
};

TEST_F(StateDashboardExecUITest, RunScriptShowsOutput) {
  ASSERT_NE(scriptEditor, nullptr);
  ASSERT_NE(runBtn, nullptr);

  scriptEditor->setPlainText("(+ 10 20)");
  runBtn->click();
  QCoreApplication::processEvents();

  QString output = outputPanel->toPlainText();
  EXPECT_TRUE(output.contains("30")) << output.toStdString();
}

TEST_F(StateDashboardExecUITest, RunScriptErrorShowsError) {
  scriptEditor->setPlainText("(undefined_func 1 2)");
  runBtn->click();
  QCoreApplication::processEvents();

  QString output = outputPanel->toPlainText();
  EXPECT_TRUE(output.contains("ERROR")) << output.toStdString();
}

TEST_F(StateDashboardExecUITest, RunScriptShowsEcho) {
  scriptEditor->setPlainText("(* 3 7)");
  runBtn->click();
  QCoreApplication::processEvents();

  QString output = outputPanel->toPlainText();
  // The script itself is echoed with "> " prefix
  EXPECT_TRUE(output.contains("> (* 3 7)")) << output.toStdString();
}

TEST_F(StateDashboardExecUITest, EmptyScriptDoesNothing) {
  scriptEditor->setPlainText("");
  runBtn->click();
  QCoreApplication::processEvents();

  EXPECT_TRUE(outputPanel->toPlainText().isEmpty());
}

TEST_F(StateDashboardExecUITest, ClearOutputClearsPanel) {
  scriptEditor->setPlainText("(+ 1 1)");
  runBtn->click();
  QCoreApplication::processEvents();
  EXPECT_FALSE(outputPanel->toPlainText().isEmpty());

  clearBtn->click();
  QCoreApplication::processEvents();
  EXPECT_TRUE(outputPanel->toPlainText().isEmpty());
}

TEST_F(StateDashboardExecUITest, ProcessTableShowsSubmittedProcess) {
  execute_options opts;
  opts.name = "table_test_proc";
  sched->execute(std::string("(begin (while t nil))"), opts);

  widget->refresh();
  QCoreApplication::processEvents();

  ASSERT_EQ(processTable->rowCount(), 1);
  // Column 0: PID, Column 1: Name
  EXPECT_EQ(processTable->item(0, 1)->text(), "table_test_proc");
}

TEST_F(StateDashboardExecUITest, ProcessTableShowsStatus) {
  execute_options opts;
  opts.name = "status_proc";
  int pid = sched->execute(std::string("(begin (while t nil))"), opts);

  widget->refresh();
  QCoreApplication::processEvents();

  // should be "ready" before any steps
  EXPECT_EQ(processTable->item(0, 2)->text(), "ready");

  sched->pause(pid);
  widget->refresh();
  QCoreApplication::processEvents();

  EXPECT_EQ(processTable->item(0, 2)->text(), "paused");
}

TEST_F(StateDashboardExecUITest, ProcessButtonsDisabledWithoutSelection) {
  EXPECT_FALSE(pauseBtn->isEnabled());
  EXPECT_FALSE(resumeBtn->isEnabled());
  EXPECT_FALSE(killBtn->isEnabled());
}

TEST_F(StateDashboardExecUITest, ProcessButtonsEnabledOnSelection) {
  execute_options opts;
  opts.name = "select_proc";
  sched->execute(std::string("(begin (while t nil))"), opts);
  widget->refresh();
  QCoreApplication::processEvents();

  processTable->selectRow(0);
  QCoreApplication::processEvents();

  EXPECT_TRUE(pauseBtn->isEnabled());
  EXPECT_TRUE(resumeBtn->isEnabled());
  EXPECT_TRUE(killBtn->isEnabled());
}

TEST_F(StateDashboardExecUITest, PauseButtonPausesSelectedProcess) {
  execute_options opts;
  opts.name = "pause_me";
  int pid = sched->execute(std::string("(begin (while t nil))"), opts);
  widget->refresh();
  QCoreApplication::processEvents();

  processTable->selectRow(0);
  QCoreApplication::processEvents();

  pauseBtn->click();
  QCoreApplication::processEvents();

  auto info = sched->get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::paused);
}

TEST_F(StateDashboardExecUITest, KillButtonKillsSelectedProcess) {
  execute_options opts;
  opts.name = "kill_me";
  int pid = sched->execute(std::string("(begin (while t nil))"), opts);
  widget->refresh();
  QCoreApplication::processEvents();

  processTable->selectRow(0);
  QCoreApplication::processEvents();

  killBtn->click();
  QCoreApplication::processEvents();

  auto info = sched->get_process_info(pid);
  ASSERT_TRUE(info.has_value());
  EXPECT_EQ(info->status, process_status::killed);
}

TEST_F(StateDashboardExecUITest, MultipleProcessesInTable) {
  execute_options opts_a, opts_b;
  opts_a.name = "proc_x";
  opts_b.name = "proc_y";
  sched->execute(std::string("(+ 1 1)"), opts_a);
  sched->execute(std::string("(+ 2 2)"), opts_b);

  widget->refresh();
  QCoreApplication::processEvents();

  EXPECT_EQ(processTable->rowCount(), 2);
}

TEST_F(StateDashboardExecUITest, CompletedProcessShowsTerminated) {
  execute_options opts;
  opts.name = "done_proc";
  sched->execute(std::string("(+ 1 1)"), opts);
  sched->run();

  widget->refresh();
  QCoreApplication::processEvents();

  ASSERT_EQ(processTable->rowCount(), 1);
  EXPECT_EQ(processTable->item(0, 2)->text(), "terminated");
}

// ═══════════════════════════════════════════════════════════════════════════
// Cluster Tab UI Interaction Tests
// ═══════════════════════════════════════════════════════════════════════════

class StateDashboardClusterUITest : public ::testing::Test {
protected:
  void SetUp() override {
    root = &state::instance(volrover3::app())("cluster_ui_test");
    root->value("cluster_ui_root");

    shard = std::make_unique<state_cluster_shard>(volrover3::app(), "ui_cluster", "ui_node");
    membership = std::make_unique<state_cluster_membership>("ui_cluster", "ui_node");
    telemetry = std::make_unique<state_telemetry_aggregator>("ui_cluster");

    widget = new StateDashboardWidget();
    widget->setShard(shard.get());
    widget->setMembership(membership.get());
    widget->setTelemetryAggregator(telemetry.get());
    widget->show();
    widget->refresh();
    QCoreApplication::processEvents();

    // Find labels
    for (auto *label : widget->findChildren<QLabel *>()) {
      QString text = label->text();
      if (text.startsWith("Node ID:"))
        nodeIdLabel = label;
      else if (text.startsWith("Cluster ID:"))
        clusterIdLabel = label;
      else if (text.startsWith("Leader:"))
        leaderLabel = label;
      else if (text.startsWith("Message Bus"))
        busLabel = label;
      else if (text.startsWith("Shard"))
        shardLabel = label;
      else if (text.startsWith("Telemetry"))
        telemetryLabel = label;
    }

    // Peer table has 4 columns
    for (auto *table : widget->findChildren<QTableWidget *>()) {
      if (table->columnCount() == 4) {
        peerTableWidget = table;
        break;
      }
    }
    ASSERT_NE(peerTableWidget, nullptr);
  }

  void TearDown() override {
    delete widget;
    root->reset();
  }

  state *root;
  std::unique_ptr<state_cluster_shard> shard;
  std::unique_ptr<state_cluster_membership> membership;
  std::unique_ptr<state_telemetry_aggregator> telemetry;
  StateDashboardWidget *widget;
  QLabel *nodeIdLabel = nullptr;
  QLabel *clusterIdLabel = nullptr;
  QLabel *leaderLabel = nullptr;
  QLabel *busLabel = nullptr;
  QLabel *shardLabel = nullptr;
  QLabel *telemetryLabel = nullptr;
  QTableWidget *peerTableWidget = nullptr;
};

TEST_F(StateDashboardClusterUITest, NodeIdLabelShowsCorrectId) {
  ASSERT_NE(nodeIdLabel, nullptr);
  EXPECT_TRUE(nodeIdLabel->text().contains("ui_node")) << nodeIdLabel->text().toStdString();
}

TEST_F(StateDashboardClusterUITest, ClusterIdLabelShowsCorrectId) {
  ASSERT_NE(clusterIdLabel, nullptr);
  EXPECT_TRUE(clusterIdLabel->text().contains("ui_cluster"))
      << clusterIdLabel->text().toStdString();
}

TEST_F(StateDashboardClusterUITest, BusStatsLabelShowsZeroCounts) {
  ASSERT_NE(busLabel, nullptr);
  EXPECT_TRUE(busLabel->text().contains("admitted: 0")) << busLabel->text().toStdString();
  EXPECT_TRUE(busLabel->text().contains("dispatched: 0")) << busLabel->text().toStdString();
}

TEST_F(StateDashboardClusterUITest, ShardStatsLabelShowsAttachedNo) {
  ASSERT_NE(shardLabel, nullptr);
  EXPECT_TRUE(shardLabel->text().contains("attached: no")) << shardLabel->text().toStdString();
}

TEST_F(StateDashboardClusterUITest, ShardStatsLabelShowsAttachedYes) {
  shard->attach();
  widget->refresh();
  QCoreApplication::processEvents();

  // Re-find label since refresh may update text
  for (auto *label : widget->findChildren<QLabel *>()) {
    if (label->text().startsWith("Shard"))
      shardLabel = label;
  }
  ASSERT_NE(shardLabel, nullptr);
  EXPECT_TRUE(shardLabel->text().contains("attached: yes")) << shardLabel->text().toStdString();
}

TEST_F(StateDashboardClusterUITest, TelemetryLabelShowsNodeCount) {
  ASSERT_NE(telemetryLabel, nullptr);
  EXPECT_TRUE(telemetryLabel->text().contains("nodes: 0")) << telemetryLabel->text().toStdString();
}

TEST_F(StateDashboardClusterUITest, PeerTablePopulatesAfterRegistration) {
  membership->register_peer("peer_ui_1", "ui_cluster", "10.0.0.1:5000");
  membership->register_peer("peer_ui_2", "ui_cluster", "10.0.0.2:5000");
  widget->refresh();
  QCoreApplication::processEvents();

  ASSERT_EQ(peerTableWidget->rowCount(), 2);
  // Verify both endpoints are present (order not guaranteed)
  QSet<QString> endpoints;
  for (int i = 0; i < peerTableWidget->rowCount(); ++i)
    endpoints.insert(peerTableWidget->item(i, 1)->text());
  EXPECT_TRUE(endpoints.contains("10.0.0.1:5000"));
  EXPECT_TRUE(endpoints.contains("10.0.0.2:5000"));
}

TEST_F(StateDashboardClusterUITest, PeerTableShowsPeerState) {
  membership->register_peer("alive_peer", "ui_cluster", "host:1234");
  widget->refresh();
  QCoreApplication::processEvents();

  ASSERT_EQ(peerTableWidget->rowCount(), 1);
  // Newly registered peers start as "alive"
  EXPECT_EQ(peerTableWidget->item(0, 2)->text(), "alive");
}

TEST_F(StateDashboardClusterUITest, ShardOnlyIdentity) {
  // Widget with shard but no membership uses shard for identity
  auto *w = new StateDashboardWidget();
  w->setShard(shard.get());
  w->refresh();
  QCoreApplication::processEvents();

  QLabel *nodeLabel = nullptr;
  for (auto *label : w->findChildren<QLabel *>()) {
    if (label->text().contains("ui_node")) {
      nodeLabel = label;
      break;
    }
  }
  EXPECT_NE(nodeLabel, nullptr);
  delete w;
}

TEST_F(StateDashboardClusterUITest, ConnectPeerViaEndpointInput) {
  QLineEdit *peerInput = nullptr;
  for (auto *edit : widget->findChildren<QLineEdit *>()) {
    if (edit->placeholderText().contains("host:port")) {
      peerInput = edit;
      break;
    }
  }
  ASSERT_NE(peerInput, nullptr);

  QPushButton *connectBtn = nullptr;
  for (auto *btn : widget->findChildren<QPushButton *>()) {
    if (btn->text() == "Connect") {
      connectBtn = btn;
      break;
    }
  }
  ASSERT_NE(connectBtn, nullptr);

  peerInput->setText("newhost:7777");
  connectBtn->click();
  QCoreApplication::processEvents();

  // Peer should now be registered
  auto peers = membership->peer_snapshot();
  EXPECT_EQ(peers.size(), 1u);
  EXPECT_EQ(peers[0].endpoint, "newhost:7777");
}

TEST_F(StateDashboardClusterUITest, ConnectPeerClearsInput) {
  QLineEdit *peerInput = nullptr;
  for (auto *edit : widget->findChildren<QLineEdit *>()) {
    if (edit->placeholderText().contains("host:port")) {
      peerInput = edit;
      break;
    }
  }
  ASSERT_NE(peerInput, nullptr);

  QPushButton *connectBtn = nullptr;
  for (auto *btn : widget->findChildren<QPushButton *>()) {
    if (btn->text() == "Connect") {
      connectBtn = btn;
      break;
    }
  }
  ASSERT_NE(connectBtn, nullptr);

  peerInput->setText("host:8888");
  connectBtn->click();
  QCoreApplication::processEvents();

  EXPECT_TRUE(peerInput->text().isEmpty());
}

TEST_F(StateDashboardClusterUITest, EmptyEndpointConnectIgnored) {
  QLineEdit *peerInput = nullptr;
  for (auto *edit : widget->findChildren<QLineEdit *>()) {
    if (edit->placeholderText().contains("host:port")) {
      peerInput = edit;
      break;
    }
  }
  ASSERT_NE(peerInput, nullptr);

  QPushButton *connectBtn = nullptr;
  for (auto *btn : widget->findChildren<QPushButton *>()) {
    if (btn->text() == "Connect") {
      connectBtn = btn;
      break;
    }
  }
  ASSERT_NE(connectBtn, nullptr);

  peerInput->setText("");
  connectBtn->click();
  QCoreApplication::processEvents();

  auto peers = membership->peer_snapshot();
  EXPECT_TRUE(peers.empty());
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
