#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cvc/state_change_journal.h>
#include <gtest/gtest.h>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

using namespace CVC_NAMESPACE;

namespace {

state_mutation make_value_mutation(const std::string &path, const std::string &value) {
  state_mutation mutation;
  mutation.cluster_id = "cluster-a";
  mutation.tree_id = "tree-main";
  mutation.path = path;
  mutation.op = state_mutation_op::set_value;
  mutation.type_name = "std::string";
  mutation.string_value = value;
  return mutation;
}

bool opt_in_enabled(const char *name) {
  const char *value = std::getenv(name);
  return value != nullptr && std::string(value) == "1";
}

} // namespace

TEST(StateChangeJournalTest, AppendsLocalSequenceAndMutationId) {
  state_change_journal journal("node-a");

  state_mutation stored = journal.append(make_value_mutation("scene.camera.x", "1.25"));

  EXPECT_EQ(stored.sequence, 1u);
  EXPECT_EQ(stored.origin_node_id, "node-a");
  EXPECT_EQ(stored.mutation_id, "node-a:1");
  EXPECT_EQ(stored.path, "scene.camera.x");
  EXPECT_EQ(stored.string_value, "1.25");
  EXPECT_EQ(journal.size(), 1u);
  EXPECT_EQ(journal.last_sequence(), 1u);
}

TEST(StateChangeJournalTest, PreservesExplicitOriginAndMutationId) {
  state_change_journal journal("node-a");
  state_mutation mutation = make_value_mutation("scene.camera.y", "2.5");
  mutation.origin_node_id = "node-b";
  mutation.mutation_id = "external-id";

  state_mutation stored = journal.append(mutation);

  EXPECT_EQ(stored.sequence, 1u);
  EXPECT_EQ(stored.origin_node_id, "node-b");
  EXPECT_EQ(stored.mutation_id, "external-id");
}

TEST(StateChangeJournalTest, SnapshotReturnsIndependentCopy) {
  state_change_journal journal("node-a");
  journal.append(make_value_mutation("a", "1"));
  journal.append(make_value_mutation("b", "2"));

  std::vector<state_mutation> snapshot = journal.snapshot();
  snapshot[0].string_value = "changed";

  std::vector<state_mutation> second_snapshot = journal.snapshot();
  ASSERT_EQ(second_snapshot.size(), 2u);
  EXPECT_EQ(second_snapshot[0].string_value, "1");
  EXPECT_EQ(second_snapshot[1].string_value, "2");
}

TEST(StateChangeJournalTest, ReplaysMutationsAfterSequence) {
  state_change_journal journal("node-a");
  journal.append(make_value_mutation("a", "1"));
  journal.append(make_value_mutation("b", "2"));
  journal.append(make_value_mutation("c", "3"));

  std::vector<state_mutation> replay = journal.replay_after(1);

  ASSERT_EQ(replay.size(), 2u);
  EXPECT_EQ(replay[0].sequence, 2u);
  EXPECT_EQ(replay[0].path, "b");
  EXPECT_EQ(replay[1].sequence, 3u);
  EXPECT_EQ(replay[1].path, "c");
}

TEST(StateChangeJournalTest, ClearDropsMutationsButKeepsMonotonicSequence) {
  state_change_journal journal("node-a");
  journal.append(make_value_mutation("a", "1"));
  journal.append(make_value_mutation("b", "2"));

  journal.clear();
  state_mutation stored = journal.append(make_value_mutation("c", "3"));

  EXPECT_EQ(journal.size(), 1u);
  EXPECT_EQ(stored.sequence, 3u);
  EXPECT_EQ(journal.last_sequence(), 3u);
}

TEST(StateChangeJournalTest, PayloadHelpersRepresentInlineAndBlobData) {
  std::vector<unsigned char> bytes = {1, 2, 3, 4};
  state_payload inline_payload = state_payload::inline_data(bytes);
  state_blob_ref blob_ref;
  blob_ref.digest = "sha256:abc";
  blob_ref.size_bytes = 1024;
  blob_ref.codec = "test-codec-v1";
  state_payload blob_payload = state_payload::blob_ref(blob_ref);

  EXPECT_FALSE(inline_payload.empty());
  EXPECT_EQ(inline_payload.kind, state_payload_kind::inline_bytes);
  EXPECT_EQ(inline_payload.inline_bytes, bytes);

  EXPECT_FALSE(blob_payload.empty());
  EXPECT_EQ(blob_payload.kind, state_payload_kind::blob);
  EXPECT_EQ(blob_payload.blob.digest, "sha256:abc");
  EXPECT_EQ(blob_payload.blob.size_bytes, 1024u);
  EXPECT_EQ(blob_payload.blob.codec, "test-codec-v1");

  EXPECT_TRUE(state_payload::none().empty());
}

TEST(StateChangeJournalTest, OperationNamesAreStable) {
  EXPECT_EQ(to_string(state_mutation_op::set_value), "set_value");
  EXPECT_EQ(to_string(state_mutation_op::set_data), "set_data");
  EXPECT_EQ(to_string(state_mutation_op::delegate_subtree), "delegate_subtree");
  EXPECT_EQ(to_string(state_mutation_op::revoke_delegation), "revoke_delegation");
}

TEST(StateChangeJournalTest, ConcurrentAppendProducesUniqueOrderedSequences) {
  state_change_journal journal("node-a");
  std::vector<state_mutation> stored_mutations;
  std::mutex stored_mutex;
  const int thread_count = 8;
  const int mutations_per_thread = 250;
  std::vector<std::thread> threads;

  for (int thread_index = 0; thread_index < thread_count; ++thread_index) {
    threads.emplace_back([&journal, &stored_mutations, &stored_mutex, thread_index]() {
      for (int mutation_index = 0; mutation_index < mutations_per_thread; ++mutation_index) {
        state_mutation mutation = make_value_mutation("thread." + std::to_string(thread_index),
                                                      std::to_string(mutation_index));
        state_mutation stored = journal.append(mutation);
        std::lock_guard<std::mutex> lock(stored_mutex);
        stored_mutations.push_back(stored);
      }
    });
  }

  for (std::thread &thread : threads) {
    thread.join();
  }

  ASSERT_EQ(stored_mutations.size(), static_cast<std::size_t>(thread_count * mutations_per_thread));
  EXPECT_EQ(journal.size(), stored_mutations.size());
  EXPECT_EQ(journal.last_sequence(), stored_mutations.size());

  std::set<std::uint64_t> sequences;
  std::set<std::string> mutation_ids;
  for (const state_mutation &mutation : stored_mutations) {
    sequences.insert(mutation.sequence);
    mutation_ids.insert(mutation.mutation_id);
  }

  EXPECT_EQ(sequences.size(), stored_mutations.size());
  EXPECT_EQ(mutation_ids.size(), stored_mutations.size());
  EXPECT_EQ(*sequences.begin(), 1u);
  EXPECT_EQ(*sequences.rbegin(), stored_mutations.size());
}

TEST(StateChangeJournalStressTest, OptionalHighVolumeAppendStress) {
  if (!opt_in_enabled("CVC_DISTRIBUTED_STATE_STRESS")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_STRESS=1 to run journal stress tests";
  }

  state_change_journal journal("stress-node");
  const int mutation_count = 100000;
  for (int mutation_index = 0; mutation_index < mutation_count; ++mutation_index) {
    journal.append(make_value_mutation("stress.path", std::to_string(mutation_index)));
  }

  EXPECT_EQ(journal.size(), static_cast<std::size_t>(mutation_count));
  EXPECT_EQ(journal.last_sequence(), static_cast<std::uint64_t>(mutation_count));
  EXPECT_EQ(journal.replay_after(mutation_count - 10).size(), 10u);
}

TEST(StateChangeJournalPerformanceTest, OptionalAppendThroughputSmoke) {
  if (!opt_in_enabled("CVC_DISTRIBUTED_STATE_PERF")) {
    GTEST_SKIP() << "Set CVC_DISTRIBUTED_STATE_PERF=1 to run journal performance smoke tests";
  }

  state_change_journal journal("perf-node");
  const int mutation_count = 50000;
  auto start_time = std::chrono::steady_clock::now();
  for (int mutation_index = 0; mutation_index < mutation_count; ++mutation_index) {
    journal.append(make_value_mutation("perf.path", std::to_string(mutation_index)));
  }
  auto end_time = std::chrono::steady_clock::now();

  double elapsed_seconds =
      std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count();
  double mutations_per_second = static_cast<double>(mutation_count) / elapsed_seconds;
  RecordProperty("mutations_per_second", mutations_per_second);

  EXPECT_EQ(journal.size(), static_cast<std::size_t>(mutation_count));
}