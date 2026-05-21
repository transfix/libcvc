// SPDX-License-Identifier: LGPL-2.1
// Tests for cvc::state_message MIME defaults and factory helpers
// (Phase 6 ergonomics).

#include <cvc/state_message.h>
#include <cvc/state_message_bus.h>

#include <gtest/gtest.h>

#include <string>
#include <vector>

using cvc::state_message;
using cvc::state_message_bus;

namespace {

std::vector<unsigned char> bytes_of(const char *s) {
  return std::vector<unsigned char>(s, s + std::char_traits<char>::length(s));
}

}  // namespace

TEST(StateMessage, MimeConstantsAreStandard) {
  EXPECT_STREQ(state_message::MIME_TEXT, "text/plain");
  EXPECT_STREQ(state_message::MIME_OCTET, "application/octet-stream");
  EXPECT_STREQ(state_message::MIME_JSON, "application/json");
}

TEST(StateMessage, EffectiveContentTypeDefaultsToText) {
  state_message m;
  EXPECT_EQ(m.effective_content_type(), state_message::MIME_TEXT);
}

TEST(StateMessage, EffectiveContentTypeDefaultsToOctetForUntypedBytes) {
  state_message m;
  m.bytes = bytes_of("\x01\x02\x03");
  EXPECT_EQ(m.effective_content_type(), state_message::MIME_OCTET);
}

TEST(StateMessage, EffectiveContentTypeHonorsExplicitType) {
  state_message m;
  m.content_type = "application/x-cvc-geometry";
  m.bytes = bytes_of("\x01");
  EXPECT_EQ(m.effective_content_type(), "application/x-cvc-geometry");
}

TEST(StateMessage, MakeTextDefault) {
  auto m = state_message::make_text("ui.event", "hello");
  EXPECT_EQ(m.path, "ui.event");
  EXPECT_EQ(m.content_type, state_message::MIME_TEXT);
  EXPECT_EQ(m.string_value, "hello");
  EXPECT_TRUE(m.bytes.empty());
  EXPECT_EQ(m.effective_content_type(), "text/plain");
}

TEST(StateMessage, MakeTextCustomMime) {
  auto m = state_message::make_text("ui.event", R"({"a":1})",
                                    state_message::MIME_JSON);
  EXPECT_EQ(m.content_type, "application/json");
  EXPECT_EQ(m.string_value, R"({"a":1})");
}

TEST(StateMessage, MakeBytesDefaultsToOctetStream) {
  auto m = state_message::make_bytes("ui.event", bytes_of("\x10\x20"));
  EXPECT_EQ(m.content_type, state_message::MIME_OCTET);
  EXPECT_TRUE(m.string_value.empty());
  EXPECT_EQ(m.bytes.size(), 2u);
  EXPECT_EQ(m.bytes[0], 0x10);
  EXPECT_EQ(m.bytes[1], 0x20);
}

TEST(StateMessage, MakeTypedAllowsArbitraryMimeAndDualPayload) {
  auto m = state_message::make_typed("geom.update",
                                     "application/x-cvc-mesh+protobuf",
                                     bytes_of("MESHBYTES"),
                                     "envelope-v1");
  EXPECT_EQ(m.content_type, "application/x-cvc-mesh+protobuf");
  EXPECT_EQ(m.string_value, "envelope-v1");
  EXPECT_EQ(m.bytes.size(), 9u);
}

TEST(StateMessage, MakeTypedEmptyContentTypeStillFallsBackByEffective) {
  auto m = state_message::make_typed("geom.update", "",
                                     bytes_of("\xff\xfe"));
  EXPECT_TRUE(m.content_type.empty());
  EXPECT_EQ(m.effective_content_type(), state_message::MIME_OCTET);
}

// --- Round-trip through state_message_bus ---------------------------------

TEST(StateMessage, BusDeliversTextWithDefaultMime) {
  state_message_bus bus;
  state_message captured;
  std::size_t hits = 0;
  bus.subscribe("ui", [&](const state_message &m) {
    captured = m;
    ++hits;
  });

  auto m = state_message::make_text("ui.toast", "hello world");
  m.origin_node_id = "node-a";
  m.message_id = "1";
  EXPECT_TRUE(bus.admit(m));
  EXPECT_EQ(hits, 1u);
  EXPECT_EQ(captured.string_value, "hello world");
  EXPECT_EQ(captured.effective_content_type(), "text/plain");
}

TEST(StateMessage, BusDeliversOctetStreamWithDefaultMime) {
  state_message_bus bus;
  state_message captured;
  bus.subscribe("blob", [&](const state_message &m) { captured = m; });

  auto m = state_message::make_bytes(
      "blob.payload",
      std::vector<unsigned char>{0x00, 0x01, 0x02, 0xff});
  m.origin_node_id = "node-b";
  m.message_id = "1";
  EXPECT_TRUE(bus.admit(m));
  EXPECT_EQ(captured.content_type, "application/octet-stream");
  ASSERT_EQ(captured.bytes.size(), 4u);
  EXPECT_EQ(captured.bytes[3], 0xff);
}

TEST(StateMessage, BusPreservesArbitraryTypedObject) {
  state_message_bus bus;
  state_message captured;
  bus.subscribe("geom", [&](const state_message &m) { captured = m; });

  auto m = state_message::make_typed("geom.mesh",
                                     "application/x-cvc-mesh+protobuf",
                                     bytes_of("\x42\x43\x44"),
                                     "v=1");
  m.origin_node_id = "node-c";
  m.message_id = "1";
  EXPECT_TRUE(bus.admit(m));
  EXPECT_EQ(captured.content_type, "application/x-cvc-mesh+protobuf");
  EXPECT_EQ(captured.string_value, "v=1");
  ASSERT_EQ(captured.bytes.size(), 3u);
  EXPECT_EQ(captured.bytes[0], 0x42);
}

TEST(StateMessage, BusDedupHonoredAcrossTypedPayloads) {
  state_message_bus bus;
  std::size_t hits = 0;
  bus.subscribe("", [&](const state_message &) { ++hits; });

  auto a = state_message::make_typed("p", "application/x-foo",
                                     bytes_of("AAA"));
  a.origin_node_id = "n";
  a.message_id = "id-1";
  EXPECT_TRUE(bus.admit(a));

  // Same (origin, message_id) but different payload: dedup wins.
  auto b = state_message::make_typed("p", "application/x-bar",
                                     bytes_of("BBB"));
  b.origin_node_id = "n";
  b.message_id = "id-1";
  EXPECT_FALSE(bus.admit(b));
  EXPECT_EQ(hits, 1u);
}
