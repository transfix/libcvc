/*
  Copyright 2026 The University of Texas at Austin

  This file is part of libcvc.

  libcvc is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License version 2.1 as published by the Free Software Foundation.
*/

#include <cvc/state_message.h>

#include <utility>

namespace CVC_NAMESPACE {

std::string state_message::effective_content_type() const noexcept {
  if (!content_type.empty())
    return content_type;
  if (!bytes.empty())
    return MIME_OCTET;
  return MIME_TEXT;
}

state_message state_message::make_text(std::string path, std::string text,
                                       std::string mime) {
  state_message m;
  m.path = std::move(path);
  m.content_type = std::move(mime);
  m.string_value = std::move(text);
  return m;
}

state_message state_message::make_bytes(std::string path,
                                        std::vector<unsigned char> bytes) {
  state_message m;
  m.path = std::move(path);
  m.content_type = MIME_OCTET;
  m.bytes = std::move(bytes);
  return m;
}

state_message state_message::make_typed(std::string path,
                                        std::string content_type,
                                        std::vector<unsigned char> bytes,
                                        std::string string_value) {
  state_message m;
  m.path = std::move(path);
  m.content_type = std::move(content_type);
  m.bytes = std::move(bytes);
  m.string_value = std::move(string_value);
  return m;
}

} // namespace CVC_NAMESPACE
