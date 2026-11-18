// macOS clipboard backend for cvc::gl::clipboard (NSPasteboard).
//
// Clipboard.cpp has declared these two entry points, and its
// __APPLE__ branch has called them, since the clipboard landed — but the file
// they pointed at was never written, so libcvcGL simply did not link on macOS:
//
//     Undefined symbols for architecture arm64:
//       "_cvcgl_mac_clip_get", referenced from: cvc::gl::clipboard::get()
//       "_cvcgl_mac_clip_set", referenced from: cvc::gl::clipboard::set(...)
//
// Nothing caught it because cvcGL was never built in CI on any platform. Two
// things kept it hidden even locally: the sources are collected with
// file(GLOB "*.cpp"), which cannot see Objective-C++, so adding a .mm here is
// invisible unless CMakeLists.txt names it explicitly (it now does, under
// if(APPLE)).
//
// Unlike X11, macOS has a real clipboard SERVER — the pasteboard outlives the
// process, so there is no ownership dance and no event pump to run, which is
// why this file is short next to the X11 path.
#import <AppKit/AppKit.h>
#include <string>

extern "C" bool cvcgl_mac_clip_set(const char *text) {
  if (!text)
    return false;
  @autoreleasepool {
    NSString *s = [NSString stringWithUTF8String:text];
    if (!s)
      return false; // not valid UTF-8
    NSPasteboard *pb = [NSPasteboard generalPasteboard];
    [pb clearContents]; // required before writing, or setString fails
    return [pb setString:s forType:NSPasteboardTypeString] ? true : false;
  }
}

std::string cvcgl_mac_clip_get() {
  // Copied out while the autorelease pool still holds the NSString's UTF-8
  // bytes. The caller owns the result, so no buffer outlives the call and
  // nothing is shared between threads or apps.
  std::string text;
  @autoreleasepool {
    NSPasteboard *pb = [NSPasteboard generalPasteboard];
    NSString *s = [pb stringForType:NSPasteboardTypeString];
    const char *u = s ? [s UTF8String] : nullptr;
    if (u)
      text = u;
  }
  return text;
}
