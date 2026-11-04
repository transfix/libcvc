# Ariadne conference: more streams, app and screen sharing, desktop audio

> **Status: PROPOSED (2026-11-03).** Nothing here is built. This is the plan, and the decisions in
> §12 are still open. Parent roadmap: [`ARIADNE_CONFERENCE_ROADMAP.md`](ARIADNE_CONFERENCE_ROADMAP.md)
> (§5l points here).

## 0. The ask

The owner's request, in order of value:

1. **More than one camera and one mic.** A participant can send other audio and video too: a second
   camera, a second mic, another source.
2. **App share.** A participant switches on a mode that streams the contents of the GL frame, either
   the whole window or selected Ariadne windows, so others see it beside the participant's camera.
3. **Screen share.** Better still: capture the desktop, or chosen windows of other applications, and
   send that as a second stream.
4. **Desktop audio.** Best of all: capture the desktop's audio, both channels, and send it in sync
   with the desktop stream.

Together these make the conference a complete video conferencing system inside the deliverable.

## 1. Where the media plane is today

Checked against master a17ca3f0 (libcvc 3.7.9).

| Part | Today | Where |
|---|---|---|
| Wire frame | `cvc::media_frame`: origin pid, `stream_id`, codec, w/h, seq, pts, rate/channels, one whole frame of payload. The same struct on WS and gRPC. | `inc/cvc/media/media_frame.h` |
| Stream names | `stream_id` is a literal **`"camera"` or `"mic"`**. | same, and every consumer below |
| Publisher | The client forwards exactly two streams, `cam_token` (latest frame) and `mic_token` (every chunk), and re-stamps each frame with one host clock reading per pump. | `src/arigl/main.cpp` (`pubs_`, `forward_frame`) |
| Receiver | `peers_` is keyed by pid alone: one video proxy and one audio queue per peer. Video waits for that peer's mic clock (A/V sync). | `src/arigl/main.cpp` (`drain_inbox`, `sync_and_show`) |
| Mixer | One output for all peers: **mono**, 48 kHz, 1024-frame chunks. Opus packets are re-chunked to fit. | `src/arigl/main.cpp` (`mix_audio`) |
| Video codec | **MJPEG only.** Every frame stands alone, so the relay can drop any frame and re-encode smaller rungs. | `inc/cvc/media/media_rungs.h` |
| Relay policy | The AOI tiers and the rungs match the literal `"camera"` + `mjpeg`. | `room_aoi.cpp:278`, `media_rungs.cpp:39` |
| Camera off | A flag (`participants.<pid>.video.off`), not frames. | `room_net.h` |
| Transport | In production one WebSocket (TCP) carries control, poses and media. The unreliable lanes are off. | [`CONFERENCE_UNRELIABLE_TRANSPORT.md`](CONFERENCE_UNRELIABLE_TRANSPORT.md) |
| gRPC room verbs | `room-publish` / `room-subscribe` already take a KIND and advertise `participants.<pid>.streams.<kind>`. The WS client does not use them. | `inc/cvc/ariadne/conference/room_media_intrinsics.h` |
| Native capture | SDL3 cameras and recording devices; any number can be opened. | `inc/cvc/gl/capture/` |
| Browser capture | SDL3's emscripten backends: `getUserMedia` for one default camera and one default mic (**verify** whether SDL 3.4 lists more than one device). | SDL `SDL_emscripten*` |
| FFmpeg | cvcpkg `ffmpeg-lgpl` 7.1.2: H.264 **decode**, VP8/VP9 encode and decode (libvpx), AV1 decode (dav1d). **No H.264 encoder** (x264 is GPL). | cvcpkg `recipes/ffmpeg-lgpl` |

## 2. What has to change

Six problems, each solved in a phase below.

1. **Stream identity.** Two fixed names become named streams per participant, each with a
   descriptor in the room tree. Relay policy keys on the stream's class, not the literal
   `"camera"`. (SH0, §3)
2. **MJPEG cannot carry a desktop.** These are estimates, to be measured in SH1. A 1080p desktop
   frame in MJPEG is roughly 150–300 KB, so 5 fps is 6–12 Mbit/s for every viewer. H.264 or VP9
   at 15 fps is roughly 0.5–2.5 Mbit/s, and much less while the screen is still. An inter-frame
   codec brings keyframes, so the relay can no longer drop any frame it likes. It must drop to the
   next keyframe and ask for one. Re-encoding rungs on the relay is no longer cheap, so rungs
   become temporal layers plus a publisher-made thumbnail. (SH1, §4)
3. **One TCP connection.** A 200 KB keyframe written ahead of a pose snapshot delays it by the
   keyframe's send time. The send queue needs priorities and big frames need chunking. (SH2, §5)
4. **Audio.** The mixer must become stereo. Desktop audio wants Opus in music mode, with no voice
   gating. The capture must leave out the conference's own playback, or every other participant
   hears themselves. (SH5, §8)
5. **Sync.** Today video follows *the peer's mic*. A screen must follow *its own* audio, so streams
   form sync groups, and shares are stamped when they are captured, not when they are pumped. (§9)
6. **Privacy and permission.** A screen or app share must never show what the sharer did not mean
   to show: DMs, the room token, the share controls themselves. OS permission prompts must be
   handled on every platform. (§6, §7)

## 3. SH0: the multi-stream model

**Names.** A stream's name is `<source>[.<n>]`: `camera`, `camera.2`, `mic`, `mic.2`, `app`,
`screen`, `screen.audio`, `screen.thumb`. The name *is* the `media_frame.stream_id`. Names use
`[a-z0-9._-]`, at most 32 characters. `camera` and `mic` keep their meaning as the defaults, so a
single-camera client changes nothing on the wire.

**Descriptors.** The publisher writes one subtree per stream:

```
conf.rooms.<room>.participants.<pid>.streams.<name>.
    kind     video | audio
    source   camera | mic | app | screen | window | desktop_audio | media
    label    "Logitech C920", "Display 1", "Terrain lab"   (shown to viewers)
    codec    mjpeg | h264 | vp9 | opus | f32
    w h fps  (video)      rate ch  (audio)
    sync     a group name: "av" for camera + mic, "screen" for a screen and its audio
    role     tile | stage   (a share asks for the stage)
    off      "1" while paused (the per-stream form of today's video.off)
    ready    relay-written, as today
```

- **Write gate** (`room_write_gate`): admit the publisher's own `streams.<name>.*` leaves above, with
  a valid name and a capped value size. Keep `ready` relay-only.
- **Relay admission:** the relay forwards a media frame only if its `stream_id` is declared under the
  origin's descriptors. This stops a client minting stream ids that open proxies on every receiver.
- **Room limits** (arisrv flags):
  - `--max-streams` per participant (proposed: 4 video and 4 audio);
  - `--max-shares` concurrent app or screen shares per room;
  - a per-stream kbps cap.

**Stream classes** drive the relay's policy:

| Class | Names | Policy |
|---|---|---|
| camera | `camera*` | today's: AOI distance tiers, MJPEG rungs |
| voice | `mic*` | today's: forwarded as sent |
| share video | `app*`, `screen*`, `window*` | no distance tier (a presentation is for the room); subscription-driven; keyframe-aware (§4) |
| share audio | `*.audio` | follows its video's subscription; never spatial |
| thumbnail | `*.thumb` | MJPEG, 1 fps, everyone; what the roster and grid show |

**Subscriptions.** A viewer writes the streams it wants at `participants.<me>.want`. By default it
gets every camera and mic (today's behaviour), and every share at `thumb`. It gets a share at full
when it puts that share on its stage. The relay reads `want` once per tick into a per-viewer set,
because the media gate runs on reader threads and must not walk the tree.

**Client.**
- `pubs_` comes from the document's `conf.self.streams.<name>.token` entries, not two fixed keys.
- `peers_` becomes pid → (name → per-stream state). The proxy token is already published at
  `media_token_key(pid, kind)`, so it only needs the name passed in place of `"camera"`.
- The mixer goes **stereo 48 kHz**. Mono mics are placed in the centre (later they can be panned
  from the avatar's position). Each stream gets its own local volume and mute in `room_users.ari`.

**DSL.**
- `room-publish` takes `{name kind source token sync role label}`.
- `room-unpublish ROOM NAME`.
- `(room-streams ROOM PID)` lists a participant's descriptors, for a `repeat`.
- A second camera is a second `stream-open` + `stream-capture-camera` with another device index.
  On native nothing else is needed.

**Old clients:** not provided for (the owner's rule for the conference, 2026-10-25).

**SH0b, the browser capture seam.** One JS library (working name `cvc_media_web.js`), loaded by
the page, that the wasm calls through `EM_JS`. Every browser phase grows this one file:
- `enumerateDevices`, then `getUserMedia({deviceId})` per stream: the second camera or mic in the
  browser.
- An AudioWorklet that hands f32 frames to the wasm.
- Later, `getDisplayMedia`, `canvas.captureStream` and WebCodecs (SH1, SH3–SH5).

**Tests:**
- `room_relay_integration_test`: two cameras and two mics per client, WS and gRPC mixed.
- `arisrv_aoi_bench --streams N`: the relay's cost per added stream.
- Write-gate cases: an undeclared stream id is dropped, another pid's descriptor is refused, an
  over-long name is refused.

## 4. SH1: inter-frame video, needed for screens at real resolution

**Codec interface.** `cvc::video_encoder` / `cvc::video_decoder` (`inc/cvc/media/media_video.h`),
with two backends:

- **Native, over `ffmpeg-lgpl`:**
  - Decode H.264 (FFmpeg's own decoder) and VP8/VP9 (libvpx).
  - Encode H.264 only through **hardware or OS encoders**: `h264_mf` (Windows Media Foundation),
    `h264_videotoolbox`, `h264_vaapi`, and NVENC / AMF / QSV where present. This is cvcpkg work:
    make the recipe enable or autodetect them, and say which builds carry them.
  - With no hardware encoder, the software fallback is **VP9 (libvpx)**, already in the bundle.
    OpenH264 is the alternative, but Cisco's patent cover applies only to Cisco's own binaries
    (§13).
- **Browser:**
  - WebCodecs `VideoEncoder` / `VideoDecoder` through the SH0b seam. Hardware H.264 is available in
    Chromium, Safari and Firefox desktop (**verify** each browser's minimum version).
  - A decoded `VideoFrame` goes straight into the stream's GL texture with `texImage2D`, with no
    copy through wasm memory. The fallback is `VideoFrame.copyTo` RGBA, then inject.

**Wire.**
- `media_frame.codec` gains `h264`, `vp9` and `vp8`.
- A new `flags` field carries *keyframe* and the *temporal layer id*. It is a MediaFrame proto
  field: both transports carry it, and the relay reads it without parsing the bitstream.
- H.264 travels as Annex-B with SPS/PPS repeated in every keyframe, so a joiner needs no side
  channel.

**Keyframe requests.** A receiver that joins, or that sees a seq gap, asks the publisher for a
keyframe: a room message `media.keyframe <stream>` routed to the origin. The relay coalesces these
to at most one per stream per second, and can ask on a viewer's behalf.

**The relay with inter-frame streams.**
- **Drops are keyframe-aware.** When a frame of such a stream is dropped for a viewer (stall drop,
  backlog, gate), that viewer gets nothing more of the stream until the next keyframe, and the
  relay asks for one.
- **Rungs:**
  - `full`;
  - `half-rate`: drop the temporal layer 1 frames. The encoder runs L1T2, which is WebCodecs'
    `scalabilityMode` and also the native encoders' setting where they have one;
  - `thumb`: the publisher's own `<name>.thumb` MJPEG stream at 1 fps. It is cheap, and the relay can
    shape it as it shapes cameras today;
  - `off`.
- **The relay never transcodes.** It stays cheap, and it stays compatible with end-to-end
  encryption (SFrame, parent §6g).

**Capabilities.** Each client publishes `participants.<pid>.caps.decode`, for example
`mjpeg,opus,h264,vp9`. The publisher picks a codec that every current subscriber can decode. A
viewer that can decode neither gets only the thumbnail, with a note saying why.

**Cameras stay MJPEG** in this plan. Their relay rungs work, and frames are small at camera sizes.
Moving cameras to H.264 later is a bandwidth win, but it is not part of this work.

## 5. SH2: transport for big frames

- **A priority send queue** in the WS transport, and in the gRPC transport, which also queues every
  write. Priority order:
  1. control and poses;
  2. audio;
  3. cameras;
  4. shares.
- **Chunking.** A share frame larger than about 16 KB goes out in pieces that the receiving
  transport reassembles. Then a keyframe never sits ahead of a pose for longer than one piece takes
  to send.
- **The publisher's rate control.** The relay's per-link counters, already published for the
  camera rungs, set the share encoder's target bitrate, which protects the sharer's uplink.
  Temporal-layer rungs handle each viewer's downlink.
- **Measure before adding a socket.** If the queue is not enough under netem on catx-03 (the lane
  bench, parent §6 and `CONFERENCE_UNRELIABLE_TRANSPORT.md`), shares get a second, media-only
  WebSocket with the same room token and device proof.
- **Later, WebRTC (parent §6d).** Shares are what RTP congestion control is for, and SFrame stays
  possible because the relay never transcodes.

## 6. SH3: app share (the Ariadne GL frame)

**Modes:**
- **whole**: the host window's final frame, scene and ImGui together.
- **windows**: selected Ariadne windows, chosen by window name, including scene views.

**Native capture:**
- After the frame is drawn, `glReadPixels` into a ring of three PBOs. Map the one from two frames
  ago, so the GPU never stalls.
- Crop, then hand the pixels to the encoder thread.
- Rate-limit to the share's fps (15 by default), scale down to at most 1920 wide, and work in
  framebuffer pixels on HiDPI screens.

**Browser capture:**
- `canvas.captureStream(fps)` → `MediaStreamTrackProcessor` (Chromium) → `VideoFrame`.
- Crop with `new VideoFrame(frame, {visibleRect})`, then WebCodecs.
- Where there is no track processor, `new VideoFrame(canvas)` once per frame works as a fallback.

**Selected windows, in two steps:**
- **SH3a, crop.** Share the union rectangle of the selected windows. An unselected window
  overlapping them will show, which the UI must say plainly. Two helpers:
  - a "keep shared windows on top" lock;
  - a `share: never` window property that blacks a window out inside the crop.
- **SH3b, compose.** Each share frame, draw only the selected windows' ImGui draw lists
  (`ImDrawData` command lists filtered by window) into an offscreen FBO.
  - Hidden or covered windows share correctly, and private windows can never appear.
  - Scene views that already render to a texture (the viewport insets) are drawn from it.
  - Cost: one extra ImGui draw of the selected windows per share frame.

**Privacy defaults.**
- `share: never` is the default for chat, DMs, the room-token and invite dialogs, and the share
  controls.
- While sharing, a local "you are sharing" frame and indicator is always visible to the sharer. It
  is itself excluded from the share.

**SH3a can ship before SH1.** On MJPEG at 2–5 fps it is a working demo, and it switches to the SH1
codec when that lands.

**Later: live share.** An alternative to pixels is to replicate an Ariadne document's state to the
viewers. That is crisp, tiny and interactive, but it is a different feature. Pixels come first
because they work for any app. It is listed as D7.

## 7. SH4: desktop and other applications' windows

**The common interface.**
- `cvc::gl::capture::screen_source`, a `frame_source`.
- `screen_sources()` enumerates `{id, kind display|window, title, app, thumbnail}`. It is native
  only; the browser uses the browser's own picker.
- v1 hands over CPU BGRA frames. Zero-copy GPU frames (D3D11 texture, IOSurface, dmabuf) into a
  hardware encoder come later.

| Platform | Video | Notes |
|---|---|---|
| **Browser** | `getDisplayMedia({video, audio, systemAudio: 'include', restrictOwnAudio: true, selfBrowserSurface: 'exclude', surfaceSwitching: 'include'})` | The browser shows its own picker and gives no enumeration. Desktop Chromium, Firefox and Safari. Phones: none on iOS and limited on Android, so phones are **view-only**. |
| **Windows** | Windows.Graphics.Capture, by HWND or HMONITOR (`IGraphicsCaptureItemInterop`); DXGI Desktop Duplication as the monitor fallback | Windows 10 1903+; border and cursor control only on newer builds. C++/WinRT from the Windows SDK (MSVC builds). When sharing a monitor, `SetWindowDisplayAffinity(WDA_EXCLUDEFROMCAPTURE)` keeps our own window out (Windows 10 2004+), so there is no hall of mirrors. |
| **macOS** | ScreenCaptureKit (12.3+): `SCShareableContent` to enumerate, `SCContentFilter` for a display, window or app, with our app excluded; `SCContentSharingPicker` on 14+ | Screen Recording permission (TCC); recent macOS re-prompts periodically. An Objective-C++ TU. |
| **Linux, Wayland** | xdg-desktop-portal ScreenCast over D-Bus, then a PipeWire stream | The portal shows the picker; a `restore_token` skips the prompt next time. Needs libpipewire (cvcpkg has `pipewire`) and a D-Bus client (check cvcpkg). |
| **Linux X11, BSDs** | XComposite for a window, XShm for the screen; `_NET_CLIENT_LIST` to enumerate | |

**Not chosen: FFmpeg's grabbers** (`ddagrab`, `gdigrab`, `avfoundation`, `x11grab`). Their window
choice, self-exclusion and audio are weaker than the native APIs. They are still useful for a one-day
spike.

## 8. SH5: desktop audio

**The stream.** `screen.audio`:
- stereo 48 kHz;
- Opus with `OPUS_APPLICATION_AUDIO` at about 128 kbit/s (64–192 configurable), 20 ms packets;
- DTX for silence instead of the mic's voice gate, and no AGC or noise suppression.

On receive it plays in the stereo mix with its own volume and mute, and it is not spatialised.

**Capture, and leaving out our own playback:**

| Platform | Capture | Our playback left out? |
|---|---|---|
| **Browser** | The `getDisplayMedia` audio track. Tab audio works everywhere in Chromium. Whole-system audio works on Windows when the entire screen is shared, and on macOS 14.2+ from Chrome 141 (a third-party report, **verify**). There is none on Linux and ChromeOS, and none in Firefox or Safari. Delivered to the wasm by an AudioWorklet and encoded by the wasm libopus already deployed. | `restrictOwnAudio: true` filters out the capturing page's own audio. Chromium, Windows and macOS (**verify** the shipping version). |
| **Windows** | WASAPI process loopback (`ActivateAudioInterfaceAsync`). `EXCLUDE_TARGET_PROCESS_TREE` on our own pid gives the whole desktop minus the conference. `INCLUDE_TARGET_PROCESS_TREE` gives one app's audio when sharing one window (HWND → pid). | Yes, on Windows 10 build 20348+ and Windows 11. Older builds only have endpoint loopback, which includes us (D6). |
| **macOS** | ScreenCaptureKit audio (13+): `capturesAudio`, per app through the content filter | `excludesCurrentProcessAudio` |
| **Linux** | v1: the default sink's monitor through SDL3, with `SDL_HINT_AUDIO_INCLUDE_MONITORS`, using the existing `audio_source` and no new capture code. v2: a PipeWire capture stream. | v1: no (D6). v2: route our playback through its own node and capture the rest. |

**Echo from the mic.** If the sharer plays desktop audio on speakers with the mic open, the mic
hears it. The browser's echo canceller cancels only the page's own output. Native has no echo
canceller yet; that is out of scope here. Until then the UI suggests headphones while desktop audio
is shared.

## 9. Sync

**Sync groups.** Each descriptor names a `sync` group: `av` for camera and mic, `screen` for a
screen and its audio.
- The receiver shows a group's video at the pts its audio has reached. This is today's rule, applied
  per group instead of per peer.
- A group with no fresh audio shows the freshest frame, as today.

**Capture-time stamps.** Each share frame and chunk is stamped when it is captured:
- Windows: WGC `SystemRelativeTime` and the WASAPI QPC position;
- macOS: the presentation time of SCK's `CMSampleBuffer`;
- Linux: PipeWire's pts;
- browser: `VideoFrame.timestamp` and `AudioData.timestamp`.

Each source's clock is mapped once to the common host clock: the offset is measured at the first
sample and re-measured every few seconds. All groups share one clock, so a camera and a screen line
up with each other too.

**Target.** Keep audio within the usual broadcast tolerance of the video, roughly 45 ms early to
125 ms late.

**Test.** A synthetic share source flashes a frame and clicks at the same pts. A headless receiver
measures the skew, and CI checks it.

## 10. SH6: UI and room policy (all `.ari`)

**Components** in `src/cvc/ariadne/components/`:
- **`share.ari`**, the Share menu:
  - pick the source: this app, chosen windows, a screen, a window;
  - include audio;
  - quality and fps;
  - a preview;
  - stop.
- **A stage layout:** one share large, with a strip of cameras.
  - Pin or fullscreen a share.
  - Fit it, or show it 1:1 with pan and zoom.
  - On a phone: view only, with pinch zoom.
- **Roster:** a "presenting" badge, per-stream volume in `room_users.ari`, and an "X is sharing"
  notice.

**In the world.** A share can show on a screen in the 3D room. The `type: stream` video-on-quad
already exists, so everyone sees the presenter's screen on the room's wall. This is the feature
other conferencing apps cannot offer.

**Policy.**
- Who may share: everyone for now, and the host only once roles exist (parent §5f).
- `--max-shares`.
- Starting a share while the stage is busy replaces it or joins it, depending on the room's setting.

## 11. Phases

| Phase | Delivers | Needs | Size | Test |
|---|---|---|---|---|
| **SH0** | Named streams, descriptors, gate, relay classes and subscriptions, stereo mixer; a second camera and mic on native | — | M | relay integration with 2+2 streams; bench `--streams` |
| **SH0b** | Browser seam: device choice, an AudioWorklet; a second camera and mic in the browser | SH0 | M | headless Chrome, fake devices (`--use-fake-device-for-media-stream`) |
| **SH1** | `video_encoder` / `video_decoder`: FFmpeg native, WebCodecs browser; `flags`; keyframe requests; keyframe-aware relay; caps; thumbnails | SH0 | L | codec round trip; relay drop-to-keyframe; browser ↔ native interop |
| **SH2** | Priority queue and chunking (WS and gRPC); share rate control | SH1 | M | pose p99 with a share running, on catx-03 netem |
| **SH3a/b** | App share: whole frame, cropped windows, then composed windows; privacy flags | SH0 (MJPEG at low fps), SH1 for full rate | M | golden frames; `share: never` never appears |
| **SH4** | Screen and window capture: browser, then Windows, macOS, Linux | SH1, SH0b | browser S; Windows and macOS M each; Linux L | per-OS manual matrix; synthetic source in CI |
| **SH5** | Desktop audio: browser, Windows, macOS, Linux v1 | SH4 (and the stereo mixer from SH0) | browser S; Windows M; macOS S; Linux S | sync skew test (§9); no own-playback echo |
| **SH6** | Share menu, stage, in-world screen, roster badges, policy | SH3 or SH4 | M | CDP-driven UI run |

**Three tracks that can run in parallel:**
- **A, model and relay:** SH0 → SH2.
- **B, codec:** SH1. Native FFmpeg and browser WebCodecs can be built side by side.
- **C, capture front ends:** SH3a on MJPEG → browser SH4/SH5 → native SH4/SH5, one OS at a time.

**First milestone: "browser screen + system audio".** SH0 + SH0b + the browser half of SH1 + the
browser halves of SH4/SH5 + a minimal share button. A sharer on Chrome or Edge on Windows shows a
screen with its audio to viewers on any browser. The deployed conference is browser-first, so this
reaches users soonest.

## 12. Decisions for the owner

| # | Question | Recommendation |
|---|---|---|
| D1 | Share codec: H.264 (hardware nearly everywhere, WebCodecs in every desktop browser) or royalty-free VP9/AV1 only | **H.264 first, VP9 as the native software fallback**, chosen per stream from the viewers' `caps.decode` |
| D2 | Desktop capture order: browser first, or native first | **Browser first**: the deployed conference is a page; then Windows, macOS, Linux |
| D3 | What a viewer receives of a share by default | **The thumbnail for everyone, full rate once it is on that viewer's stage** |
| D4 | Who may share | **Everyone now**, host-only once roles land |
| D5 | Transport for shares | **The existing WS** with the priority queue and chunking; a second socket only if measured necessary; WebRTC later |
| D6 | Desktop audio where we cannot leave out our own playback (endpoint loopback on old Windows, Linux v1) | **Allow it, with a warning**, and duck our playback while sharing |
| D7 | "Live share" (replicate an `.ari` document instead of pixels) | **Later**, as its own roadmap item |

## 13. Risks and checks

- **Browser devices.** If SDL's emscripten camera and audio see only the default device, the second
  camera or mic in the browser depends on the SH0b seam. Check SDL 3.4's lists first.
- **WebCodecs and track processors.** WebCodecs versions differ per browser, and
  `MediaStreamTrackProcessor` is Chromium-first. Every browser path needs the `new VideoFrame(…)`
  fallback. Check minimum versions before writing the support table.
- **cvc.dll exports** are near the 65,535 cap. New classes stay pimpl or file-local.
- **Relay egress** is N viewers × share bitrate. There is no transcoding, so the CPU stays low, but
  the server's link must be checked for an 8-way room with one 2 Mbit/s share.
- **Permission UX** differs on every platform: the macOS TCC prompt and its periodic re-prompts,
  Wayland portal prompts, the browser's picker on every share.
- **Patents and licences:**
  - libcvc is LGPL-2.1-only, so `ffmpeg-lgpl` stays the FFmpeg we link.
  - Hardware, OS and browser H.264 encoders come with their vendors' licences.
  - A software H.264 encoder that we build ourselves (OpenH264 from source) is not covered by
    Cisco's binary licence, so prefer VP9 for software.
  - WinRT, ScreenCaptureKit and the portal are system APIs.
- **Echo** (§8) stays a known limit until native echo cancellation exists.
