# Ariadne media: images from URLs, backgrounds, video and audio

Status: design (2026-10-30). Each section names the PR that builds it. The user-facing
reference moves into [ARIADNE.md](../ARIADNE.md) as each PR lands.

Social clients need several things a document can't do today: a profile with a banner and an
avatar, and a feed of cards whose posts carry images, video and audio. These widgets are
generic and work in every Ariadne host:

- `arigl` in a window (SDL3 + ImGui);
- `ari` with FTXUI and with tvision (Kitty, Sixel or iTerm pixels, or half-blocks);
- the arigl-web and ari.html pages.

Nothing here is specific to one app. The document's own layout (a profile page, a feed) stays
in the app's repository.

## 1. What exists

- `image:` draws a raster the host published with `Backend::set_image(name, img)`, by name:
  `src: NAME`, or `bind:` a key that holds the name. `size:` is the display width in px, and
  the height follows the aspect. ImGui uploads a texture. The terminal backends draw Kitty,
  Sixel or iTerm pixels, or half-blocks (ARIADNE.md section 4, *Pixel graphics*). Hosted
  tvision always draws half-blocks.
- `cvc::image` decodes files only (stb for png/jpg/gif/bmp/tga, ImageMagick for the rest).
- The network: `enable_net` (net_host.h), the allowlist (`NetPolicy`) and the host-held
  credential store. `auth: NAME` adds a bearer token that the document never sees
  (ARIADNE.md section 10.3).
- #914 (L4): `cvc::net::start(req, policy, done)` does not block in single-threaded wasm.
- SDL3 audio output, `cvc::gl::playback::audio_output`, lives in cvcGL. Opus is in core media.

## 2. Images from URLs (PR M1)

```yaml
- image: avatar                       # the label: alt text, and the error/fallback caption
  src: https://media.example/u/42.webp
  auth: example                       # optional: the host adds credential `example`'s token
  size: [48, 48]                      # a box in px; one number is a width (aspect kept)
  fit: cover                          # contain (default) | cover | fill
  shape: circle                       # rect (default) | circle
- image: ""
  bind: feed.items.{i}.media.0.url    # a key or a computed expression whose value is a URL or a name
  size: [480, 270]
```

- **What `src:` can be.** A value with a scheme (`http`, `https`, `file`, `cvc`, `state`) is a
  URI. Anything else is a host-published name, as today. `bind:` takes a state path or a
  computed expression, and its value follows the same rule. So a feed's `repeat:` names each
  post's URL.
- **Fetch.** http(s) goes through the host's policy. The image is drawn only where the host
  turned the network on and allows the origin, so `ari --net-allow https://media.example`.
  `auth:` takes the same path as `http-get`'s `auth`: a bearer header goes only to an origin
  the credential is bound to, and redirects are not followed. The token is never in state,
  in a URL, in an error, or in a cache key; the key uses the credential's *name*.
  `file:`, `cvc:` and `state:` resolve through the URI resolver.
- **Off the frame.** A widget asks the runtime's media cache for `(uri, auth)`, and the cache
  answers at once with *loading*, *ready* or *failed*. The fetch, the decode, the downscale
  and the fit (crop or pad, plus the circle mask) run as one job.
  - Natively, the job runs on the app's background pool.
  - In single-threaded wasm, the request is started with `cvc::net::start` (#914), so the frame
    loop keeps going while it is out. There is no thread to decode on, so the decode runs on the
    frame thread, at most one image per frame. A feed's images then arrive over several frames
    instead of stalling one. Decoding in the browser (`createImageBitmap`) would take even that
    off the frame, and is later work.
  - Each decode is bounded by a size cap: 8192 px a side and 64 MiB of encoded bytes.
  - The frame thread only publishes the finished raster (`set_image`) and draws it.
- **Formats.** PNG, JPEG, GIF (first frame) and BMP decode with stb from memory, and WebP with
  libwebp. libwebp is already a libcvc dependency (for ImageMagick and libtiff); it is linked
  directly and is optional. AVIF and the rest decode through ImageMagick where the build has it.
  A decoded image is downscaled to at most `ariadne.media.max_image_px` (default 2048) on its
  long side.
- **Fit.** The core turns the decoded image into a raster with the box's aspect ratio, so every
  backend draws it unchanged at the box size:
  - `contain` pads the image with transparent pixels;
  - `cover` crops the centre;
  - `fill` stretches;
  - `shape: circle` sets alpha outside the inscribed circle.
  The raster is prepared at twice the box size for HiDPI, capped at the source size.
- **Placeholder and error states.** While loading, the widget draws a neutral raster the size
  of the box (a disc for `circle`), so the layout doesn't move. On failure it draws a darker
  one with the label as its caption, and the error goes in the tooltip, never with a token in
  it. Each state can be read back: `ariadne.media.images.<key>.state` is
  `loading|ready|failed`. Inside a mount, the key is under the mount's `sys.` (section 7).
- **Cache.** The cache lives in memory, per runtime, keyed by `(uri, auth name)`. It holds
  decoded images up to `ariadne.media.cache_mb` (default 64) in LRU order, and evicts entries
  no widget has drawn for 30 s. Requests for the same URL share one fetch (single-flight). A
  failed entry is retried after 30 s, or when the URL changes.
- **Transparency in terminals.** Half-blocks leave a transparent pixel's half of the cell at
  the terminal's own background, so a circle avatar has no black corners on a light terminal.
  Kitty sends RGBA (`f=32`). Sixel and iTerm (JPEG) composite onto black.

## 3. Background images for containers (PR M2)

```yaml
- window: Profile
  background: {src: https://media.example/banner/42.jpg, fit: cover, opacity: 0.6}
  children: [...]
- type: group
  id: banner
  size: [auto, 140]
  background:
    bind: profile.banner_url          # key or computed expression, as image: takes
    auth: example
    fit: cover                        # cover (default) | contain | fill | tile
    opacity: 0.5                      # 0..1 (default 1)
    terminal: behind                  # behind (default) | banner | none
  children: [...]
```

`background:` is a map on any `window`, `overlay` or `group`. It takes the same `src`, `bind`
and `auth` as `image:`, and loads through the same cache.

- **ImGui.** The image is drawn in the container's rectangle, behind the content:
  - in a window, into the window's draw list right after `Begin`;
  - in a group, on channel 0 of a draw-list split, so the group's size is known when it is
    drawn.
  `fit` sets the UVs. `opacity` is the tint alpha.
- **Terminals** (FTXUI, and tvision's hosted bodies):
  - `behind` draws the image as half-block cells under the content, dimmed toward black by
    `opacity`. A cell the content writes text into keeps the dimmed colour as its background,
    with the text in the default foreground, so text stays readable.
  - Pixel protocols can't put pixels under text (Kitty's placeholders are themselves cells, and
    Sixel and iTerm images are drawn over the cells). So `behind` is half-blocks in every
    terminal mode.
  - `banner` draws the image instead as an image row of `banner_rows` (default 6) at the top of
    the container, above the content. That row uses pixels where the terminal has them.
  - `none` draws nothing in a terminal.
- **Fallback.** While the background is loading, or when it fails, nothing is drawn behind the
  content.

## 4. Video and audio (PRs M4, M5, M6)

```yaml
- video: clip                         # the label: caption and fallback text
  src: https://media.example/p/9.mp4
  auth: example
  poster: https://media.example/p/9.jpg   # optional; else the first decoded frame
  size: [480, 270]
  fit: contain
  bind: feed.items.{i}.player         # optional: state the player reports and takes (below)
  autoplay: false
  loop: false
  muted: false
- audio: voice note
  src: https://media.example/p/10.opus
  bind: feed.items.{i}.player
```

- **Controls.** The core draws the controls with the widgets every backend has:
  - a play/pause button;
  - a seek slider;
  - `mm:ss / mm:ss`;
  - mute;
  - "Open" (below).
  So the controls look and work the same in ImGui, FTXUI and tvision.
- **The picture.** `video:` publishes its current frame with `set_image`, as `stream_view`
  does. So it is an ImGui texture, or Kitty, Sixel, iTerm or half-block frames. Frames are
  scaled to the box before they are published. Terminals get at most `ariadne.media.term_fps`
  (default 8) frames a second, and a window gets the stream's own rate.
- **State** under `bind:`, which the document reads and writes:
  - `state`: `idle|loading|ready|playing|paused|ended|failed|remote|unsupported`;
  - `position` and `duration`, in seconds;
  - `playing` (`0`/`1`, writable: writing it plays or pauses);
  - `seek` (writable seconds; consumed);
  - `muted` (writable);
  - `error`.
  With no `bind:`, the state lives under `ariadne.media.players.<key>`.
- **The state machine** is generic and unit-tested without a decoder:
  `idle -> loading -> ready`, then `playing <-> paused`, then `ended`, with `failed` from any
  state. A player that can't play locally goes to `remote` or `unsupported` (below). It lives
  in core libcvc (`cvc::ariadne::media::PlayerState`), and the core holds it.
- **Players behind one interface.** `cvc::ariadne::media::Player` (open, play, pause, seek,
  mute, current frame, clock) is in core. A host registers a factory for it:
  - **Native (PR M5): FFmpeg + SDL3**, in a new GL-free library `cvc_media`. A decode thread
    per player demuxes and decodes with libavformat and libavcodec. Video frames are converted
    with libswscale to RGBA at the box size. Audio is resampled with libswresample to f32 at
    48 kHz stereo, and played through SDL3's audio subsystem. Audio is the master clock, and
    with no audio device the wall clock is. The bytes come from the same fetch as images:
    downloaded whole into memory (64 MiB cap) and read through an AVIOContext. A partial-range
    streaming reader is later work. LGPL linkage: FFmpeg is the cvcpkg `ffmpeg-lgpl` package,
    linked as shared libraries only, and `cvc_media` itself is LGPL-2.1 like libcvc.
  - **Browser (PR M6): the browser's own decoders,** never FFmpeg compiled to wasm. A JS bridge
    in core's wasm build creates a hidden `<video>` or `<audio>` element per player. Its `src`
    is a blob URL of the bytes fetched through cvc::net, so `auth:` and the policy hold. The
    element plays the audio itself, through the browser's audio output. For video, each
    animation frame that has a new picture is drawn into an OffscreenCanvas at the box size,
    and `getImageData` hands RGBA to `set_image`. That's at most `term_fps` frames a second in
    ari.html, and the display rate in arigl-web.
  - **The tradeoff.** The alternative was positioning a real `<video>` over the canvas. That
    is cheaper and sharper, but it only works in arigl-web: ari.html draws a terminal. It also
    breaks ImGui's z-order (a window or menu over the video), and it needs layout to follow the
    widget every frame. Copying frames costs one `getImageData` per shown frame (about 1-3 ms
    at 480x270). In exchange, both pages and every widget path work the same, including
    clipping and overlap. A `<video>` overlay could be an opt-in later for arigl-web.
- **Over SSH.** Audio would play on the remote machine. When the host sees `SSH_CONNECTION`
  (or `SSH_CLIENT`/`SSH_TTY`) and no `--media-play` flag, players start in state `remote`:
  - the poster and the controls draw, but Play is disabled;
  - "Open" runs `(open-url URL)` (#916) where the host has it, and otherwise shows the URL;
  - `ari --media-play` (host flag) forces local playback; `--media-play never` turns it off
    everywhere.
  An `auth:` URL opened in a browser carries no token: the browser needs its own session.
  The widget says so in its tooltip.
- **No player.** A host with no player factory, such as the lean `ari` or a build without
  FFmpeg or SDL, puts players in `unsupported`. They show the poster, "Open", and the reason.

## 5. `ari` with SDL audio, still a terminal program (PR M4)

- SDL3 initialises only `SDL_INIT_AUDIO`: no window, no video subsystem, no display.
  `audio_output` moves from cvcGL into `cvc_media`, as `cvc::media::audio_output`, and cvcGL
  keeps its header as an alias. So the conference app is unchanged.
  `audio_stream_player` moves too. It only needs core's stream types.
- `cvc_media` is a separate library in libcvc's tree, built when SDL3 is found, plus FFmpeg for
  the player. It is installed beside libcvc and found with `find_package(cvc COMPONENTS media)`.
  It links neither GL, VTK nor ImGui. `cvc` itself gains no dependency.
- **The packaging choice:**
  - The default `ari` package stays lean. It links only libcvc, so it has no SDL and no FFmpeg,
    and its players are `unsupported`.
  - A new package, `ari-media`, builds the same source with `ARI_WITH_MEDIA=ON`. It installs
    `bin/ari-media`, and depends on `sdl3` and `ffmpeg-lgpl`, shared.
  - It is a separate binary name, not a second `bin/ari`, so the two packages never conflict
    and a launcher can prefer `ari-media` when it is installed.
  - A plugin that `ari` would `dlopen` was considered and set aside: it is an ABI contract
    with libcvc's exact build on three OSes, for one saved binary name.
- `arigl` already links SDL3. It links `cvc_media` when it is found and registers the player,
  so its media widgets just work.
- The `libcvc` recipe builds `cvc_media` on linux, macos and windows, with `sdl3` and
  `ffmpeg-lgpl` as build dependencies scoped to those platforms.

## 6. A generic `card` component (PR M3)

The file is `src/cvc/ariadne/components/card.ari`. Its `card` unit is a bordered group:

- an optional header row: avatar (`image` with `shape: circle`), title, subtitle and timestamp;
- a body text;
- a media slot: any widget list, such as an `image`, a `video` or an `audio`;
- an action row: any widget list, such as buttons.

Its arguments are state paths and literals, so a `repeat:` over a feed can include it once per
item. It uses only built-in widgets, so it draws in every backend. It's in INDEX.ari and
ARIADNE.md section 6.5.

## 7. Tests (each PR)

- Unit tests:
  - decode from memory: each format, a truncated file, a too-large file, WebP when built;
  - fit and the circle mask (exact pixels);
  - the cache: single-flight, LRU, retry, a key with no token;
  - the player state machine, with a fake player;
  - the SSH default;
  - `auth:` in the media fetch. The token reaches only the bound origin, and is not in any
    error, state key or warning. The test is a fake transport that records each request,
    plus a scan of the whole state tree for the token.
- `ari snapshot` over both terminal backends against a local test server (Python stdlib, as
  net_probe_server.py): an image drawn, a 404 drawn as the error state, a background behind
  a group, and a card with media placeholders. tmux tests for `ari run` (CI runs them since
  #901).
- ImGui: `cvcgl_*` headless tests read back the texture and the draw list.
- wasm: the ari terminal backend's node job loads an image from its probe server; CI builds
  both pages.

## 8. PRs

| PR | scope | depends on |
|---|---|---|
| M0 | this design | none |
| M1 | `image:` from URLs: media cache, fetch with policy/auth, decode from memory (stb, libwebp), fit/shape, placeholder/error, transparency in terminals, tests | none (wasm uses #914's `start` once that lands) |
| M2 | `background:` on window/overlay/group: ImGui draw list, terminal behind/banner | M1 |
| M3 | `card` component | M1 (M4/M5 for its media slot to play) |
| M4 | `cvc_media` library: SDL3 audio output moved out of cvcGL, `ari-media` recipe and option | none |
| M5 | `video:`/`audio:` widgets, player state machine, FFmpeg player, SSH default, `--media-play` | M1, M4 |
| M6 | the browser player (HTMLMediaElement bridge) for arigl-web and ari.html | M5 |
