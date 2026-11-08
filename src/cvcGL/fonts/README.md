# The fonts cvcGL's ImGui overlay falls back on

`ImGuiOverlay` draws its UI with a font it resolves when it builds its context (the system's, a
host's or the document's; see `ImGuiOverlay.h`). These two fill in what that font lacks, and are
the whole of the browser builds' fonts, which have no system fonts:

| file | what for | version | sha256 | licence |
|---|---|---|---|---|
| `DejaVuSansMono.ttf` | the browser builds' UI font, and arrows, symbols and box drawing any UI font lacks (`↩ ⇄ ♥ …`) | DejaVu 2.37 (`dejavu-fonts-ttf-2.37.tar.bz2`) | `b4a6c3e4faab8773f4ff761d56451646409f29abedd68f05d38c2df667d3c582` | Bitstream Vera and Arev licences, DejaVu changes public domain: `LICENSE.DejaVu` |
| `Twemoji.Mozilla.ttf` | emoji, in colour (COLR/CPAL, drawn through FreeType) | Mozilla's twemoji-colr v0.7.0 (Twemoji 14) | `6d90152ee0d29e82fe2a87793af5aa4b7ad13e6538360889e141e81ed299ee8e` | the emoji art: CC-BY 4.0, by Twitter, Inc. and other contributors (<https://github.com/twitter/twemoji>); the build: Apache 2.0, Mozilla. `LICENSE.Twemoji.md` |

Sources:

- <https://github.com/dejavu-fonts/dejavu-fonts/releases/tag/version_2_37>
- <https://github.com/mozilla/twemoji-colr/releases/tag/v0.7.0>

They install to `share/libcvc/fonts` with cvcGL. The overlay finds them beside the library or
program that holds cvcGL (`<bin or lib>/../share/libcvc/fonts`), in `CVC_UI_FONT_DIR`, or at the
install's own data directory; the wasm hosts embed them at `/usr/share/libcvc/fonts`. A build tree
gets a copy at `<build>/share/libcvc/fonts`, so its programs find them too.
