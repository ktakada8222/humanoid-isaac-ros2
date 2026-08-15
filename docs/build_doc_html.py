#!/usr/bin/env python3
"""Convert docs/environment_setup.md -> docs/environment_setup.html as a
single-file HTML manual matching the g1_vr_teleop.html house style.

The CSS / rendering pipeline is reused verbatim from
  /home/tron/kohira/IsaacLab/scripts/build_g1_doc_html.py
so the visual style stays identical. Only the cover/footer differ.
"""
import base64
import mimetypes
import pathlib
import re

import markdown

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE / "nav2_environment_setup.md"
DST = HERE / "nav2_environment_setup.html"
REF_BUILDER = pathlib.Path(
    "/home/tron/kohira/IsaacLab/scripts/build_g1_doc_html.py"
)

# ---- Reuse the exact CSS from the reference builder so the style matches ----
ref_src = REF_BUILDER.read_text(encoding="utf-8")
m_css = re.search(r'CSS = r"""(.*?)"""', ref_src, re.DOTALL)
if not m_css:
    raise SystemExit("could not extract CSS from reference builder")
CSS = m_css.group(1)

md_text = SRC.read_text(encoding="utf-8")

# ---- Swap the nav-architecture marker block (which contains a ```mermaid block
#      so GitHub renders it natively) with a pre-rendered PNG, BEFORE the mermaid
#      stashing below, so the HTML build stays fully script-free (mail-safe). ----
_nav_png = HERE / "assets" / "nav_flow.png"
if _nav_png.is_file():
    _nb64 = base64.b64encode(_nav_png.read_bytes()).decode("ascii")
    _nav_img = (
        '\n\n<p><img alt="ナビゲーション構成" '
        f'src="data:image/png;base64,{_nb64}"></p>\n\n'
    )
    md_text = re.sub(
        r"<!--\s*NAV_IMG_START\s*-->.*?<!--\s*NAV_IMG_END\s*-->",
        lambda m: _nav_img, md_text, flags=re.DOTALL,
    )

# ---- Stash mermaid blocks before markdown processing ----
_mermaid_blocks = []


def _stash_mermaid(m):
    _mermaid_blocks.append(m.group(1))
    return f"\n\n<!--MERMAID_BLOCK_{len(_mermaid_blocks) - 1}-->\n\n"


md_text = re.sub(r"```mermaid\n(.*?)\n```", _stash_mermaid, md_text, flags=re.DOTALL)

# ---- Render body ----
md = markdown.Markdown(
    extensions=["tables", "fenced_code", "toc", "codehilite", "attr_list", "sane_lists"],
    extension_configs={
        "toc": {"title": "目次", "anchorlink": True, "permalink": "", "toc_depth": "2-3"},
        "codehilite": {"guess_lang": False, "noclasses": True, "pygments_style": "manni"},
    },
)
html_body = md.convert(md_text)
toc_html = getattr(md, "toc", "")

# ---- Embed local images as base64 ----
def _embed_img(match):
    src = match.group(1)
    if src.startswith(("http://", "https://", "data:")):
        return match.group(0)
    img_path = (SRC.parent / src).resolve()
    if not img_path.is_file():
        return match.group(0)
    mime, _ = mimetypes.guess_type(str(img_path))
    if mime is None:
        mime = "image/png"
    b64 = base64.b64encode(img_path.read_bytes()).decode("ascii")
    return match.group(0).replace(f'src="{src}"', f'src="data:{mime};base64,{b64}"')


html_body = re.sub(r'<img [^>]*src="([^"]+)"[^>]*>', _embed_img, html_body)


# ---- Embed local <video>/<source> media as base64 (keeps the HTML self-contained) ----
def _embed_media(match):
    src = match.group(1)
    if src.startswith(("http://", "https://", "data:")):
        return match.group(0)
    media_path = (SRC.parent / src).resolve()
    if not media_path.is_file():
        return match.group(0)
    mime, _ = mimetypes.guess_type(str(media_path))
    if mime is None:
        mime = "video/webm" if src.endswith(".webm") else "video/mp4"
    b64 = base64.b64encode(media_path.read_bytes()).decode("ascii")
    return match.group(0).replace(f'src="{src}"', f'src="data:{mime};base64,{b64}"')


html_body = re.sub(r'<(?:source|video) [^>]*src="([^"]+)"[^>]*>', _embed_media, html_body)

# ---- Replace the build-flow marker block with the compact diagram image (HTML only).
#      The Markdown source keeps a plain-text flow between the markers so the .md stays
#      portable; here we swap that block for the embedded PNG in the HTML build. ----
_flow_png = HERE / "assets" / "build_flow.png"
if _flow_png.is_file():
    _b64 = base64.b64encode(_flow_png.read_bytes()).decode("ascii")
    _img = (
        '<p><img alt="構築フェーズの全体像" '
        f'src="data:image/png;base64,{_b64}"></p>'
    )
    html_body = re.sub(
        r"<!--\s*FLOW_IMG_START\s*-->.*?<!--\s*FLOW_IMG_END\s*-->",
        _img, html_body, flags=re.DOTALL,
    )

# ---- Restore mermaid blocks ----
for i, code in enumerate(_mermaid_blocks):
    placeholder = f"<!--MERMAID_BLOCK_{i}-->"
    replacement = f'<pre class="mermaid">{code}</pre>'
    html_body = html_body.replace(f"<p>{placeholder}</p>", replacement)
    html_body = html_body.replace(placeholder, replacement)
has_mermaid = bool(_mermaid_blocks)

# ---- Title (first h1) ----
m = re.search(r"<h1[^>]*>(.*?)</h1>", html_body, re.DOTALL)
title = re.sub(r"<.*?>", " ", m.group(1)).strip() if m else "Document"
short_title = title.split("構築")[0].rstrip() if "構築" in title else title

# ---- Page-break per horizontal rule ----
html_body = html_body.replace("<hr />", '<hr class="section-break" />')

# ---- Wrap tables ----
html_body = re.sub(
    r"(<table>.*?</table>)", r'<div class="table-wrap">\1</div>', html_body, flags=re.DOTALL
)

MERMAID_SCRIPT = (
    """
<script type="module">
  import mermaid from 'https://cdn.jsdelivr.net/npm/mermaid@10/dist/mermaid.esm.min.mjs';
  const isDark = window.matchMedia && window.matchMedia('(prefers-color-scheme: dark)').matches;
  mermaid.initialize({
    startOnLoad: true,
    theme: isDark ? 'dark' : 'default',
    securityLevel: 'loose',
    flowchart: { htmlLabels: true, curve: 'basis' },
  });
</script>
"""
    if has_mermaid
    else ""
)

PAGE = f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{short_title} — TRON-KK</title>
<meta name="generator" content="docs/build_doc_html.py">
<style>{CSS}</style>
{MERMAID_SCRIPT}
</head>
<body>

<header class="cover">
  <div class="inner">
    <div class="badge">TRON-KK / Technical Manual</div>
    <h1>humanoid-isaac-ros2<br>Isaac Sim ナビゲーション環境 構築・ナビ実行ガイド</h1>
    <div class="meta">環境構築＋ナビ実行編 &nbsp;·&nbsp; 第 1.25 版 &nbsp;·&nbsp; 2026-06-17</div>
  </div>
</header>

<div class="layout">

  <aside class="toc-side">
    <div class="toc-heading">Contents</div>
    {toc_html}
  </aside>

  <main>
{html_body}
    <footer class="doc-footer">
      <p>© 2026 トロン株式会社 (TRON K.K.) All Rights Reserved. &nbsp;·&nbsp; 本書の無断転載・複製を禁じます。</p>
    </footer>
  </main>

</div>

</body>
</html>
"""

DST.write_text(PAGE, encoding="utf-8")
print(f"wrote {DST}")
print(f"  body: {len(html_body):,} bytes")
print(f"  full: {len(PAGE):,} bytes")
