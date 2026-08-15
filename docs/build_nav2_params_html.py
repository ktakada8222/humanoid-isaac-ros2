#!/usr/bin/env python3
"""Convert docs/nav2_parameters.md -> docs/nav2_parameters.html.

Reuses the same CSS / markdown pipeline / page template as build_doc_html.py
so the visual style matches the main manual. This doc has no images or
mermaid, so those steps are omitted (output stays fully script-free).
"""
import pathlib
import re

import markdown

HERE = pathlib.Path(__file__).resolve().parent
SRC = HERE / "nav2_parameters.md"
DST = HERE / "nav2_parameters.html"
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

# ---- Title (first h1) ----
m = re.search(r"<h1[^>]*>(.*?)</h1>", html_body, re.DOTALL)
title = re.sub(r"<.*?>", " ", m.group(1)).strip() if m else "Nav2 パラメータ設定ガイド"
short_title = "Nav2 パラメータ設定ガイド"

# ---- Page-break per horizontal rule ----
html_body = html_body.replace("<hr />", '<hr class="section-break" />')

# ---- Wrap tables ----
html_body = re.sub(
    r"(<table>.*?</table>)", r'<div class="table-wrap">\1</div>', html_body, flags=re.DOTALL
)

PAGE = f"""<!doctype html>
<html lang="ja">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{short_title} — TRON-KK</title>
<meta name="generator" content="docs/build_nav2_params_html.py">
<style>{CSS}</style>
</head>
<body>

<header class="cover">
  <div class="inner">
    <div class="badge">TRON-KK / Technical Manual</div>
    <h1>humanoid-isaac-ros2<br>Nav2 パラメータ設定ガイド</h1>
    <div class="meta">Nav2 設定値の解説＋ビルド方法 &nbsp;·&nbsp; 第 1.2 版 &nbsp;·&nbsp; 2026-06-19</div>
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
